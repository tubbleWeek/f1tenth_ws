import numpy as np
import math
import copy
import numba
import time
import sys 

import matplotlib.pyplot as plt
import pdb

import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

# Import pyCuda modules for computations
from pycuda.curandom import XORWOWRandomNumberGenerator
from pycuda import gpuarray

# Information about your GPU
# Initialize PyCUDA and create a primary context
cuda.init()
device = cuda.Device(0)
primary_context = device.retain_primary_context()
primary_context.push()

from numba import cuda as numba_cuda
from numba.cuda.random import create_xoroshiro128p_states, xoroshiro128p_normal_float32
from .cuda_device_functions import *
from .input_constraints import *
# import utils.mppi_nln_utils as uls 
gpu = numba_cuda.get_current_device()
print(numba_cuda.is_available())
max_threads_per_block = gpu.MAX_THREADS_PER_BLOCK
max_square_block_dim = (int(gpu.MAX_BLOCK_DIM_X**0.5), int(gpu.MAX_BLOCK_DIM_X**0.5))
max_blocks = gpu.MAX_GRID_DIM_X
max_rec_blocks = rec_max_control_rollouts = int(1e6) # Though theoretically limited by max_blocks on GPU
rec_min_control_rollouts = 100

import os


class Config:
  
  """ Configurations that are typically fixed throughout execution. """
  
  def __init__(self, 
               T=5, # Horizon (s)
               dt=0.1, # Length of each step (s)
               num_control_rollouts=16384, # Number of control sequences
               num_vis_state_rollouts=16384, # Number of visualization rollouts
               seed=1):
    
    self.seed = seed
    self.T = T
    self.dt = dt
    self.num_steps = int(T/dt)
    self.max_threads_per_block = max_threads_per_block # save just in case

    assert T > 0
    assert dt > 0
    assert T > dt
    assert self.num_steps > 0
    
    # Number of control rollouts are currently limited by the number of blocks
    self.num_control_rollouts = num_control_rollouts
    if self.num_control_rollouts > rec_max_control_rollouts:
      self.num_control_rollouts = rec_max_control_rollouts
      print("MPPI Config: Clip num_control_rollouts to be recommended max number of {}. (Max={})".format(
        rec_max_control_rollouts, max_blocks))
    elif self.num_control_rollouts < rec_min_control_rollouts:
      self.num_control_rollouts = rec_min_control_rollouts
      print("MPPI Config: Clip num_control_rollouts to be recommended min number of {}. (Recommended max={})".format(
        rec_min_control_rollouts, rec_max_control_rollouts))
    
    # For visualizing state rollouts
    self.num_vis_state_rollouts = num_vis_state_rollouts
    self.num_vis_state_rollouts = min([self.num_vis_state_rollouts, self.num_control_rollouts])
    self.num_vis_state_rollouts = max([1, self.num_vis_state_rollouts])

DEFAULT_OBS_COST = 1e4
DEFAULT_DIST_WEIGHT = 1e6


class MPPI_Numba(object):
  
  """ 
  Implementation of Information theoretic MPPI by Williams et. al. 
  Alg 2. in https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf


  Planner object that initializes GPU memory and runs MPPI on GPU via numba. 
  
  Typical workflow: 
    1. Initialize object with config that allows pre-initialization of GPU memory
    2. reset()
    3. setup(mppi_params) based on problem instance
    4. solve(), which returns optimized control sequence
    5. get_state_rollout() for visualization
    6. shift_and_update(next_state, optimal_u_sequence, num_shifts=1)
    7. Repeat from 2 if params have changed
  """

  def __init__(self, cfg):

    # Fixed configs
    self.cfg = cfg
    self.T = cfg.T
    self.dt = cfg.dt
    self.num_steps = cfg.num_steps
    self.num_control_rollouts = cfg.num_control_rollouts

    self.num_vis_state_rollouts = cfg.num_vis_state_rollouts
    self.seed = cfg.seed
    self.vehicle_length = 0.57
    self.vehicle_width = 0.3
    self.vehicle_wheelbase = 0.32
    # Basic info 
    self.max_threads_per_block = cfg.max_threads_per_block

    # Initialize reuseable device variables
    self.noise_samples_d = None
    self.u_cur_d = None
    self.u_prev_d = None
    self.costs_d = None
    self.weights_d = None
    self.rng_states_d = None
    self.state_rollout_batch_d = None # For visualization only. Otherwise, inefficient

    # Other task specific params
    self.device_var_initialized = False

    self.generator = XORWOWRandomNumberGenerator()
    self.mppi_type = 0 # Normal dist / 1: NLN
    if self.mppi_type == 1:
      self.mu_LogN, self.std_LogN = uls.Normal2LogN(0, np.mean([0.045, 0.047]))
      self.LogN_info = [self.mppi_type, self.mu_LogN, self.std_LogN]

    # local costmap size and resolution
    self.local_costmap_size = 200
    self.local_costmap_resolution = 0.05
    self.reset()
    
  def reset(self):
    # Other task specific params
    self.u_seq0 = np.zeros((self.num_steps, 2), dtype=np.float32)
    self.params = None
    self.params_set = False
    self.costmap_loaded = False
    self.u_prev_d = None
    
    # Initialize all fixed-size device variables ahead of time. (Do not change in the lifetime of MPPI object)
    self.init_device_vars_before_solving()


  def init_device_vars_before_solving(self):

    if not self.device_var_initialized:
      t0 = time.time()
      
      self.noise_samples_d = numba_cuda.device_array((self.num_control_rollouts, self.num_steps, 2), dtype=np.float32) # to be sampled collaboratively via GPU
      self.u_cur_d = numba_cuda.to_device(self.u_seq0) 
      self.u_prev_d = numba_cuda.to_device(self.u_seq0) 
      self.costs_d = numba_cuda.device_array((self.num_control_rollouts), dtype=np.float32)
      self.weights_d = numba_cuda.device_array((self.num_control_rollouts), dtype=np.float32)
      self.rng_states_d = create_xoroshiro128p_states(self.num_control_rollouts*self.num_steps, seed=self.seed)
      
      self.state_rollout_batch_d = numba_cuda.device_array((self.num_vis_state_rollouts, self.num_steps+1, 5), dtype=np.float32)   
      # self.local_costmap_d = numba_cuda.device_array((self.local_costmap_size, self.local_costmap_size), dtype=np.float32)   
      self.device_var_initialized = True
      print("MPPI planner has initialized GPU memory after {} s".format(time.time()-t0))


  def setup(self, params):
    # These tend to change (e.g., current robot position, the map) after each step
    self.set_params(params)


  def set_params(self, params):
    self.params = copy.deepcopy(params)
    self.params_set = True
    self.load_costmap(self.params['costmap'])

  def load_costmap(self, costmap):
    # COSTMAP
    costmap_full = costmap.astype(np.float32)
    # Padding the costmap to avoid index out of bounds
    self.costmap_full_padded = np.pad(costmap_full, self.local_costmap_size//2, 'constant', constant_values=np.max(costmap_full)*2)
    self.costmap_loaded = True

  def check_solve_conditions(self):
    if not self.params_set:
      print("MPPI parameters are not set. Cannot solve")
      return False
    if not self.device_var_initialized:
      print("Device variables not initialized. Cannot solve.")
      return False
    if not self.costmap_loaded:
      print("Costmap not loaded. Cannot solve.")
      return False
    return True

  def solve(self):
    """Entry point for different algoritims"""
    
    if not self.check_solve_conditions():
      print("MPPI solve condition not met. Cannot solve. Return")
      return
    
    return self.solve_with_nominal_dynamics()

  def convert_position_to_costmap_indices(self, position): 
    map_resolution = 0.05
    origin = [-15, -10]
    map_y = int((position[0] - origin[0]) / map_resolution)
    map_x = int((position[1] - origin[1] ) / map_resolution)
    return map_x, map_y

  def random_noise_sample(self):
    # Use the random generator to generate random noise
    # The logic from log-MPPI_ros github repo
    if self.mppi_type == 0: # Normal Dist
      du_d = self.generator.gen_normal(
          self.num_control_rollouts * self.num_steps * 2,
          np.float32)

    # log-MPPI
    elif self.mppi_type == 1: # NLN
        print('NLN IS USED FOR NOISE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        du_LogN_d = self.generator.gen_log_normal(
            self.num_control_rollouts * self.num_steps * 2,
            np.float32, self.mu_LogN, self.std_LogN)
        du_d = du_LogN_d * self.generator.gen_normal(
            self.num_control_rollouts * self.num_steps * 2,
            np.float32)
        
    return du_d.get()

  def move_mppi_task_vars_to_device(self):
    vrange_d = numba_cuda.to_device(self.params['vrange'].astype(np.float32))
    wrange_d = numba_cuda.to_device(self.params['wrange'].astype(np.float32))
    xgoal_d = numba_cuda.to_device(self.params['xgoal'].astype(np.float32))
    goal_tolerance_d = np.float32(self.params['goal_tolerance'])
    lambda_weight_d = np.float32(self.params['lambda_weight'])

    u_std_d = numba_cuda.to_device(self.params['u_std'].astype(np.float32))
    self.u_std = self.params['u_std'].astype(np.float32)
    x0_d = numba_cuda.to_device(self.params['x0'].astype(np.float32))
    dt_d = np.float32(self.params['dt'])
    vehicle_length_d = np.float32(self.vehicle_length)
    vehicle_width_d = np.float32(self.vehicle_width)
    vehicle_wheelbase_d = np.float32(self.vehicle_wheelbase)
    v_switch_d = np.float32(self.params['v_switch'])
    a_max_d = np.float32(self.params['a_max'])
    delta_range_d = numba_cuda.to_device(self.params['delta_range'].astype(np.float32))
    # # COSTMAP
    # costmap_full = self.params['costmap'].astype(np.float32)
    # # Padding the costmap to avoid index out of bounds
    # costmap_full = np.pad(costmap_full, self.local_costmap_size//2, 'constant', constant_values=1)
    map_x, map_y = self.convert_position_to_costmap_indices(self.params['x0'][:2])
    # get the local costmap with the robot at the center
    local_costmap = self.costmap_full_padded[map_x - self.local_costmap_size//2:map_x + self.local_costmap_size//2,
                                  map_y - self.local_costmap_size//2:map_y + self.local_costmap_size//2]
    # make the local_costmap contigous
    local_costmap = np.ascontiguousarray(local_costmap)
    max_local_cost_d = np.float32(np.max(local_costmap))
    # set the local costmap to the local_costmap_d on the device
    local_costmap_d = numba_cuda.to_device(local_costmap)
    obs_cost_d = np.float32(DEFAULT_OBS_COST if 'obs_penalty' not in self.params 
                                  else self.params['obs_penalty'])
    return vrange_d, wrange_d, xgoal_d, \
           goal_tolerance_d, lambda_weight_d, \
           vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d, \
           v_switch_d, a_max_d, delta_range_d, \
           u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d

  def solve_with_nominal_dynamics(self):
    """
    Launch GPU kernels that use nominal dynamics but adjsuts cost function based on worst-case linear speed.
    """
    
    vrange_d, wrange_d, xgoal_d, goal_tolerance_d, lambda_weight_d, \
           vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d,\
            v_switch_d, a_max_d, delta_range_d,\
              u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d = self.move_mppi_task_vars_to_device()
  
    # Weight for distance cost
    dist_weight = DEFAULT_DIST_WEIGHT if 'dist_weight' not in self.params else self.params['dist_weight']

    # Optimization loop
    for k in range(self.params['num_opt']):

      # Sample control noise
      noise_samples = self.random_noise_sample()
      # reshape the noise samples to (num_control_rollouts, num_steps, 2)
      noise_samples_reshaped = noise_samples.reshape(self.num_control_rollouts, self.num_steps, 2).astype(np.float32)
      noise_samples_reshaped[:,:,0] *= u_std_d[0]
      noise_samples_reshaped[:,:,1] *= u_std_d[1]

      self.noise_samples_d = numba_cuda.to_device(noise_samples_reshaped)


      # self.sample_noise_numba[self.num_control_rollouts, self.num_steps](
      #       self.rng_states_d, u_std_d, self.noise_samples_d)
      

      
      # Rollout and compute mean or cvar
      self.rollout_numba[self.num_control_rollouts, 1](
        vrange_d,
        wrange_d,
        xgoal_d,
        local_costmap_d, # local_costmap added
        max_local_cost_d,
        obs_cost_d,
        vehicle_length_d,
        vehicle_width_d,
        vehicle_wheelbase_d,
        goal_tolerance_d,
        lambda_weight_d,
        u_std_d,
        x0_d,
        dt_d,
        dist_weight,
        v_switch_d, 
        a_max_d,
        delta_range_d,
        self.noise_samples_d,
        self.u_cur_d,

        # results
        self.costs_d
      )

      self.u_prev_d = self.u_cur_d

      # Compute cost and update the optimal control on device
      self.update_useq_numba[1, 32](
        lambda_weight_d, 
        self.costs_d, 
        self.noise_samples_d, 
        self.weights_d, 
        vrange_d,
        wrange_d,
        self.u_cur_d
      )

    return self.u_cur_d.copy_to_host()


  def shift_and_update(self, new_x0, u_cur, num_shifts=1):
    self.params["x0"] = new_x0.copy()
    self.shift_optimal_control_sequence(u_cur, num_shifts)


  def shift_optimal_control_sequence(self, u_cur, num_shifts=1):
    u_cur_shifted = u_cur.copy()
    u_cur_shifted[:-num_shifts] = u_cur_shifted[num_shifts:]
    self.u_cur_d = numba_cuda.to_device(u_cur_shifted.astype(np.float32))


  def get_state_rollout(self):
    """
    Generate state sequences based on the current optimal control sequence.
    """

    assert self.params_set, "MPPI parameters are not set"

    if not self.device_var_initialized:
      print("Device variables not initialized. Cannot run mppi.")
      return
    
    # Move things to GPU
    vrange_d = numba_cuda.to_device(self.params['vrange'].astype(np.float32))
    wrange_d = numba_cuda.to_device(self.params['wrange'].astype(np.float32))
    x0_d = numba_cuda.to_device(self.params['x0'].astype(np.float32))
    dt_d = np.float32(self.params['dt'])
    v_switch_d = np.float32(self.params['v_switch'])
    a_max_d = np.float32(self.params['a_max'])
    delta_range_d = numba_cuda.to_device(self.params['delta_range'].astype(np.float32))
    vehicle_wheelbase_d = np.float32(self.vehicle_wheelbase)

    self.get_state_rollout_across_control_noise[self.num_vis_state_rollouts, 1](
        self.state_rollout_batch_d, # where to store results
        x0_d, 
        dt_d,
        self.noise_samples_d,
        vrange_d,
        wrange_d,
        v_switch_d,
        a_max_d,
        delta_range_d,
        vehicle_wheelbase_d,
        self.u_prev_d,
        self.u_cur_d,
        )
    
    return self.state_rollout_batch_d.copy_to_host()

  def get_vehicle_boundary_points_p(self, x_curr, vehicle_length, vehicle_width):
    x_center, y_center, _, _, theta = x_curr
    # Half dimensions
    half_length = vehicle_length / 2
    half_width = vehicle_width / 2

    # Define the relative positions of the corners
    corners = np.array([
        [half_length, half_width],     # Front left
        [half_length, -half_width],    # Front right
        [-half_length, -half_width],   # Rear right
        [-half_length, half_width]     # Rear left
    ])

    # Compute the rotation matrix based on heading angle (theta)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    rotation_matrix = np.array([
        [cos_theta, -sin_theta],
        [sin_theta, cos_theta]
    ])

    # Rotate corners by the heading angle and translate to world coordinates
    world_corners = rotation_matrix @ corners.T 
    world_corners = world_corners.T + np.array([x_center, y_center])
    # Add first point to the end for visualization
    world_corners = np.vstack([world_corners, world_corners[0]])
    return world_corners
  

  """GPU kernels from here on"""

  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def rollout_numba(
          vrange_d, 
          wrange_d, 
          xgoal_d, 
          local_costmap_d,
          max_local_cost_d, # local_costmap added
          obs_cost_d,
          vehicle_length_d,
          vehicle_width_d,
          vehicle_wheelbase_d,         
          goal_tolerance_d, 
          lambda_weight_d, 
          u_std_d, 
          x0_d, 
          dt_d,
          dist_weight_d,
          v_switch_d,
          a_max_d,
          delta_range_d,
          noise_samples_d,
          u_cur_d,
          costs_d):
    """
    There should only be one thread running in each block, where each block handles a single sampled control sequence.
    """


    # Get block id and thread id
    bid = numba_cuda.blockIdx.x   # index of block
    tid = numba_cuda.threadIdx.x  # index of thread within a block
    costs_d[bid] = 0.0

    # Explicit unicycle update and map lookup
    # From here on we assume grid is properly padded so map lookup remains valid
    x_curr = numba_cuda.local.array(5, numba.float32) # x, y, delta, v, theta

    for i in range(5): 
      x_curr[i] = x0_d[i]

    timesteps = len(u_cur_d)
    goal_reached = False
    goal_tolerance_d2 = goal_tolerance_d*goal_tolerance_d
    dist_to_goal2 = 1e9
    v_nom =v_noisy = w_nom = w_noisy = 0.0

    # Allocate space for vehicle boundary points (4)
    vehicle_boundary_points_d = numba_cuda.local.array((4, 2), dtype=np.float32)
    vehicle_boundary_points__grid_d = numba_cuda.local.array((4, 2), dtype=np.float32)
    # printed=False
    gamma = 0.98
    for t in range(timesteps):
      # Nominal noisy control
      v_nom = u_cur_d[t, 0] + noise_samples_d[bid, t, 0]
      w_nom = u_cur_d[t, 1] + noise_samples_d[bid, t, 1]
      v_noisy = max(vrange_d[0], min(vrange_d[1], v_nom))
      w_noisy = max(wrange_d[0], min(wrange_d[1], w_nom))
      
      # Forward simulate
      # Kinematic model update
      x_curr[0] += dt_d*x_curr[3]*math.cos(x_curr[4])
      x_curr[1] += dt_d*x_curr[3]*math.sin(x_curr[4])
      x_curr[2] += dt_d*steering_contrainsts_cuda(x_curr[2],w_noisy, delta_range_d[0], delta_range_d[1], wrange_d[0], wrange_d[1])
      x_curr[3] += dt_d*acceleration_contrainsts_cuda(x_curr[3], v_noisy, vrange_d[0], vrange_d[1], a_max_d, v_switch_d)
      x_curr[4] += dt_d*(x_curr[3]/vehicle_wheelbase_d)*math.tan(x_curr[2])
      x_curr[4] = math.fmod(x_curr[4], 2*math.pi)

      # If else statements will be expensive
      dist_to_goal2 = (xgoal_d[0]-x_curr[0])**2 + (xgoal_d[1]-x_curr[1])**2
      costs_d[bid]+= stage_cost(dist_to_goal2, dist_weight_d) * gamma

      # Compute vehicle boundary points for the current state
      get_vehicle_boundary_points(x_curr, vehicle_length_d, vehicle_width_d, vehicle_boundary_points_d)
      
      # Convert vehicle boundary points to costmap indices
      # -15:x_min, -10:y_min, 0.05:grid_resolution 10:scaling factor
      get_vehicle_boundary_points_grid(vehicle_boundary_points_d, -15.0, -10.0, 0.05, 10.0, vehicle_boundary_points__grid_d)
      
      # Add obstacle costs

      costs_d[bid] +=  (calculate_obstacle_cost(vehicle_boundary_points_d, obs_cost_d, max_local_cost_d, local_costmap_d) / 4 )* gamma 
      gamma *= 0.95

      if dist_to_goal2<= goal_tolerance_d2:
        goal_reached = True
        break
    
    # Accumulate terminal cost 
    costs_d[bid] += term_cost(dist_to_goal2, goal_reached)

    for t in range(timesteps):
      costs_d[bid] += lambda_weight_d*(
              (u_cur_d[t,0]/(u_std_d[0]**2))*noise_samples_d[bid, t,0] + (u_cur_d[t,1]/(u_std_d[1]**2))*noise_samples_d[bid, t, 1])

  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def update_useq_numba(
        lambda_weight_d,
        costs_d,
        noise_samples_d,
        weights_d,
        vrange_d,
        wrange_d,
        u_cur_d):
    """
    GPU kernel that updates the optimal control sequence based on previously evaluated cost values.
    Assume that the function is invoked as update_useq_numba[1, NUM_THREADS], with one block and multiple threads.
    """

    tid = numba_cuda.threadIdx.x
    num_threads = numba_cuda.blockDim.x
    numel = len(noise_samples_d)
    gap = int(math.ceil(numel / num_threads))

    # Find the minimum value via reduction
    starti = min(tid*gap, numel)
    endi = min(starti+gap, numel)
    if starti<numel:
      weights_d[starti] = costs_d[starti]
    for i in range(starti, endi):
      weights_d[starti] = min(weights_d[starti], costs_d[i])
    numba_cuda.syncthreads()

    s = gap
    while s < numel:
      if (starti % (2 * s) == 0) and ((starti + s) < numel):
        # Stride by `s` and add
        weights_d[starti] = min(weights_d[starti], weights_d[starti + s])
      s *= 2
      numba_cuda.syncthreads()

    beta = weights_d[0]
    
    # Compute weight
    for i in range(starti, endi):
      weights_d[i] = math.exp(-1./lambda_weight_d*(costs_d[i]-beta))
    numba_cuda.syncthreads()

    # Normalize
    # Reuse costs_d array
    for i in range(starti, endi):
      costs_d[i] = weights_d[i]
    numba_cuda.syncthreads()
    for i in range(starti+1, endi):
      costs_d[starti] += costs_d[i]
    numba_cuda.syncthreads()
    s = gap
    while s < numel:
      if (starti % (2 * s) == 0) and ((starti + s) < numel):
        # Stride by `s` and add
        costs_d[starti] += costs_d[starti + s]
      s *= 2
      numba_cuda.syncthreads()

    for i in range(starti, endi):
      weights_d[i] /= costs_d[0]
    numba_cuda.syncthreads()
    
    # update the u_cur_d
    timesteps = len(u_cur_d)
    for t in range(timesteps):
      for i in range(starti, endi):
        numba_cuda.atomic.add(u_cur_d, (t, 0), weights_d[i]*noise_samples_d[i, t, 0])
        numba_cuda.atomic.add(u_cur_d, (t, 1), weights_d[i]*noise_samples_d[i, t, 1])
    numba_cuda.syncthreads()

    # Blocks crop the control together
    tgap = int(math.ceil(timesteps / num_threads))
    starti = min(tid*tgap, timesteps)
    endi = min(starti+tgap, timesteps)
    for ti in range(starti, endi):
      u_cur_d[ti, 0] = max(vrange_d[0], min(vrange_d[1], u_cur_d[ti, 0]))
      u_cur_d[ti, 1] = max(wrange_d[0], min(wrange_d[1], u_cur_d[ti, 1]))


  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def get_state_rollout_across_control_noise(
          state_rollout_batch_d, # where to store results
          x0_d, 
          dt_d,
          noise_samples_d,
          vrange_d,
          wrange_d,
          v_switch_d,
          a_max_d,
          delta_range_d,
          vehicle_wheelbase_d,
          u_prev_d,
          u_cur_d):
    """
    Do a fixed number of rollouts for visualization across blocks.
    Assume kernel is launched as get_state_rollout_across_control_noise[num_blocks, 1]
    The block with id 0 will always visualize the best control sequence. Other blocks will visualize random samples.
    """
    
    # Use block id
    tid = numba_cuda.threadIdx.x
    bid = numba_cuda.blockIdx.x
    timesteps = len(u_cur_d)


    if bid==0:
      # Visualize the current best 
      # Explicit unicycle update and map lookup
      # From here on we assume grid is properly padded so map lookup remains valid
      x_curr = numba_cuda.local.array(5, numba.float32) # x, y, delta, v, theta

      for i in range(5): 
        x_curr[i] = x0_d[i]
        state_rollout_batch_d[bid,0,i] = x0_d[i]
      
      for t in range(timesteps):
        # Nominal noisy control
        v_nom = u_cur_d[t, 0]
        w_nom = u_cur_d[t, 1]
        
        # Forward simulate
        # Kinematic model update
        x_curr[0] += dt_d*x_curr[3]*math.cos(x_curr[4])
        x_curr[1] += dt_d*x_curr[3]*math.sin(x_curr[4])
        x_curr[2] += dt_d*steering_contrainsts_cuda(x_curr[2], w_nom, delta_range_d[0], delta_range_d[1], wrange_d[0], wrange_d[1])
        x_curr[3] += dt_d*acceleration_contrainsts_cuda(x_curr[3], v_nom, vrange_d[0], vrange_d[1], a_max_d, v_switch_d)
        x_curr[4] += dt_d*(x_curr[3]/vehicle_wheelbase_d)*math.tan(x_curr[2])
        x_curr[4] = math.fmod(x_curr[4], 2*math.pi)
        # Save state
        state_rollout_batch_d[bid,t+1,0] = x_curr[0]
        state_rollout_batch_d[bid,t+1,1] = x_curr[1]
        state_rollout_batch_d[bid,t+1,2] = x_curr[2]
        state_rollout_batch_d[bid,t+1,3] = x_curr[3]
        state_rollout_batch_d[bid,t+1,4] = x_curr[4]
    else:
      
      # Explicit unicycle update and map lookup
      # From here on we assume grid is properly padded so map lookup remains valid
      x_curr = numba_cuda.local.array(5, numba.float32)
      for i in range(5): 
        x_curr[i] = x0_d[i]
        state_rollout_batch_d[bid,0,i] = x0_d[i]

      
      for t in range(timesteps):
        # Nominal noisy control
        v_nom = u_prev_d[t, 0] + noise_samples_d[bid, t, 0]
        w_nom = u_prev_d[t, 1] + noise_samples_d[bid, t, 1]
        v_noisy = max(vrange_d[0], min(vrange_d[1], v_nom))
        w_noisy = max(wrange_d[0], min(wrange_d[1], w_nom))

        # # Nominal noisy control
        v_nom = u_prev_d[t, 0]
        w_nom = u_prev_d[t, 1]
        
        # Kinematic model update
        x_curr[0] += dt_d*x_curr[3]*math.cos(x_curr[4])
        x_curr[1] += dt_d*x_curr[3]*math.sin(x_curr[4])
        x_curr[2] += dt_d*steering_contrainsts_cuda(x_curr[2],w_noisy, delta_range_d[0], delta_range_d[1], wrange_d[0], wrange_d[1])
        x_curr[3] += dt_d*acceleration_contrainsts_cuda(x_curr[3], v_noisy, vrange_d[0], vrange_d[1], a_max_d, v_switch_d)
        x_curr[4] += dt_d*(x_curr[3]/vehicle_wheelbase_d)*math.tan(x_curr[2])
        x_curr[4] = math.fmod(x_curr[4], 2*math.pi)
        # Save state
        state_rollout_batch_d[bid,t+1,0] = x_curr[0]
        state_rollout_batch_d[bid,t+1,1] = x_curr[1]
        state_rollout_batch_d[bid,t+1,2] = x_curr[2]
        state_rollout_batch_d[bid,t+1,3] = x_curr[3]
        state_rollout_batch_d[bid,t+1,4] = x_curr[4]

  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def sample_noise_numba(rng_states, u_std_d, noise_samples_d):
    """
    Should be invoked as sample_noise_numba[NUM_U_SAMPLES, NUM_THREADS].
    noise_samples_d.shape is assumed to be (num_rollouts, time_steps, 2)
    Assume each thread corresponds to one time step
    For consistency, each block samples a sequence, and threads (not too many) work together over num_steps.
    This will not work if time steps are more than max_threads_per_block (usually 1024)
    """
    block_id = numba_cuda.blockIdx.x
    thread_id = numba_cuda.threadIdx.x
    abs_thread_id = numba_cuda.grid(1)

    noise_samples_d[block_id, thread_id, 0] = u_std_d[0]*xoroshiro128p_normal_float32(rng_states, abs_thread_id)
    noise_samples_d[block_id, thread_id, 1] = u_std_d[1]*xoroshiro128p_normal_float32(rng_states, abs_thread_id)


'''
Test the MPPI planner

'''



def main():
    try:
            
      NUMBER_OF_TRAJECTORIES = 1

      cfg = Config(T = 4,
                  dt = 0.05,
                  num_control_rollouts =2000, # Same as number of blocks, can be more than 1024
                  num_vis_state_rollouts = 200,
                  seed = 1)

      # Read environment from pickle file
      import pickle
      #Read the environment from the pickle file
      # with open(r'C:\Users\ogpoy\Documents\GitHub\diffusionPolicy\src\traj_generation\envs\environments_1.pickle', 'rb') as f:
      # with open('/home/jiang/Documents/GitHub/diffusionPolicy/src/traj_generation/envs/environments_4_with_costmaps.pickle', 'rb') as f:
      with open('/mnt/c/Users/ogpoy/Documents/GitHub/diffusionPolicy/src/traj_generation/envs/environments_2_with_costmaps.pickle', 'rb') as f:
          env = pickle.load(f)


      # Initial State should be (0,0,random theta between 0 and pi/2)
      all_trajectories = []

      for i in range(len(env)):
        if i < 2:
          continue
        env_name = f"environment_{i + 1}"
        # print(env_name)
        obstacles = env[env_name]['circles']
        goals = env[env_name]['goal_positions']
        # print(goals)
        occupancy_grid = env[env_name]['occupancy_grid']
        costmap_env = env[env_name]['costmap']

        # from obstacles, get obstacle_positions and obstacle_radius,(x,y,radius)
        obstacle_positions = np.array([obstacle[:2] for obstacle in obstacles], dtype=np.float32)
        obstacle_radius = np.array([obstacle[2] for obstacle in obstacles], dtype=np.float32)

        env_trajectories = [] # Store all trajectories for this environment for every goal position
        # pdb.set_trace()
        for xgoal_idx, xgoal in enumerate(goals):
          if xgoal_idx < 1:
            continue
          # xgoal = np.array([5.2,5.9])
          theta = np.pi/4
          x0 = np.array([0.0, 0.0, 0.0, 0.0, theta]) # Kinematic model states
          
          mppi_params = dict(
              # Task specification
              dt = cfg.dt, 
              x0 = x0, # Start state
              xgoal = xgoal, # Goal position
              # vehicle length(lf and lr wrt the cog) and width
              vehicle_length = 0.57,
              vehicle_width = 0.3,
              vehicle_wheelbase = 0.32,
              # For risk-aware min time planning
              goal_tolerance = 0.2,
              dist_weight = 10, #  Weight for dist-to-goal cost.

              lambda_weight = 0.5, # Temperature param in MPPI
              num_opt = 1, # Number of steps in each solve() function call.

              # Control and sample specification
              u_std = np.array([1.0, 1.0]), # Noise std for sampling linear and angular velocities.
              vrange = np.array([0.0, 1.5]), # Linear velocity range.
              wrange = np.array([-np.pi, np.pi]), # Angular velocity range.
              v_switch = 0.2, 
              a_max = 2.0 , # Maximum linear acceleration
              delta_range = np.array([-np.pi/3, np.pi/3]), # Steering angle range
              ## obstacles
              costmap = costmap_env,
              obs_penalty = 1e8,
          )
          # MPPI planner setup
          mppi_planner = MPPI_Numba(cfg)
          mppi_planner.setup(mppi_params)
          rollout_states_vis_list = []
          
          goal_trajectories = [] # Store all trajectories for this goal position
          # Loop
          for traj_i in range(NUMBER_OF_TRAJECTORIES):
              print(f"Environment: {env_name}, Goal Position: {xgoal}, Trajectory: {traj_i}")
              max_steps = int(1e3)
              xhist = np.zeros((max_steps+1, 5))*np.nan # Kinematic Model States 
              uhist = np.zeros((max_steps, 2))*np.nan
              boundary_points_arr = np.zeros((max_steps,5,2))*np.nan
              xhist[0] = x0
              # print(mppi_planner.get_vehicle_boundary_points_p(np.array([0.0,0.0,np.deg2rad(0)]), 1.0, 1.0))
              boundary_points_arr[0] = mppi_planner.get_vehicle_boundary_points_p(x0, mppi_params['vehicle_length'], mppi_params['vehicle_width'])
              
              # 700, 1500 --> 0.05 --> 35,75 --> 
              vis_xlim = [-2, 7]
              vis_ylim = [-2, 3]
              plot_every_n = 50
              # Create an empty list to store the rollout states
              rollout_states = []
              for t in range(max_steps):
                  # Solve
                  st = time.time()
                  useq = mppi_planner.solve()
                  u_curr = useq[0]
                  uhist[t] = u_curr
                  print("Solve time", time.time()-st)
                  # Simulate state forward using the sampled map

                  #Kineamtic model update
                  xhist[t+1, 0] = xhist[t, 0] + cfg.dt*xhist[t, 3]*np.cos(xhist[t, 4])
                  xhist[t+1, 1] = xhist[t, 1] + cfg.dt*xhist[t, 3]*np.sin(xhist[t, 4])
                  xhist[t+1, 2] = xhist[t, 2] + cfg.dt*steering_constraints(xhist[t, 2],u_curr[1], mppi_params['delta_range'][0], mppi_params['delta_range'][1], mppi_params['wrange'][0], mppi_params['wrange'][1])
                  xhist[t+1, 3] = xhist[t, 3] + cfg.dt*acceleration_constraints(xhist[t, 3], u_curr[0], mppi_params['vrange'][0], mppi_params['vrange'][1], mppi_params['a_max'], mppi_params['v_switch'])
                  xhist[t+1, 4] = xhist[t, 4] + cfg.dt*(xhist[t, 3]/mppi_params['vehicle_wheelbase'])*np.tan(xhist[t, 2])
                  xhist[t+1, 4] = math.fmod(xhist[t+1, 4], 2*np.pi)

                  # Get vehicle boundary points for visualization
                  boundary_points_arr[t+1] = mppi_planner.get_vehicle_boundary_points_p(xhist[t+1], mppi_params['vehicle_length'], mppi_params['vehicle_width'])
                  
                  if (t%plot_every_n==0): # and (i >= 1):
                  # if t==max_steps-1:
                      # Visualize the basic set up
                      fig, ax = plt.subplots()
                      ax.plot([x0[0]], [x0[1]], 'ro', markersize=10, markerfacecolor='none', label="Start")
                      ax.plot([xhist[t, 0]], [xhist[t, 1]], 'ro', markersize=10, label="Curr. State", zorder=5)
                      # Curr vehicle boundary points for visualization as rectangle
                      print(xhist[t],"Current state")
                      ax.plot(boundary_points_arr[t,:,0], boundary_points_arr[t,:,1], 'r', label="Vehicle", zorder=5)
                      # ax.plot(curr_boundary_points[:,0], curr_boundary_points[:,1], 'r', label="Vehicle", zorder=5)
                      c1 = plt.Circle(xgoal, mppi_params['goal_tolerance'], color='b', linewidth=2, fill=False, label="Goal", zorder=7)
                      ax.add_patch(c1)

                      # Show obstacles
                      for obs_pos, obs_r in zip(obstacle_positions, obstacle_radius):
                          obs = plt.Circle(obs_pos, obs_r, color='b', fill=True, zorder=6)
                          ax.add_patch(obs)


                      # Get rollout states from subset of maps for visualization? (e.g., 50)
                      rollout_states_vis = mppi_planner.get_state_rollout()
                      # Save rollout_states_vis for each time step for futher analysis
                      rollout_states_vis_list.append(rollout_states_vis)
                      

                      # print(rollout_states_vis.shape)

                      
                      ax.plot(xhist[:,0], xhist[:,1], 'r', label="Past State")
                      
                      # ax.plot(rollout_states_vis[:,-1,0].T, rollout_states_vis[:,-1,1].T, 'r.', zorder=4)
                      ax.plot(rollout_states_vis[:,:,0].T, rollout_states_vis[:,:,1].T, 'k', alpha=0.5, zorder=3)
                      ax.plot(rollout_states_vis[0,:,0], rollout_states_vis[0,:,1], 'g', alpha=1, label="Rollouts", zorder=6)
                      ax.set_xlim(vis_xlim)
                      ax.set_ylim(vis_ylim)

                      ax.legend(loc="upper left")
                      ax.set_aspect("equal")
                      plt.tight_layout()

                      # Plot the current costmap
                      fig1, ax1 = plt.subplots()
                      ax1.imshow((1-mppi_params['costmap']), cmap='gray', interpolation='none', origin='lower', zorder=0)
                      x0_x_map, x0_y_map = convert_position_to_costmap_indices_cpu(x0[:2], map_resolution=0.05, origin=[-15, -10])
                      xhist_x_map, xhist_y_map = convert_position_to_costmap_indices_cpu(np.array([xhist[t,0], xhist[t,1]]), map_resolution=0.05, origin=[-15, -10])
                      ax1.plot([x0_y_map], [x0_x_map], 'ro', markersize=10, markerfacecolor='none', label="Start")
                      ax1.plot([xhist_y_map], [xhist_x_map], 'ro', markersize=10, label="Curr. State", zorder=5)
                      # convert xgoal to costmap indices
                      xgoal_x_map, xgoal_y_map = convert_position_to_costmap_indices_cpu(xgoal, map_resolution=0.05, origin=[-15, -10])
                      ax1.plot([xgoal_y_map], [xgoal_x_map], 'bo', markersize=10, markerfacecolor='none', label="Goal")
                      # Curr vehicle boundary points for visualization as rectangle
                      # ax1.plot(boundary_points_arr[t,:,0], boundary_points_arr[t,:,1], 'r', label="Vehicle", zorder=5)

                      
                      plt.show()
              
                  # Update MPPI state (x0, useq)
                  mppi_planner.shift_and_update(xhist[t+1], useq, num_shifts=1)

              # Goal check
                  if np.linalg.norm(xhist[t+1, :2] - mppi_params['xgoal']) <= mppi_params['goal_tolerance']:
                      print("goal reached at t={:.2f}s".format(t*cfg.dt))
                      break
                  

              goal_trajectories.append(xhist)
              theta = np.random.uniform(0, np.pi / 2)
              x0[4] = theta
              mppi_params['x0'] = x0


          env_trajectories.append(goal_trajectories)
          print(f"Environment: {env_name}, Goal Position: {xgoal}, Trajectories are completed")
          # print which environment and goal position is being processed
          

          goal_trajectories = np.array(goal_trajectories)
          print(goal_trajectories.shape)
          # fig, ax = plt.subplots()
          # for i in range(goal_trajectories.shape[0]):
          #     ax.plot(goal_trajectories[i,:,0], goal_trajectories[i,:,1], label=f'xhist_{i}')
          # ax.legend()
          # ax.axis('equal')
          # plt.show()
        all_trajectories.append(env_trajectories)
        print(f"Environment: {env_name} is completed")
      all_trajectories = np.array(all_trajectories)
      print(all_trajectories.shape)

      isExist = True
      saveFlag = True
      idx = 0
      while isExist:
          if os.path.exists(f'trajectories_{idx}.pickle'):
              print(f"Warning: 'trajectories_{idx}.pickle' already exists in the current directory.")
              idx += 1
          elif saveFlag:
              with open(f'trajectories_{idx}.pickle', 'wb') as file:
                  pickle.dump(all_trajectories, file)
              print(f"\'trajectories_{idx}.pickle' is created in the current directory.")
              isExist = False
          else:
              print("Do you want to save the environments? Change saveFlag to True to save the environments.")
              isExist = False

    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt. Exiting...")

    finally:
        # Ensure the context is popped to avoid any CUDA-related issues
        print("Popping CUDA context...")
        # Clean up the context for both numba and cuda
        # numba_cuda.contextmanager.clean()
        primary_context.pop()
        primary_context.detach()

if __name__ == "__main__":
    main()
    sys.exit(0)
