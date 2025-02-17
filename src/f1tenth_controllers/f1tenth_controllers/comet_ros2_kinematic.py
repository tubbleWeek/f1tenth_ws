#!/usr/bin/env python3
# ROS 2 packages
import rclpy
from tf2_ros import Buffer, TransformListener # for locolization
import traceback
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped  # Use TransformStamped instead of Rigids
from collections import deque  # For implementing a circular buffer
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid # for applying local costmap
import std_msgs

import numpy as np
import math
import copy
import numba
import time
import sys 
import pickle

from scipy.ndimage import gaussian_filter, distance_transform_cdt, distance_transform_edt
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
# primary_context.pop()

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
np.set_printoptions(precision=2, suppress=True)

num_traj = 1000 # only use the first 'num_traj' trajectories
print("Reading cuniform trajectories...")
# with open('/home/nvidia/f1tenth_ws/src/f1tenth_controllers/resource/FINAL_C_Uniform_100000_trajectories_disjoint_DUBINS_v_1_perturb_2.01_slack_2.01_seed_2025_grid_0.05_0.05_4.50deg_na45_t4.01_ts0.2.pkl', 'rb') as f:
with open("/home/nvidia/f1tenth_ws/src/f1tenth_controllers/resource/Final_Rahul's_Unsupervised_C_Uniform_50000.pickle", 'rb') as f:
    cuniform_trajectories = pickle.load(f)[:num_traj]

# Function to handle None in the scalar part
def process_element(array_part, scalar_part):
    # Replace None scalar with 0.0
    if scalar_part is None:
        scalar_part = 0.0
    return np.concatenate([array_part, [scalar_part]])

# Process all trajectories
print("Processing cuniform trajectories...")
processed_trajectories = [
    np.array([process_element(array_part, scalar_part) for array_part, scalar_part in trajectory])
    for trajectory in cuniform_trajectories
]

# Combine into a 3D array if all trajectories are of the same size
cuniform_trajectories_transformed = np.array(processed_trajectories)

import os
class Config:
  """ Configurations that are typically fixed throughout execution. """
  def __init__(self, 
               T=3, # Horizon (s)
               dt=0.2, # Length of each step (s)
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
DEFAULT_DIST_WEIGHT = 1e1
class CUniform_Numba(object):
  """ 
  Planner object that initializes GPU memory and runs MPPI on GPU via numba.   
  CURRENT IMPLEMENATION will one calculate the cost of the each trajectory and select the one with the minimum one. 
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
    self.x0 = None
    # Basic info 
    self.max_threads_per_block = cfg.max_threads_per_block

    # Initialize reuseable device variables
    self.u_cur_d = None
    self.u_prev_d = None
    self.costs_d = None
    self.feasible_mask_d = None           # This variable and one below is new
    self.num_feasible_d = np.float32(0.0)
    self.weights_d = None
    self.rng_states_d = None
    self.state_rollout_batch_d = None # For visualization only. Otherwise, inefficient

    # Other task specific params
    self.device_var_initialized = False
    self.generator = XORWOWRandomNumberGenerator()

    # Task specific params
    # local costmap size and resolution
    self.local_costmap_size = 120
    self.local_costmap_resolution = 0.05
    self.reset()
    
  def reset(self):
    # Other task specific params
    self.u_seq0 = np.zeros((self.num_steps, 2), dtype=np.float32)
    self.u_seq0[:,0] = 1.0 # Linear velocity
    self.params = None
    self.params_set = False
    self.costmap_loaded = False
    self.u_prev_d = None
    
    # Initialize all fixed-size device variables ahead of time. (Do not change in the lifetime of MPPI object)
    self.init_device_vars_before_solving()

  def load_trajectories(self, trajectories):
    self.trajectories = copy.deepcopy(trajectories)
    self.trajectories_d = numba_cuda.to_device(self.trajectories.astype(np.float32))
    self.original_trajectories_d = numba_cuda.to_device(self.trajectories.astype(np.float32))

  def init_device_vars_before_solving(self):
    if not self.device_var_initialized:
      t0 = time.time()
      # Useq
      self.u_cur_d = numba_cuda.to_device(self.u_seq0) 
      self.u_prev_d = numba_cuda.to_device(self.u_seq0)

      self.costs_d = numba_cuda.device_array((self.num_control_rollouts), dtype=np.float32)
      self.feasible_mask_d = numba_cuda.device_array((self.num_control_rollouts, self.num_steps+1), dtype=np.float32)
      # add full ones to the feasible mask
      self.feasible_mask_d[0:self.num_control_rollouts, 0:self.num_steps+1] = 1.0
      self.weights_d = numba_cuda.device_array((self.num_control_rollouts), dtype=np.float32)
      self.local_costmap_d = numba_cuda.device_array((self.local_costmap_size, self.local_costmap_size), dtype=np.float32)  
       
      self.debug_d = numba_cuda.device_array((self.num_control_rollouts, self.num_steps+1, 4), dtype=np.float32)
      self.device_var_initialized = True
      print(" CUniform planner has initialized GPU memory after {} s".format(time.time()-t0))

  def setup(self, params):
    # These tend to change (e.g., current robot position, the map) after each step
    self.set_params(params)

  def set_params(self, params):
    self.params = copy.deepcopy(params)
    self.x0 = self.params['x0']
    if self.params['costmap'] is not None: # should be type ndarray
        self.costmap_loaded = True
    self.params_set = True

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
      print("C-Uniform solve condition not met. Cannot solve. Return")
      return
    return self.get_rollout_cost()
  
  def move_cuniform_task_vars_to_device(self):
    xgoal_d = numba_cuda.to_device(self.params['xgoal'].astype(np.float32))
    x0_d = numba_cuda.to_device(self.params['x0'].astype(np.float32))
    goal_tolerance_d = np.float32(self.params['goal_tolerance'])

    vehicle_length_d = np.float32(self.vehicle_length)
    vehicle_width_d = np.float32(self.vehicle_width)
    vehicle_wheelbase_d = np.float32(self.vehicle_wheelbase)

    #USEQ UPDATE
    lambda_weight_d = np.float32(self.params['lambda_weight'])
    vrange_d = np.array(self.params['vrange'], dtype=np.float32)
    wrange_d = np.array(self.params['wrange'], dtype=np.float32)

    ''' COSTMAP Variables'''
    local_costmap_edt = self.params['costmap']
    local_costmap_edt = np.ascontiguousarray(local_costmap_edt)
    max_local_cost_d = np.float32(np.max(local_costmap_edt))
    # set the local costmap to the local_costmap_d on the device
    local_costmap_d = numba_cuda.to_device(local_costmap_edt)
    costmap_origin_x = np.float32(self.params['costmap_origin'][0])
    costmap_origin_y = np.float32(self.params['costmap_origin'][1])
    costmap_resolution = np.float32(self.params['costmap_resolution'])

    #obstacle
    obs_cost_d = np.float32(DEFAULT_OBS_COST if 'obs_penalty' not in self.params 
                                  else self.params['obs_penalty'])

    return xgoal_d, x0_d, goal_tolerance_d, \
            vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d, \
            lambda_weight_d, vrange_d, wrange_d, \
            obs_cost_d, local_costmap_d, max_local_cost_d, \
            costmap_origin_x, costmap_origin_y, costmap_resolution

  def get_rollout_cost(self):
    '''
    Calculate the cost of the each trajectories and find the trajectory with min cost. and return that trajectory to the host
    '''
    xgoal_d, x0_d, goal_tolerance_d, \
    vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d, \
    lambda_weight_d, vrange_d, wrange_d, \
    obs_cost_d, local_costmap_d, max_local_cost_d, \
    costmap_origin_x, costmap_origin_y, costmap_resolution, \
    = self.move_cuniform_task_vars_to_device()

    # Weight for distance cost
    dist_weight = DEFAULT_DIST_WEIGHT if 'dist_weight' not in self.params else self.params['dist_weight']

    # Transform the trajectories to the current state #TODO: What does this part do?
    # threadperblock = (16,16)
    # blockpergrid_x = (self.num_control_rollouts + threadperblock[0] - 1) // threadperblock[0]
    # blockpergrid_y = (self.num_steps + threadperblock[1] - 1) // threadperblock[1]
    # blockpergrid = (blockpergrid_x, blockpergrid_y)
    # self.transform_trajectories[blockpergrid, threadperblock](x0_d, self.original_trajectories_d, self.trajectories_d)

    # get the number of feasible trajectories
    self.num_feasible_d = np.float32(np.sum(self.feasible_mask_d.copy_to_host()))

    self.rollouts_cost_numba[self.num_control_rollouts, 1](
      self.trajectories_d,
      self.costs_d,
      goal_tolerance_d,
      xgoal_d,
      x0_d,
      obs_cost_d,
      local_costmap_d,
      costmap_origin_x,
      costmap_origin_y,
      costmap_resolution,
      max_local_cost_d,
      vehicle_length_d,
      vehicle_width_d,
      vehicle_wheelbase_d,
      dist_weight,
      
      self.debug_d
    )

    # # get the cost of the trajectories that are feasible to the host
    cost_arr = self.costs_d.copy_to_host()
    min_cost_index = np.argmin(cost_arr)

    self.u_prev_d = self.u_cur_d
    return min_cost_index, self.costs_d.copy_to_host(), self.trajectories_d[min_cost_index].copy_to_host(), self.u_cur_d.copy_to_host()

  def get_state_rollout(self, x_curr, trajectories):
    # First translate the point
    transformed_trajectories = copy.deepcopy(trajectories) #RUNTIME: negeligible

    # Then rotate the point, Rotation matrix
    theta = x_curr[2]
    translation = x_curr[:2]
    R = np.array([[math.cos(x_curr[2]), -math.sin(x_curr[2])], [math.sin(x_curr[2]), math.cos(x_curr[2])]])
    transformed_trajectories[:, :, :2] = np.einsum('ij,nmj->nmi', R, transformed_trajectories[:, :, :2])

    transformed_trajectories[:, :, 2] += theta
    transformed_trajectories[:, :, :2] += translation
    return transformed_trajectories
    
  def shift_and_update(self, x_next, useq, trajectories):
    trajs = copy.deepcopy(trajectories)
    transformed_trajs = self.get_state_rollout(x_next, trajs)
    self.load_trajectories(transformed_trajs)

    self.x0 = self.params["x0"] = x_next.copy()

    u_cur_shifted = useq.copy()
    u_cur_shifted[:-1] = u_cur_shifted[1:]
    self.u_cur_d = numba_cuda.to_device(u_cur_shifted.astype(np.float32))

  """GPU kernels from here on"""
  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def rollouts_cost_numba(
    trajectories_d,
    costs_d,
    goal_tolerance_d,
    xgoal_d,
    x0_d,
    obs_cost_d,
    local_costmap_d,
    costmap_origin_x,
    costmap_origin_y,
    params_costmap_resolution,
    max_local_cost_d,
    vehicle_length_d,
    vehicle_width_d,
    vehicle_wheelbase_d,
    dist_weight_d,
    debug_d
  ):
    """
    There should only be one thread running in each block, where each block handles a single sampled trajecotry calulation.
    """
    # Get block id and thread id
    bid = numba_cuda.blockIdx.x   # index of block
    tid = numba_cuda.threadIdx.x  # index of thread within a block

    # Initialize the cost for the trajectory
    costs_d[bid] = 0.0
    goal_reached = False
    isCollided = False
    goal_tolerance_d2 = goal_tolerance_d*goal_tolerance_d
    dist_to_goal2 = prev_dist_to_goal2 = 1e9 # initialize to a large value

    # Allocate space for vehicle boundary points (4)
    x_curr = numba_cuda.local.array(3, numba.float32) # Dubins car model states x,y,theta
    x_curr_grid_d = numba_cuda.local.array((2), dtype=np.int32)
    
    gamma = 1.0 # Discount factor for cost

    # Loop through each state in the trajectory
    num_steps = trajectories_d.shape[1]
    for step in range(num_steps):
        # Extract current state (x, y, theta)
        for i in range(3):
          x_curr[i] = trajectories_d[bid, step, i]

        if not isCollided:
          convert_position_to_costmap_indices_gpu(
              x_curr[0],
              x_curr[1],
              costmap_origin_x,
              costmap_origin_y,
              params_costmap_resolution,
              x_curr_grid_d,
          )

          # Check for collision
          if check_state_collision_gpu(local_costmap_d, x_curr_grid_d) == 1.0:
            isCollided = True
            # costs_d[bid] += 1e6 * (action)**2
          costs_d[bid] += calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d) / max_local_cost_d * obs_cost_d

          # Compute distance to goal
          dist_to_goal2 = (((xgoal_d[0]-x_curr[0])**2 + (xgoal_d[1]-x_curr[1])**2))**0.5
          costs_d[bid] += stage_cost(dist_to_goal2, 1.0)
          
          if dist_to_goal2  <= goal_tolerance_d: #TODO: why not use goal_tolerance_d2 here?
            goal_reached = True
            break
          prev_dist_to_goal2 = dist_to_goal2
        else:
          costs_d[bid] += 1 * obs_cost_d
          costs_d[bid] += prev_dist_to_goal2 # distans

        debug_d[bid, step, 0] = x_curr_grid_d[0]
        debug_d[bid, step, 1] = x_curr_grid_d[1]
        debug_d[bid, step, 2] = calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d)
        costs_d[bid] +=  calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d) * obs_cost_d * gamma
        debug_d[bid, step, 3] = calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d) * obs_cost_d * gamma
        gamma *= 1.0

    # Accumulate terminal cost 
    costs_d[bid] += term_cost(dist_to_goal2, goal_reached)
    # give reward for reaching the goal
    # costs_d[bid] += (-goal_reached) * 10

  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def transform_trajectories(
    x_curr,
    trajectories,
    transformed_trajectories
  ):
    """ Transform the trajectories to the current state. """
    i, j = numba_cuda.grid(2)
    if i < trajectories.shape[0] and j < trajectories.shape[1]:
      # precompute sin and cos
      cos_theta = math.cos(x_curr[2])
      sin_theta = math.sin(x_curr[2])
      x_curr_x = x_curr[0]
      x_curr_y = x_curr[1]
      x_curr_theta = x_curr[2]

      # load the trajectory
      x = trajectories[i, j, 0]
      y = trajectories[i, j, 1]
      theta = trajectories[i, j, 2]

      # Rotate the trajectory
      transformed_trajectories[i, j, 0] = x * cos_theta - y * sin_theta + x_curr_x
      transformed_trajectories[i, j, 1] = x * sin_theta + y * cos_theta + x_curr_y
      transformed_trajectories[i, j, 2] = theta + x_curr_theta

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
      self.mu_LogN, self.std_LogN = Normal2LogN(0, np.mean([0.1, 0.2]))
      self.rLogN_info = [self.mppi_type, self.mu_LogN, self.std_LogN]

    # local costmap size and resolution
    self.local_costmap_size = 120
    self.local_costmap_resolution = 0.05
    self.reset()
    
  def reset(self):
    # Other task specific params
    self.u_seq0 = np.zeros((self.num_steps, 2), dtype=np.float32)
    self.u_seq0[:,0] = 1.0 # Linear speed
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
      
      self.debug_d = numba_cuda.device_array((self.num_control_rollouts, self.num_steps+1, 3), dtype=np.float32)

      self.state_rollout_batch_d = numba_cuda.device_array((self.num_vis_state_rollouts, self.num_steps+1, 3), dtype=np.float32)   
      self.local_costmap_d = numba_cuda.device_array((self.local_costmap_size, self.local_costmap_size), dtype=np.float32)   
      self.device_var_initialized = True
      print("MPPI planner has initialized GPU memory after {} s".format(time.time()-t0))
    else:
      print('first actions are reinitialized')
      self.u_cur_d = numba_cuda.to_device(self.u_seq0)
      self.u_prev_d = numba_cuda.to_device(self.u_seq0)

  def setup(self, params):
    # These tend to change (e.g., current robot position, the map) after each step
    self.set_params(params)
  
  def set_params(self, params):
    self.params = copy.deepcopy(params)
    self.params_set = True
    if self.params['costmap'] is not None: # should be type ndarray
      self.costmap_loaded = True

  def set_actions(self, u_seq):
    u_seq_dummy = copy.deepcopy(u_seq)
    self.u_cur_d = numba_cuda.to_device(u_seq_dummy.astype(np.float32))

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

  def random_noise_sample(self):
    # Use the random generator to generate random noise
    # The logic from log-MPPI_ros github repo
    if self.mppi_type == 0: # Normal Dist
      du_d = self.generator.gen_normal(
          self.num_control_rollouts * self.num_steps * 2,
          np.float32)

    # log-MPPI
    elif self.mppi_type == 1: # NLN
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

    local_costmap_edt = self.params['costmap']
    local_costmap_edt = np.ascontiguousarray(local_costmap_edt)
    max_local_cost_d = np.float32(np.max(local_costmap_edt))
    # set the local costmap to the local_costmap_d on the device
    local_costmap_d = numba_cuda.to_device(local_costmap_edt)
    obs_cost_d = np.float32(DEFAULT_OBS_COST if 'obs_penalty' not in self.params 
                                  else self.params['obs_penalty'])
    costmap_origin_x = np.float32(self.params['costmap_origin'][0])
    costmap_origin_y = np.float32(self.params['costmap_origin'][1])
    costmap_resolution = np.float32(self.params['costmap_resolution'])
    return vrange_d, wrange_d, xgoal_d, \
           goal_tolerance_d, lambda_weight_d, \
           vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d, \
           v_switch_d, a_max_d, delta_range_d, \
           u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d, \
           costmap_origin_x, costmap_origin_y, costmap_resolution

  def solve_with_nominal_dynamics(self):
    """
    Launch GPU kernels that use nominal dynamics but adjsuts cost function based on worst-case linear speed.
    """
    vrange_d, wrange_d, xgoal_d, goal_tolerance_d, lambda_weight_d, \
    vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d,\
    v_switch_d, a_max_d, delta_range_d,\
    u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d, \
    costmap_origin_x, costmap_origin_y, params_costmap_resolution = self.move_mppi_task_vars_to_device()

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

      # Rollout and compute mean or cvar
      self.rollout_new_cost_numba[self.num_control_rollouts, 1](
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
        costmap_origin_x,
        costmap_origin_y,
        params_costmap_resolution,

        # results
        self.costs_d,
        self.debug_d
      )      
      # print(f"costs_d: {self.costs_d.copy_to_host()}")
      # print(f'max cost: {np.max(self.costs_d.copy_to_host())}')
      # print(f'min cost: {np.min(self.costs_d.copy_to_host())}')
      
      # # print a debug trajectory
      # debug_arr = self.debug_d.copy_to_host()
      # print(f"debug_arr: {debug_arr[0,:,:]}")
      # print(f"debug_arr: {debug_arr[1,:,:]}")
      # print(f"debug_arr: {debug_arr[2,:,:]}")
      # print(f"debug_arr: {debug_arr[3,:,:]}")
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
    self.params["x0"] = new_x0.copy() #NOTE: mppi_dubins_moving_target_ros_node.py have commentted this line out
    self.shift_optimal_control_sequence(u_cur, num_shifts)

  def shift_optimal_control_sequence(self, u_cur, num_shifts=1):
    u_cur_shifted = u_cur.copy()
    u_cur_shifted[:-num_shifts] = u_cur_shifted[num_shifts:]
    self.u_cur_d = numba_cuda.to_device(u_cur_shifted.astype(np.float32))

  def get_state_rollout(self):
    """ Generate state sequences based on the current optimal control sequence. """
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

  """GPU kernels from here on"""
  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def rollout_new_cost_numba(
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
          costmap_origin_x,
          costmap_origin_y,
          params_costmap_resolution,
          costs_d,
          debug_d):
    """
    There should only be one thread running in each block, where each block handles a single sampled control sequence.
    """
    # Get block id and thread id
    bid = numba_cuda.blockIdx.x   # index of block
    tid = numba_cuda.threadIdx.x  # index of thread within a block
    costs_d[bid] = 0.0

    # Explicit unicycle update and map lookup
    # From here on we assume grid is properly padded so map lookup remains valid
    x_curr = numba_cuda.local.array(3, numba.float32) # x, y, theta
    x_curr_grid_d = numba_cuda.local.array((2), dtype=np.int32)

    for i in range(3): 
      x_curr[i] = x0_d[i]

    timesteps = len(u_cur_d)
    goal_reached = False
    isCollided = False

    # squared goal tolerance
    goal_tolerance_d2 = goal_tolerance_d*goal_tolerance_d
    dist_to_goal2 = prev_dist_to_goal2 = 1e9

    v_nom = v_noisy = w_nom = w_noisy = 0.0

    gamma = 1.0
    for t in range(timesteps):
      # Nominal noisy control
      w_nom = u_cur_d[t, 1] + noise_samples_d[bid, t, 1]
      w_noisy = max(wrange_d[0], min(wrange_d[1], w_nom))
      v_noisy = vrange_d[0] # fixed speed 1.0

      # Forward simulate
      # Dubins model update
      x_curr[0] += dt_d*v_noisy*math.cos(x_curr[2])
      x_curr[1] += dt_d*v_noisy*math.sin(x_curr[2])
      # x_curr[2] += dt_d*(x_curr[3]/vehicle_wheelbase_d)*math.tan(w_noisy)
      x_curr[2] += dt_d*v_noisy*math.tan(w_noisy)/vehicle_wheelbase_d
      # x_curr[2] = math.fmod(x_curr[2], 2*math.pi)

      # Check the state is collided with the obstacle
      # Get current state costmap indices
      if not isCollided:
        convert_position_to_costmap_indices_gpu(
          x_curr[0],
          x_curr[1],
          costmap_origin_x,
          costmap_origin_y,
          params_costmap_resolution,
          x_curr_grid_d,
        )
        if check_state_collision_gpu(local_costmap_d, x_curr_grid_d) == 1.0:
          isCollided = True
        costs_d[bid] += calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d) / 49 * obs_cost_d

        # distance to goal cost
        dist_to_goal2 = (((xgoal_d[0]-x_curr[0])**2) + ((xgoal_d[1]-x_curr[1])**2)) ** 0.5
        costs_d[bid] += stage_cost(dist_to_goal2, 1.0)
        if dist_to_goal2 <= goal_tolerance_d:
          goal_reached = True
          break
        prev_dist_to_goal2 = dist_to_goal2
        
      else:
        costs_d[bid] +=  1 * obs_cost_d
        costs_d[bid] += prev_dist_to_goal2 # distance to goal cost
    # Accumulate terminal cost 
    costs_d[bid] += term_cost(dist_to_goal2, goal_reached)

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
      x_curr = numba_cuda.local.array(3, numba.float32) # x, y, theta
      for i in range(3): 
        x_curr[i] = x0_d[i]
        state_rollout_batch_d[bid,0,i] = x0_d[i]
      
      for t in range(timesteps):
        # Nominal noisy control
        v_nom = vrange_d[0]
        w_nom = u_cur_d[t, 1]
        
        # Forward simulate
        # Kinematic model update
        x_curr[0] += dt_d*v_nom*math.cos(x_curr[2])
        x_curr[1] += dt_d*v_nom*math.sin(x_curr[2])
        # x_curr[2] += dt_d*(x_curr[3]/vehicle_wheelbase_d)*math.tan(w_noisy)
        x_curr[2] += dt_d*v_nom*math.tan(w_nom)/vehicle_wheelbase_d
        # x_curr[2] = math.fmod(x_curr[2], 2*math.pi)
        # Save state
        state_rollout_batch_d[bid,t+1,0] = x_curr[0]
        state_rollout_batch_d[bid,t+1,1] = x_curr[1]
        state_rollout_batch_d[bid,t+1,2] = x_curr[2]
    else:
      # Explicit unicycle update and map lookup
      # From here on we assume grid is properly padded so map lookup remains valid
      x_curr = numba_cuda.local.array(3, numba.float32)
      for i in range(3): 
        x_curr[i] = x0_d[i]
        state_rollout_batch_d[bid,0,i] = x0_d[i]
      
      for t in range(timesteps):
        # Nominal noisy control
        # v_nom = u_prev_d[t, 0] + noise_samples_d[bid, t, 0]
        # v_noisy = max(vrange_d[0], min(vrange_d[1], v_nom))
        w_nom = u_cur_d[t, 1] + noise_samples_d[bid, t, 1]
        w_noisy = max(wrange_d[0], min(wrange_d[1], w_nom))

        # # Nominal noisy control
        v_nom = v_noisy = vrange_d[0]
        w_nom = u_cur_d[t, 1]
        
        # Kinematic model update
        x_curr[0] += dt_d*v_noisy*math.cos(x_curr[2])
        x_curr[1] += dt_d*v_noisy*math.sin(x_curr[2])
        # x_curr[2] += dt_d*(x_curr[3]/vehicle_wheelbase_d)*math.tan(w_noisy)
        x_curr[2] += dt_d*v_noisy*math.tan(w_noisy)/vehicle_wheelbase_d
        # x_curr[2] = math.fmod(x_curr[2], 2*math.pi)
        # Save state
        state_rollout_batch_d[bid,t+1,0] = x_curr[0]
        state_rollout_batch_d[bid,t+1,1] = x_curr[1]
        state_rollout_batch_d[bid,t+1,2] = x_curr[2]

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
  
class COMETPlannerNode(Node):
    def __init__(self):
        super().__init__('COMET_planner_node')
        self.cfg = Config(T = 3,
            dt = 0.2,
            num_control_rollouts =1000, # Same as number of blocks, can be more than 1024
            num_vis_state_rollouts = 1000,
            seed = 1,
            )
        
        # Define initial parameters for the c_uniform planner.
        self.cuniform_params = dict(
          # Task specification
          dt = self.cfg.dt, 
          x0 = np.zeros(3), # Start state
          xgoal = np.array([-1.0, -15.0]), # Goal position
          # vehicle length(lf and lr wrt the cog) and width
          vehicle_length = 0.57,
          vehicle_width = 0.3,
          vehicle_wheelbase= 0.32,
          # For risk-aware min time planning
          goal_tolerance = 0.40,
          dist_weight = 1e2, #  Weight for dist-to-goal cost.
          num_opt = 1, # Number of steps in each solve() function call.

          # Control and sample specification
          # variance = 0.1
          u_std = np.array([0.023, 0.05]), # Noise std for sampling linear and angular velocities.
          vrange = np.array([1.0, 1.0]), # Linear velocity range. Constant Linear Velocity
          wrange = np.array([-np.pi/4, np.pi/4]), # Angular velocity range.
          
          costmap = None, # intiallly nothing
          obs_penalty = 1e4
        )
        
        # Instantiate the c_uniform planner and load trajectories
        self.cuniform = CUniform_Numba(self.cfg)
        self.original_trajectories = cuniform_trajectories_transformed
        self.cuniform.setup(self.cuniform_params)
        self.cuniform.load_trajectories(self.original_trajectories[:])
        self.get_logger().info('Cuniform Planner initialized.')

        # Define initial parameters for the MPPI planner
        self.mppi_params = dict(
          # Task specification
          dt = self.cfg.dt, 
          x0 = np.zeros(3), # Start state
          # vehicle length(lf and lr wrt the cog) and width
          vehicle_length = 0.57,
          vehicle_width = 0.3,
          vehicle_wheelbase= 0.32,
          # For risk-aware min time planning
          goal_tolerance = 0.40,
          dist_weight = 10, #  Weight for dist-to-goal cost.

          lambda_weight = 1.0, # Temperature param in MPPI
          num_opt = 1, # Number of steps in each solve() function call.

          # Control and sample specification
          # variance = 0.1
          u_std = np.array([0.023, 0.1]), # Noise std for sampling linear and angular velocities.
          vrange = np.array([1.0, 1.0]), # Linear velocity range. Constant Linear Velocity
          wrange = np.array([-np.pi/4, np.pi/4]), # Angular velocity range.
          costmap = None, # intiallly nothing
          obs_penalty = 1e4
        )
        # Instantiate the MPPI planner
        self.mppi = MPPI_Numba(self.cfg)
        self.mppi.setup(self.mppi_params)

        # Publishers, subscribers, and tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        '''############### for costmap ##############'''
        # high level steps
        # Step 1: subscribe to OccupancyGrid and convert msg.data into a numpy array
        # Step 2: Pass that array to cuniform_params['costmap'] in solve_cuniform()
        self.local_costmap = None  # store the latest costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, # Type: nav_msgs/msg/OccupancyGrid
            '/local_costmap/costmap',
            self.costmap_callback,
            1 # only the most recent message is kept in the queue
        )
        self.debug_local_costmap_pub = self.create_publisher(OccupancyGrid, '/debug_local_costmap', 1)
        self.no_costmap_received_timer = self.create_timer(2.0, self.notify_no_costmap)
        self.action_pub = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic="/drive",
            qos_profile=qos_profile_sensor_data,
        )

        # Create a timer to call the COMET planner solve routine.
        self.timer = self.create_timer(0.1, self.solve_COMET)
        self.i = 0
        self.isGoalReached = False

    def setup(self, params):
        """
        Set up or update planning parameters.
        This method passes the parameters (such as costmap, current pose, etc.)
        to both internal planners.
        """
        self.params = copy.deepcopy(params)
        self.cuniform.setup(self.params)
        self.mppi.setup(self.params)

    def notify_no_costmap(self):
        if self.local_costmap is None:
            self.get_logger().warn("No /local_costmap/costmap data received yet...")

    def costmap_callback(self, msg: OccupancyGrid):
        # Convert msg.data to a 2D list or np.array
        width = msg.info.width
        height = msg.info.height
        # Convert msg data to float costmap and store resolution/origin
        costmap_int8 = np.array(msg.data, dtype=np.int8).reshape(height, width)
        # costmap_int8[costmap_int8 == -1] = 100 # make unknown area as obstacles 
        self.local_costmap = costmap_int8.astype(np.float32)
        self.cuniform_params['costmap_resolution'] = msg.info.resolution
        self.cuniform_params['costmap_origin'] = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.cuniform_params['costmap'] = self.local_costmap

    def solve_COMET(self):
        try:
            # 1. Look up transform from map -> base_link
            transform = self.tf_buffer.lookup_transform(
                'map',           # source frame (or "map")
                'base_link',     # target frame (your robot)
                rclpy.time.Time()
            )
            # 2. Extract x, y
            x_robot = transform.transform.translation.x
            y_robot = transform.transform.translation.y
            # 3. Convert quaternion to yaw
            quat = transform.transform.rotation
            r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            yaw_robot = r.as_euler('xyz', degrees=False)[2]            
            current_state = np.array([x_robot, y_robot, yaw_robot])

            # Update planner parameters with current state and latest costmap.
            self.cuniform_params['x0'] = current_state
            if self.local_costmap is not None:
                self.cuniform_params['costmap'] = self.local_costmap

            # Update c_uniform with the new state.
            self.cuniform.shift_and_update(current_state, self.original_trajectories)
            self.cuniform.setup(self.cuniform_params)

            ################## This is the major line change for COMET ##################
            refined_control = self.solve_COMET_core(current_state)

            '''Old solve cuniform code
            # Solve CUniform
            self.cuniform.setup(self.cuniform_params)
            min_idx, _ , _  = self.cuniform.solve()
            self.cuniform.local_costmap_origin = self.cuniform_params['costmap_origin']
            self.publish_local_costmap_debug()
            omega = self.cuniform.control(x_current, self.cuniform.trajectories[min_idx][1], dt=0.2)
            u_execute = [1.0, omega]
            '''

            # Log some status information
            h = std_msgs.msg.Header()
            h.stamp = self.get_clock().now().to_msg()
            if (self.i % 10) == 0:
                self.get_logger().info('-----------------')
                self.get_logger().info(f'Running COMET planner with configuration: x: {x_robot:.2f}, y: {y_robot:.2f}, theta: {yaw_robot:.2f}...')
                self.get_logger().info(f"Target Position: x: {self.cuniform_params['xgoal'][0]}, y: {self.cuniform_params['xgoal'][1]}")
            if self.isGoalReached:
                refined_control = [0.0, 0.0]
                self.get_logger().info("Goal Reached!!!!")
            # Create and publish drive command based on refined_control.
            drive = AckermannDrive(steering_angle=refined_control[1], speed=refined_control[0])
            data = AckermannDriveStamped(header=h, drive=drive)
            if (self.i % 10) == 0:
                self.get_logger().info(f"Input given: velocity {refined_control[0]}, Steering_Angle: {np.rad2deg(refined_control[1])}")
            self.action_pub.publish(data)
            
            # Compute distance to goal and update goal status.
            dist2goal2 = (self.cuniform_params['xgoal'][0] - x_robot)**2 + (self.cuniform_params['xgoal'][1] - y_robot)**2
            goaltol2 = self.cuniform_params['goal_tolerance'] ** 2
            if (self.i % 10) == 0:
                self.get_logger().info(f"Distance to the Goal: {dist2goal2}, Goal Tolerance: {goaltol2}")
            if dist2goal2 < goaltol2:
                self.isGoalReached = True
            else:
                self.isGoalReached = False
            self.i += 1
        except Exception as e:
            tb_str = ''.join(traceback.format_exception(None, e, e.__traceback__))
            self.get_logger().warn(f"Cannnot run solve_cuniform: {e}\n{tb_str}")

    def solve_COMET_core(self, current_state):
        """
        Core COMET planning routine:
          - Update the c_uniform planner with the current state.
          - Solve c_uniform to get the candidate trajectory.
          - Extract the candidate control sequence and use it to seed the MPPI planner.
          - Solve MPPI to refine the control sequence.
          - Update both planners and return the first control command.
        """
        # Update c_uniform planner with the current state.
        self.cuniform.shift_and_update(current_state, self.original_trajectories)
        # Solve c_uniform planner.
        min_idx, costs, candidate_traj = self.cuniform.solve()
        # Extract the control seed (assumed to be stored in column index 3).
        control_seed = candidate_traj[:, 3]
        constant_velocity = self.cuniform_params['vrange'][0]
        control_sequence = np.hstack((np.full((control_seed.shape[0], 1), constant_velocity),
                                      control_seed.reshape(-1, 1)))
        # Set actions to the MPPI planner (dropping the final action if needed).
        self.mppi.set_actions(control_sequence[:-1])
        # Solve MPPI to refine the control sequence.
        refined_control_sequence = self.mppi.solve()
        # Update both planners with the new state.
        self.cuniform.shift_and_update(current_state, self.original_trajectories)
        current_u = self.mppi.u_cur_d.copy_to_host()
        self.mppi.shift_and_update(current_state, current_u)
        return refined_control_sequence[0] # return the first control command 

    def on_shutdown(self):
        # Clean up the context for both numba and cuda
        self.get_logger().info('COMET Planner Node shutting down')
        self.get_logger().info("Popping CUDA context...")
        primary_context.pop()
        primary_context.detach()

def main(args=None):
    rclpy.init(args=args)
    node = COMETPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()
    return

if __name__ == "__main__":
    main()
    sys.exit(0)
