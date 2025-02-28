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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import std_msgs

import numpy as np
import math
import copy
import numba
import time
import sys 
import pickle 
from scipy.ndimage import gaussian_filter, distance_transform_cdt, distance_transform_edt
from scipy.special import softmax
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
from .prior_samples_with_costs import *
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
DEFAULT_OBS_COST = 1e4
DEFAULT_DIST_WEIGHT = 10
class Config:
  """ Configurations that are typically fixed throughout execution. """
  def __init__(self, 
               T=3, # Horizon (s)
               dt=0.2, # Length of each step (s)
               num_control_rollouts=1000, # Number of control sequences
               num_vis_state_rollouts=1000, # Number of visualization rollouts
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
      self.mu_LogN, self.std_LogN = Normal2LogN(0, np.mean([0.05, 0.05]))
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

  def solve(self, cov, nominal_seq):
    """Entry point for different algoritims"""
    if not self.check_solve_conditions():
      print("MPPI solve condition not met. Cannot solve. Return")
      return
    return self.solve_with_nominal_dynamics(cov, nominal_seq)
  
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
           v_switch_d, \
           u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d, \
           costmap_origin_x, costmap_origin_y, costmap_resolution

  def solve_with_nominal_dynamics(self, covs, nominal_seq):
    """
    Launch GPU kernels that use nominal dynamics but adjsuts cost function based on worst-case linear speed.
    """
    vrange_d, wrange_d, xgoal_d, goal_tolerance_d, lambda_weight_d, \
    vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d,\
    v_switch_d, \
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
      noise_samples_reshaped[:,:,0] *= 1            #NOTE: this part is different
      noise_samples_reshaped[:,:,1] *= covs[:,0]
      self.noise_samples_d = numba_cuda.to_device(noise_samples_reshaped)
      nominal_seq_velocity = np.ones((nominal_seq.shape[0], 1))
      nominal_seq_theta = nominal_seq[:,:,0]
      final_nominal_seq = np.hstack((nominal_seq_velocity, nominal_seq_theta))
      self.u_cur_d = numba_cuda.to_device(final_nominal_seq)
      
      # Rollout and compute mean or cvar
      self.rollouts_cost_numba[self.num_control_rollouts, 1](
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
        self.noise_samples_d,
        self.u_cur_d,
        costmap_origin_x,
        costmap_origin_y,
        params_costmap_resolution,

        # results
        self.costs_d,
      )      
      self.u_prev_d = final_nominal_seq

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

  """GPU kernels from here on"""
  @staticmethod
  @numba_cuda.jit(fastmath=True)
  def rollouts_cost_numba(
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
          noise_samples_d,
          u_cur_d,
          costmap_origin_x,
          costmap_origin_y,
          params_costmap_resolution,
          costs_d,
        ):
    """
    There should only be one thread running in each block, where each block handles a single sampled control sequence.
    """
    # Get block id and thread id
    bid = numba_cuda.blockIdx.x   # index of block
    costs_d[bid] = 0.0

    # Explicit unicycle update and map lookup
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
      # kinematic model update
      x_curr[0] += dt_d*v_noisy*math.cos(x_curr[2])
      x_curr[1] += dt_d*v_noisy*math.sin(x_curr[2])
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
        costs_d[bid] += calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d) / (49*100) * obs_cost_d

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
class SVGuidedMPPI:
    def __init__(self, svg_mppi_params, mppi_params):
        self.vehicle_length = 0.57
        self.vehicle_width = 0.3
        self.vehicle_wheelbase = 0.32
        self.local_costmap_size = 120
        self.local_costmap_resolution = 0.05
        self.params = mppi_params
        self.costmap_loaded = False
        self.prediction_step_size_ = svg_mppi_params["prediction_step_size"]
        self.lambda_ = svg_mppi_params["lambda"]
        self.alpha_ = svg_mppi_params["alpha"]
        self.non_biased_sampling_rate_ = svg_mppi_params["non_biased_sampling_rate"]
        self.steer_cov_ = svg_mppi_params["steer_cov"]
        self.grad_lambda_ = svg_mppi_params["grad_lambda"]
        self.steer_cov_for_grad_estimation_ = svg_mppi_params["steer_cov_for_grad_estimation"]
        self.svgd_step_size_ = svg_mppi_params["svgd_step_size"]
        self.num_svgd_iteration_ = svg_mppi_params["num_svgd_iteration"]
        self.is_use_nominal_solution_ = svg_mppi_params["is_use_nominal_solution"]
        self.is_covariance_adaptation_ = svg_mppi_params["is_covariance_adaptation"]
        self.gaussian_fitting_lambda_ = svg_mppi_params["gaussian_fitting_lambda"]
        self.min_steer_cov = svg_mppi_params["min_steer_cov"]
        self.max_steer_cov = svg_mppi_params["max_steer_cov"]
        self.sample_batch_num = svg_mppi_params["sample_batch_num"]
        self.guide_sample_num = svg_mppi_params["guide_sample_num"]
        self.sample_num_for_grad_estimation_ = svg_mppi_params["sample_num_for_grad_estimation"]
        self.max_steer_angle = svg_mppi_params["max_steer_angle"]
        self.min_steer_angle = svg_mppi_params["min_steer_angle"]
        self.max_control_inputs = np.array([self.max_steer_angle])
        self.min_control_inputs = np.array([self.min_steer_angle])
        self.prior_samples_ptr_ = PriorSamplesWithCosts(
           self.sample_batch_num, self.prediction_step_size_, self.max_control_inputs, 
           self.min_control_inputs, self.non_biased_sampling_rate_, 42
        )
        
        self.guide_samples_ptr_ = PriorSamplesWithCosts(
           self.guide_sample_num, self.prediction_step_size_, self.max_control_inputs, 
           self.min_control_inputs, self.non_biased_sampling_rate_, 42
        )
        
        self.prev_control_seq_ = self.prior_samples_ptr_.get_zero_control_seq()
        self.nominal_control_seq_ = self.prior_samples_ptr_.get_zero_control_seq()

        self.control_seq_cov_matrices = self.guide_samples_ptr_.get_constant_control_seq_cov_matrices(self.steer_cov_)
        self.generator = XORWOWRandomNumberGenerator()
        self.guide_samples_ptr_.random_sampling_numba(self.generator, self.guide_samples_ptr_.get_zero_control_seq(), self.control_seq_cov_matrices)
        self.num_vis_state_rollouts = 200
        self.state_rollout_batch_d = numba_cuda.device_array((self.num_vis_state_rollouts, self.prediction_step_size_, 3), dtype=np.float32)
        self.grad_sampler_ptrs_ = list()
        
        for i in range(self.sample_batch_num):
            self.grad_sampler_ptrs_.append(PriorSamplesWithCosts(self.sample_num_for_grad_estimation_, self.prediction_step_size_,
                                                            self.max_control_inputs, self.min_control_inputs,
                                                            self.non_biased_sampling_rate_, i))

    """GPU kernels from here on"""
    @staticmethod
    @numba_cuda.jit(fastmath=True)
    def rollouts_cost_numba(
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
          noise_samples_d,
          costmap_origin_x,
          costmap_origin_y,
          params_costmap_resolution,
          costs_d,
        ):
        """
        There should only be one thread running in each block, where each block handles a single sampled control sequence.
        """
        # Get block id and thread id
        bid = numba_cuda.blockIdx.x   # index of block
        costs_d[bid] = 0.0

        # Explicit unicycle update and map lookup
        x_curr = numba_cuda.local.array(3, numba.float32) # x, y, theta
        x_curr_grid_d = numba_cuda.local.array((2), dtype=np.int32)

        for i in range(3): 
            x_curr[i] = x0_d[i]

        timesteps = noise_samples_d.shape[1]
        goal_reached = False
        isCollided = False

        # squared goal tolerance
        goal_tolerance_d2 = goal_tolerance_d*goal_tolerance_d
        dist_to_goal2 = prev_dist_to_goal2 = 1e9

        v_nom = v_noisy = w_nom = w_noisy = 0.0

        gamma = 1.0
        for t in range(timesteps):
            # Nominal noisy control
            w_nom = noise_samples_d[bid, t, 0] #NOTE: this line is different
            w_noisy = max(wrange_d[0], min(wrange_d[1], w_nom))
            v_noisy = vrange_d[0] # fixed speed 1.0

            # Forward simulate
            # kinematic model update
            x_curr[0] += dt_d*v_noisy*math.cos(x_curr[2])
            x_curr[1] += dt_d*v_noisy*math.sin(x_curr[2])
            x_curr[2] += dt_d*v_noisy*math.tan(w_noisy)/vehicle_wheelbase_d
            # x_curr[2] = math.fmod(x_curr[2], 2*math.pi)

            convert_position_to_costmap_indices_gpu(
                x_curr[0],
                x_curr[1],
                costmap_origin_x,
                costmap_origin_y,
                params_costmap_resolution,
                x_curr_grid_d,
            )
            costs_d[bid] += calculate_localcostmap_cost(local_costmap_d, x_curr_grid_d) / (100) * obs_cost_d

            # Check the state is collided with the obstacle
            # Get current state costmap indices
            if not isCollided:
                if check_state_collision_gpu(local_costmap_d, x_curr_grid_d) == 1.0:
                    isCollided = True

                # distance to goal cost
                dist_to_goal2 = (((xgoal_d[0]-x_curr[0])**2) + ((xgoal_d[1]-x_curr[1])**2)) ** 0.5
                costs_d[bid] += stage_cost(dist_to_goal2, 5.0)
                if dist_to_goal2 <= goal_tolerance_d:
                    goal_reached = True
                    break
                prev_dist_to_goal2 = dist_to_goal2
            else:
                # costs_d[bid] +=  1 * obs_cost_d
                costs_d[bid] += stage_cost(prev_dist_to_goal2, 5.0) # distance to goal cost
    	# Accumulate terminal cost
        costs_d[bid] += term_cost(dist_to_goal2, goal_reached)

    def move_mppi_task_vars_to_device(self, x0):
        vrange_d = numba_cuda.to_device(self.params['vrange'].astype(np.float32))
        wrange_d = numba_cuda.to_device(self.params['wrange'].astype(np.float32))
        xgoal_d = numba_cuda.to_device(self.params['xgoal'].astype(np.float32))
        goal_tolerance_d = np.float32(self.params['goal_tolerance'])
        lambda_weight_d = np.float32(self.params['lambda_weight'])

        u_std_d = numba_cuda.to_device(self.params['u_std'].astype(np.float32))
        self.u_std = self.params['u_std'].astype(np.float32)
        x0_d = numba_cuda.to_device(x0.astype(np.float32))
        dt_d = np.float32(self.params['dt'])
        vehicle_length_d = np.float32(self.vehicle_length)
        vehicle_width_d = np.float32(self.vehicle_width)
        vehicle_wheelbase_d = np.float32(self.vehicle_wheelbase)
        v_switch_d = np.float32(self.params['v_switch'])

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
                v_switch_d, \
                u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d, \
                costmap_origin_x, costmap_origin_y, costmap_resolution

    def func_calc_costs(self, sampler, initial_state):
        vrange_d, wrange_d, xgoal_d, goal_tolerance_d, lambda_weight_d, \
        vehicle_length_d, vehicle_width_d, vehicle_wheelbase_d,\
        v_switch_d, \
        u_std_d, x0_d, dt_d, local_costmap_d, obs_cost_d, max_local_cost_d, \
        costmap_origin_x, costmap_origin_y, params_costmap_resolution = self.move_mppi_task_vars_to_device(initial_state)

        noise_samples_d = numba_cuda.to_device(sampler.noised_control_seq_samples_)
        self.costs_d = numba_cuda.device_array((noise_samples_d.shape[0]), dtype=np.float32)
        dist_weight = DEFAULT_DIST_WEIGHT if 'dist_weight' not in self.params else self.params['dist_weight']
        self.rollouts_cost_numba[noise_samples_d.shape[0], 1](
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
            noise_samples_d,
            costmap_origin_x,
            costmap_origin_y,
            params_costmap_resolution,
            # results
            self.costs_d,
      )
        return np.array(self.costs_d.copy_to_host())
    
    def solve(self, initial_state):
        costs_history = list()
        control_seq_history = list()
        ## Stein Gradient Descent
        for _ in range(self.num_svgd_iteration_):
            # Transport samples by Stein Variational Gradient Descent
            ## Calculating Stein gradient
            grad_log_posterior_batch = self.approx_grad_posterior_batch(self.guide_samples_ptr_, self.func_calc_costs, initial_state) #runtime around 0.05, manageable
            for i in range(self.guide_samples_ptr_.get_num_samples()): # for loop runtime negligible
                # Apply SVGD step
                self.guide_samples_ptr_.noised_control_seq_samples_[i] += self.svgd_step_size_ * grad_log_posterior_batch[i]
            # Calculate costs for the current batch
            costs = self.func_calc_costs(self.guide_samples_ptr_, initial_state) # runtime 0.01-0.03
            # Store costs and samples for adaptive covariance calculation
            costs_history.extend(costs)
            control_seq_history.extend(self.guide_samples_ptr_.noised_control_seq_samples_.tolist())
        guide_costs = self.func_calc_costs(self.guide_samples_ptr_, initial_state)  # Extract first element (costs)
        # # Find the index of the minimum cost
        min_idx = np.argmin(guide_costs)
        # Retrieve the best control sequence
        best_particle = self.guide_samples_ptr_.noised_control_seq_samples_[min_idx,:,:]
        covs = self.prior_samples_ptr_.get_constant_control_seq_cov_matrices([self.steer_cov_])
        control_seq_history = np.array(control_seq_history)
        if (self.is_covariance_adaptation_):
            softmax_costs = softmax(np.array(costs_history) * -self.gaussian_fitting_lambda_)
            for i in range(self.prediction_step_size_ - 1):
                steer_samples = np.array([seq[i, 0] for seq in control_seq_history])
                
                q_star = softmax_costs
                _, sigma = self.gaussian_fitting(steer_samples, q_star)
                
                sigma_clamped = np.clip(sigma, self.min_steer_cov, self.max_steer_cov)
                covs[i] = np.identity(1) * sigma_clamped
        return best_particle, covs 

    def calc_weights(self, prior_samples_ptr_, nominal_control_seq_):
        costs_with_control_term = prior_samples_ptr_.get_costs_with_control_term_numba(self.lambda_, self.alpha_, nominal_control_seq_)
        return softmax(np.array(costs_with_control_term)* -self.lambda_)

    def gaussian_fitting(self, x, y):
        assert len(x) == len(y)
        y_hat = np.maximum(y, 1e-10) # Ensure y values are positive for the log function
        
        # Construct matrix A and vector b
        A = np.zeros((3, 3))
        b = np.zeros(3)
        
        for i in range(len(x)):
            y_hat_2 = y_hat[i] ** 2
            y_hat_log = np.log(y_hat[i])
            
            A[0, 0] += y_hat_2
            A[0, 1] += y_hat_2 * x[i]
            A[0, 2] += y_hat_2 * x[i] ** 2
            
            A[1, 0] += y_hat_2 * x[i]
            A[1, 1] += y_hat_2 * x[i] ** 2
            A[1, 2] += y_hat_2 * x[i] ** 3
            
            A[2, 0] += y_hat_2 * x[i] ** 2
            A[2, 1] += y_hat_2 * x[i] ** 3
            A[2, 2] += y_hat_2 * x[i] ** 4
            
            b[0] += y_hat_2 * y_hat_log
            b[1] += y_hat_2 * x[i] * y_hat_log
            b[2] += y_hat_2 * x[i] ** 2 * y_hat_log
        
        # Solve the linear system Au = b
        u = np.linalg.solve(A, b)
        
        # Calculate mean and variance
        eps = 1e-5
        mean = -u[1] / (2.0 * min(u[2], -eps))
        variance = np.sqrt(1.0 / (2.0 * abs(u[2])))
        return mean, variance
    
    def approx_grad_posterior_batch(self, samples, calc_costs, initial_state):
        ## Get zero control sequence
        grad_log_likelihoods = samples.get_zero_control_seq_batch() ## (1,14,1)
        ## Get mean which is 0
        mean = samples.get_mean() # (14,1)
        ## Calculate gradient
        for i in range(samples.get_num_samples()):
            grad_log_likelihood = self.approx_grad_log_likelihood_numba(
                mean, samples.noised_control_seq_samples_[i], samples.get_inv_cov_matrices(), calc_costs, self.grad_sampler_ptrs_[i], initial_state
            ) # runtime around 0.05
            grad_log_likelihoods[i] = grad_log_likelihood
        return grad_log_likelihoods
    
    def approx_grad_log_likelihood_numba(self, mean_seq, noised_seq, inv_covs, calc_costs, sampler, initial_state):
        ## Get constant covariance matrix
        grad_cov = sampler.get_constant_control_seq_cov_matrices([self.steer_cov_for_grad_estimation_])
        # # generate gaussian random samples, center of which is noised_seq which has that mean and variance
        
        sampler.random_sampling_numba(self.generator, noised_seq, grad_cov)
        # sampler.random_sampling(noised_seq, grad_cov)
        sampler.costs_ = calc_costs(sampler, initial_state)
        # calculate forward simulation and costs
        num_samples = sampler.get_num_samples()
        exp_costs = np.zeros(num_samples)
        sampler_inv_covs = sampler.get_inv_cov_matrices()
        # calculate cost with control term
        
        costs_gpu = numba_cuda.to_device(sampler.costs_.astype(np.float32))
        prev_control_seq_gpu = numba_cuda.to_device(self.prev_control_seq_.astype(np.float32))
        noised_control_seq_samples_gpu = numba_cuda.to_device(sampler.noised_control_seq_samples_.astype(np.float32))
        exp_costs_gpu = numba_cuda.to_device(exp_costs.astype(np.float32))
        sum_of_grads_gpu = numba_cuda.device_array((np.int32(num_samples), mean_seq.shape[0], mean_seq.shape[1]), dtype = np.float32)
        inv_covs_gpu = numba_cuda.to_device(inv_covs.astype(np.float32))
        sampler_inv_covs_gpu = numba_cuda.to_device(sampler_inv_covs.astype(np.float32))
        noised_seq_gpu = numba_cuda.to_device(noised_seq.astype(np.float32))
        pred_size_gpu = numba_cuda.to_device(np.int64(self.prediction_step_size_))
        lamba_gpu = numba_cuda.to_device(np.float32(self.grad_lambda_))
        
        self.calculate_grad[num_samples, 1](
        costs_gpu,
        self.prediction_step_size_,
        self.grad_lambda_,
        prev_control_seq_gpu,
        noised_control_seq_samples_gpu,
        exp_costs_gpu,
        sum_of_grads_gpu,
        inv_covs_gpu,
        sampler_inv_covs_gpu,
        noised_seq_gpu)
        
        exp_costs_final = exp_costs_gpu.copy_to_host()
        sum_of_grads_final = np.sum(sum_of_grads_gpu.copy_to_host(), axis=0)
        sum_of_costs = np.sum(exp_costs_final)
        return sum_of_grads_final / (sum_of_costs + 1e-10)
    
    @staticmethod
    @numba_cuda.jit(fastmath=True)
    def calculate_grad(
        costs,
        prediction_step_size_,
        grad_lambda_,
        prev_control_seq_,
        noised_control_seq_samples_,
        exp_costs, ## Saving in this
        sum_of_grads, ## Saving in this
        inv_covs,
        sampler_inv_covs,
        noised_seq
    ):
        # Use block id
        bid = numba_cuda.blockIdx.x
        cost_with_control_term = costs[bid]
        current_noised_control_seq = noised_control_seq_samples_[bid]
        for j in range(prediction_step_size_- 1):
            prev_control_diff = cuda.local.array(15, dtype=np.float32)  # Temporary local array for subtraction result
            for k in range(15):
                prev_control_diff[k] = prev_control_seq_[j, k] - current_noised_control_seq[j, k]
        
            # Manually subtract elements for the second term
            current_noised_diff = cuda.local.array(15, dtype=np.float32)  # Temporary local array for second subtraction
            for k in range(15):
                current_noised_diff[k] = prev_control_seq_[j, k] - current_noised_control_seq[j, k]

            diff_control_term = 0.0
        
            # Matrix multiplication manually: prev_control_diff @ inv_covs[j] @ prev_control_diff.T
            for k in range(prev_control_seq_.shape[1]):
                diff_control_term += prev_control_diff[k] * inv_covs[j][k, k]  # Assuming inv_covs[j] is diagonal, adjust if necessary
            # Manually apply grad_lambda_
            grad_lambda_ = np.float32(grad_lambda_)
            diff_control_term = np.float32(diff_control_term)
            diff_control_term = diff_control_term * grad_lambda_
            cost_with_control_term += diff_control_term
        exp_cost = 1.0 / (1.0 + np.float32(-cost_with_control_term / grad_lambda_))
        exp_costs[bid] = exp_cost
        exp_cost = np.float32(exp_cost)
        for j in range(prediction_step_size_ - 1): 
            current_inv_cov = sampler_inv_covs[j,0,0]
            current_noised_control_seq_samples_ = noised_control_seq_samples_[bid, j,0]
            current_noised_seq = noised_seq[j,0]
            new_term = exp_cost * current_inv_cov * (current_noised_control_seq_samples_ - current_noised_seq)
            sum_of_grads[bid,j,0] = new_term

class SteinMPPIPlannerNode(Node):
    def __init__(self):
        super().__init__('Stein_MPPI_Planner_Node')
        self.cfg = Config(T = 3,
            dt = 0.2,
            num_control_rollouts = 500, # Same as number of blocks, can be more than 1024
            num_vis_state_rollouts = 1000,
            seed = 1,
            )
        ## Change the variance for MPPI
        self.mppi_params = dict(
            # Task specification
            dt = self.cfg.dt, 
            xgoal = np.array([-1.0, -15.0]), # Goal position (x, y)
            # vehicle length(lf and lr wrt the cog) and width
            vehicle_length = 0.57,
            vehicle_width = 0.3,
            vehicle_wheelbase = 0.32,
            # For risk-aware min time planning
            goal_tolerance = 0.4,
            dist_weight = 10, #  Weight for dist-to-goal cost.

            lambda_weight = 1.0, # Temperature param in MPPI
            num_opt = 1, # Number of steps in each solve() function call.

            # Control and sample specification
            u_std = np.array([0.023, 0.1]), # Noise std for sampling linear and angular velocities.
            vrange = np.array([1.0, 1.0]), # Linear velocity range. Constant Linear Velocity
            wrange = np.array([-np.pi/6, np.pi/6]), # Angular velocity range.
            v_switch = 0.2, 
            ## obstacles
            costmap = None,
            obs_penalty = 1e2,
        )
        self.mppi = MPPI_Numba(self.cfg)
        self.mppi.setup(self.mppi_params)
        self.get_logger().info('MPPI NUMBA initialized.')
        ## Change the max_steer_cov, steer_cov according to the experiment
        self.svg_mppi_params = {
            "sample_batch_num": 2000,
            "lambda": 1.0, # temperature parameter [0, inf) of free energy, which is a balancing term between control cost and state cost.
            "non_biased_sampling_rate": 0.1, # [0, 1]. add random noise to candidate control sequence with this rate.
            "alpha": 0.1, # weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal control sequence.
            "steer_cov": 0.01, # initial covariance or constant covariance if is_covariance_adaptation is false
            "guide_sample_num": 1,
            "grad_lambda": 1.0,
            "sample_num_for_grad_estimation": 100,
            "steer_cov_for_grad_estimation": 0.01,
            # "svgd_step_size": 0.005,
            "svgd_step_size": 0.015,
            "num_svgd_iteration": 2,
            "is_use_nominal_solution": True,
            "is_covariance_adaptation": True,
            "gaussian_fitting_lambda": 0.1,
            "min_steer_cov": 0.001,
            "max_steer_cov": 0.2,
            "prediction_step_size": 16,
            "max_steer_angle": np.pi/6,
            "min_steer_angle": -np.pi/6,
        }
        self.stein = SVGuidedMPPI(self.svg_mppi_params, self.mppi_params)
        self.get_logger().info('SVG MPPI initialized.')

        # Publishers, subscribers, and tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        '''############### for costmap ##############'''
        self.local_costmap = None  # store the latest costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, # Type: nav_msgs/msg/OccupancyGrid
            '/local_costmap/costmap',
            self.costmap_callback,
            1 # only the most recent message is kept in the queue
        )
        self.path_pub = self.create_publisher(Path, "/min_cost_path", 10)
        self.mppi_path_pub = self.create_publisher(Path, "/mppi_path", 10)

        self.no_costmap_received_timer = self.create_timer(2.0, self.notify_no_costmap)
        self.action_pub = self.create_publisher(
            msg_type=AckermannDriveStamped,
            topic="/drive",
            qos_profile=qos_profile_sensor_data,
        )

        # Create a timer to call the Stein planner solve routine.
        self.timer = self.create_timer(0.25, self.solve_Stein)
        self.i = 0
        self.isGoalReached = False

    def notify_no_costmap(self):
        if self.local_costmap is None:
            self.get_logger().warn("No /local_costmap/costmap data received yet...")

    def costmap_callback(self, msg: OccupancyGrid):
        # Convert msg.data to a 2D list or np.array
        width = msg.info.width
        height = msg.info.height
        # Convert msg data to float costmap and store resolution/origin
        costmap_int8 = np.array(msg.data, dtype=np.int8).reshape(height, width)
        self.local_costmap = costmap_int8.astype(np.float32)

        self.mppi_params['costmap_resolution'] = msg.info.resolution
        self.mppi_params['costmap_origin'] = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.mppi_params['costmap'] = self.local_costmap

    def _dynamics_KS_3d_steering_angle(self, state, action, dt): #constant velocity
        x, y, theta = state
        steering_angle, v = action
        L_wb = 0.324 # wheelbase for F1Tenth
        x_new = x + v * np.cos(theta) * dt 
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + v/L_wb * np.tan(steering_angle) * dt
        return (x_new, y_new, theta_new) 
    
    def _state_to_pose(self, state):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(state[0])
        pose.pose.position.y = float(state[1])
        pose.pose.position.z = 0.0
        q = R.from_euler('z', float(state[2])).as_quat()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def solve_Stein(self):
        try:
            solve_Stein_whole_start = time.perf_counter()
            # 1. Look up transform from map -> base_link
            transform = self.tf_buffer.lookup_transform(
                'map',           # source frame (or "map")
                # 'base_link',   # target frame (your robot)
                'laser',         # target frame (your robot)
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
            self.mppi_params['x0'] = current_state
            if self.local_costmap is not None:
                self.mppi_params['costmap'] = self.local_costmap
            self.mppi.setup(self.mppi_params)
            refined_control = self.solve_Stein_core(current_state)

            # Log some status information
            h = std_msgs.msg.Header()
            h.stamp = self.get_clock().now().to_msg()
            if (self.i % 10) == 0:
                self.get_logger().info('-----------------')
            if self.isGoalReached:
                refined_control = [0.0, 0.0]
                self.get_logger().info("Goal Reached!!!!")
            # Create and publish drive command based on refined_control.
            drive = AckermannDrive(steering_angle=float(refined_control[1]), speed=float(refined_control[0]))
            data = AckermannDriveStamped(header=h, drive=drive)
            self.action_pub.publish(data)
            
            # Compute distance to goal and update goal status.
            dist2goal2 = (self.mppi_params['xgoal'][0] - x_robot)**2 + (self.mppi_params['xgoal'][1] - y_robot)**2
            goaltol2 = self.mppi_params['goal_tolerance'] ** 2
            if dist2goal2 < goaltol2:
                self.isGoalReached = True
            else:
                self.isGoalReached = False
            self.i += 1
            if (self.i % 10) == 0:
                self.get_logger().info(f"  Distance to the Goal: {dist2goal2}, Goal Tolerance: {goaltol2:2f}")
                self.get_logger().info(f"  WHOLE solve_Stein runtime {time.perf_counter()-solve_Stein_whole_start}")
        except Exception as e:
            tb_str = ''.join(traceback.format_exception(None, e, e.__traceback__))
            self.get_logger().warn(f"Cannnot run solve_Stein: {e}\n{tb_str}")

    def solve_Stein_core(self, current_state):
        stein_start = time.perf_counter()
        cov, nominal_seq = self.stein.solve(current_state)
        if (self.i % 10) == 0:
            self.get_logger().info(f"  runtime stein.solve {time.perf_counter()-stein_start}")
        mppi_strat = time.perf_counter()
        useq = self.mppi.solve(cov, nominal_seq)
        if (self.i % 10) == 0:
            self.get_logger().info(f"  runtime mppi.solve {time.perf_counter()-mppi_strat}")
        self.mppi.shift_and_update(current_state, useq)

        ############### visualize min cost traj below ##############
        visualize_traj = False
        if visualize_traj:
            # Visualize minimum cost trajectory as a Path message
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            propagated_state = current_state.copy()
            path_msg.poses.append(self._state_to_pose(propagated_state))
            for action in nominal_seq:
                action = np.reshape(action, (-1,))
                swapped_action = (action[0], 1.0)
                propagated_state = self._dynamics_KS_3d_steering_angle(propagated_state, swapped_action, self.cfg.dt)
                path_msg.poses.append(self._state_to_pose(propagated_state))
            self.path_pub.publish(path_msg)

            # Visualize final MPPI trajectory by propagating the state using refined control sequence.
            mppi_path_msg = Path()
            mppi_path_msg.header.frame_id = "map"
            mppi_path_msg.header.stamp = self.get_clock().now().to_msg()
            propagated_state = current_state.copy()
            mppi_path_msg.poses.append(self._state_to_pose(propagated_state))
            for action in useq:
                # Swap action order: [v, steering_angle] -> (steering_angle, v)
                swapped_action = (action[1], action[0])
                propagated_state = self._dynamics_KS_3d_steering_angle(propagated_state, swapped_action, self.cfg.dt)
                mppi_path_msg.poses.append(self._state_to_pose(propagated_state))
            self.mppi_path_pub.publish(mppi_path_msg)
        return useq[0] # return the first control command 

    def on_shutdown(self):
        # Clean up the context for both numba and cuda
        self.get_logger().info('Stein-MPPI Planner Node shutting down')
        self.get_logger().info("Popping CUDA context...")
        primary_context.pop()
        primary_context.detach()

def main(args=None):
    np.random.seed(1)
    rclpy.init(args=args)
    node = SteinMPPIPlannerNode()
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