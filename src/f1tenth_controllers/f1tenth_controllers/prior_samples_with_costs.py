import numpy as np 
from scipy.stats import norm
from numba import cuda as numba_cuda
import math
from numba.cuda.random import create_xoroshiro128p_states, xoroshiro128p_normal_float32
from pycuda.curandom import XORWOWRandomNumberGenerator
np.random.seed(1)
class PriorSamplesWithCosts:
    def __init__(self, num_samples, prediction_horizon, max_control_inputs, min_control_inputs, non_biased_sampling_rate, seed):
        self.num_samples_ = num_samples
        self.prediction_horizon_ = prediction_horizon
        self.non_biased_sampling_rate_ = non_biased_sampling_rate
        self.max_control_inputs_ = max_control_inputs
        self.min_control_inputs_ = min_control_inputs
        ## Control space input = 1 (Only Steering)
        self.control_seq_mean_ = np.zeros((self.prediction_horizon_ - 1, 1))
        self.control_seq_cov_matrices_ = np.zeros((self.prediction_horizon_ - 1, 1, 1))
        self.control_seq_inv_cov_matrices_ = np.zeros((self.prediction_horizon_ - 1, 1, 1))
        self.noised_control_seq_samples_ = np.zeros((self.num_samples_, self.prediction_horizon_ - 1, 1))
        self.noise_seq_samples_ = np.zeros((self.num_samples_, self.prediction_horizon_ - 1, 1))
        self.costs_ = np.zeros(self.num_samples_)
        self.normal_dists_ptr_ = np.zeros((self.prediction_horizon_ - 1, 1))
        
        for i in range(self.prediction_horizon_ - 1):
            # Store the normal distribution object (with mean 0 and std 1) in the array
            self.normal_dists_ptr_[i, 0] = np.random.normal(0.0, 1.0)
    
    def get_costs_with_control_term(self, lambda_val, alpha, nominal_control_seq):
        costs_with_control_term = self.costs_
        for i in range(self.num_samples_):
            for j in range(self.prediction_horizon_ - 1):
                control_term = lambda_val * (1 - alpha) * (self.control_seq_mean_[j,:] - nominal_control_seq[j,:]) * self.control_seq_inv_cov_matrices_[j] * self.noised_control_seq_samples_[i][j]
                costs_with_control_term[i] += control_term
        return costs_with_control_term
    
    def get_costs_with_control_term_numba(self, lambda_val, alpha, nominal_control_seq):
        costs_with_control_term = self.costs_
        costs_with_control_term_gpu = numba_cuda.to_device(costs_with_control_term.astype(np.float32))
        nominal_control_seq_gpu = numba_cuda.to_device(nominal_control_seq.astype(np.float32))
        control_seq_mean_gpu = numba_cuda.to_device(self.control_seq_mean_.astype(np.float32))
        control_seq_inv_cov_matrices_gpu = numba_cuda.to_device(self.control_seq_inv_cov_matrices_.astype(np.float32))
        noised_control_seq_samples_gpu = numba_cuda.to_device(self.noised_control_seq_samples_.astype(np.float32))
        self.calculate_control_term[self.num_samples_, 1](costs_with_control_term_gpu, lambda_val, alpha, self.prediction_horizon_, nominal_control_seq_gpu, control_seq_mean_gpu, control_seq_inv_cov_matrices_gpu, noised_control_seq_samples_gpu)
        costs_with_control_term = costs_with_control_term_gpu.copy_to_host()
        return costs_with_control_term

    @staticmethod
    @numba_cuda.jit(fastmath=True)
    def calculate_control_term(
        costs_with_control_term,
        lambda_val,
        alpha,
        prediction_horizon_,
        nominal_control_seq,
        control_seq_mean,
        control_seq_inv_cov_matrices,
        noised_control_seq_samples
        ):
        tid = numba_cuda.threadIdx.x
        bid = numba_cuda.blockIdx.x
        for j in range(prediction_horizon_ - 1):
            control_1 = (control_seq_mean[j,0] - nominal_control_seq[j,0])
            control_2 = noised_control_seq_samples[bid,j,0] 
            control_2_1 = control_seq_inv_cov_matrices[j,0,0] * control_2
            control_term = lambda_val * (1 - alpha) *  control_1 * control_2_1
            costs_with_control_term[bid] += control_term
        

    def get_mean(self):
        return self.control_seq_mean_

    def get_zero_control_seq(self):
        return np.zeros((self.prediction_horizon_ - 1, 1))
    
    def get_inv_cov_matrices(self):
        return self.control_seq_inv_cov_matrices_
    
    def get_num_samples(self):
        return self.num_samples_

    def get_zero_control_seq_batch(self):
        return np.zeros((self.num_samples_, self.prediction_horizon_ - 1, 1))
    
    def get_zero_control_seq_cov_matrices(self):
        return np.zeros((self.prediction_horizon_ - 1, 1, 1))
    
    def get_constant_control_seq_cov_matrices(self, diag):
        control_seq_covs = self.get_zero_control_seq_cov_matrices()
        control_seq_covs[:, 0, 0] = diag
        return control_seq_covs
    
    def set_control_seq_mean(self, control_seq_mean):
        self.control_seq_mean_ = control_seq_mean
    
    def set_control_seq_cov_matrices(self, control_seq_cov_matrices):
        # To prevent singular matrix, add small value to diagonal elements
        eps = 1e-4
        self.control_seq_cov_matrices_ = control_seq_cov_matrices

        # Calculate the inverse of covariance matrices in advance to reduce computational cost
        for i in range(self.prediction_horizon_ - 1):
            # Add small value to diagonal to prevent singular matrix
            cov_matrix = self.control_seq_cov_matrices_[i]
            inv_cov_matrix = np.linalg.inv(cov_matrix) + eps * np.eye(1)
            self.control_seq_inv_cov_matrices_[i] = inv_cov_matrix
    
    def random_sampling(self, control_seq_mean, control_seq_cov_matrices):
        self.set_control_seq_mean(control_seq_mean)
        self.set_control_seq_cov_matrices(control_seq_cov_matrices)
        # Set normal distributions parameters
        for i in range(self.num_samples_):
            for j in range(self.prediction_horizon_ - 1):
                std_dev = np.sqrt(self.control_seq_cov_matrices_[j,0, 0])
                self.noise_seq_samples_[i,j,0] = np.random.normal(0.0, std_dev)
            if i < int((1 - self.non_biased_sampling_rate_) * self.num_samples_):
                self.noised_control_seq_samples_[i] = control_seq_mean + self.noise_seq_samples_[i]
            else:
                self.noised_control_seq_samples_[i] = self.noise_seq_samples_[i]

            for j in range(1):
                for k in range(self.prediction_horizon_ - 1):
                    self.noised_control_seq_samples_[i,k,j] = np.clip(self.noised_control_seq_samples_[i,k,j], self.min_control_inputs_[j], self.max_control_inputs_[j])
    
    def random_sampling_numba(self, generator, control_seq_mean, control_seq_cov_matrices):
        self.set_control_seq_mean(control_seq_mean)
        self.set_control_seq_cov_matrices(control_seq_cov_matrices)
        num_random_samples = self.num_samples_ * (self.prediction_horizon_ - 1)
        normal_samples = generator.gen_normal(num_random_samples, np.float32)
        normal_samples = normal_samples.get().reshape(self.num_samples_, self.prediction_horizon_ - 1, 1)
        std_devs = np.sqrt(self.control_seq_cov_matrices_[:, 0, 0]).reshape(1, -1, 1)
        normal_samples = np.array(normal_samples, dtype=np.float32)
        std_devs = np.array(std_devs, dtype=np.float32)
        self.noise_seq_samples_ = normal_samples * std_devs  # Element-wise scaling
        num_biased = int((1 - self.non_biased_sampling_rate_) * self.num_samples_)
        self.noised_control_seq_samples_[num_biased:] = self.noise_seq_samples_[num_biased:].astype(np.float64)
        # First part uses control sequence mean
        self.noised_control_seq_samples_[:num_biased] = control_seq_mean + self.noise_seq_samples_[:num_biased]
        # Second part only uses noise
        self.noised_control_seq_samples_ = np.clip(
                    self.noised_control_seq_samples_,
                    self.min_control_inputs_.reshape(1, 1, -1),
                    self.max_control_inputs_.reshape(1, 1, -1),
                        )