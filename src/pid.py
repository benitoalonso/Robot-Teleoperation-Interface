import numpy as np
from lab_utils.orientation_utils import *
from copy import deepcopy
from scipy.optimize import minimize
from lab_utils.global_values import *

class PID:
	def __init__(self, 
					kp_linear = 1.5, kd_linear = 0.1, ki_linear = 0.0, 
					kp_angular = 0.3, kd_angular = 0.1, ki_angular = 0.0):
     
		self.kp_linear = kp_linear
		self.kd_linear = kd_linear
		self.ki_linear = ki_linear

		self.kp_angular = kp_angular
		self.kd_angular = kd_angular
		self.ki_angular = ki_angular

		self.prev_error_position = 0.0
		self.prev_error_angle = 0.0

		self.prev_body_to_goal = 0.0


	# PID controller for linear velocity
	def get_v_input_pid(self, curr_state, goal_pt):			
		error_position = get_distance(curr_state[0], curr_state[1], goal_pt[0], goal_pt[1])
		linear_velocity_control = self.kp_linear*error_position + self.kd_linear*(error_position - self.prev_error_position)
		self.prev_error_position = error_position

		if linear_velocity_control>MAX_LINEAR_VELOCITY:
			linear_velocity_control = MAX_LINEAR_VELOCITY
   
		elif linear_velocity_control<MIN_LINEAR_VELOCITY:
			linear_velocity_control = MIN_LINEAR_VELOCITY
   
		return linear_velocity_control
	
	# PID controller for angular velocity
	def get_w_input_pid(self, curr_state, goal_pt):
		error_angle = get_angle(curr_state[0], curr_state[1], goal_pt[0], goal_pt[1]) - curr_state[2] 
		angular_velocity_control = self.kp_angular*error_angle + self.kd_angular*(error_angle - self.prev_error_angle)
		self.prev_error_angle = error_angle

		if angular_velocity_control>MAX_ANGULAR_VELOCITY:
			angular_velocity_control = MAX_ANGULAR_VELOCITY

		if angular_velocity_control<MIN_ANGULAR_VELOCITY:
			angular_velocity_control = MIN_ANGULAR_VELOCITY

		return angular_velocity_control