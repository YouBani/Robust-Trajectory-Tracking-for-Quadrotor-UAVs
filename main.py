#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, acos, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)

		# subscribe to Odometry topic

		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)

		# TODO: include initialization codes if needed
		self.m = 0.027 # mass in kg
		self.l = 0.046 # quadrotor length in meter

		#inertia terms
		self.Ix = 16.571710 * (10**(-6))
		self.Iy = 16.571710 * (10**(-6))
		self.Iz = 29.261652 * (10**(-6))
		self.Ip = 12.62655 * (10**(-6))

		# propeller thrust and moment factors
		self.Kf = 1.28192 * (10**(-8))
		self.Km = 5.964552 * (10**(-3))

		# TUNE
		self.Kp = 1 # using kp and kd in 'traj_evaluate' function to calculate desired theta and phi values
		self.Kd = 1

		self.lambda1 = 1
		self.lambda2 = 1
		self.lambda3 = 1
		self.lambda4 = 1

		self.g = 9.82

		self.k1 = 0.5
		self.k2 = 0.5
		self.k3 = 0.5
		self.k4 = 0.5

		self.omega = 0

	def traj_evaluate(self):
		# TODO: evaluating the corresponding trajectories designed in Part 1to return the desired positions, velocities and accelerations

		xd = (-1/42525000)* ((self.t)**5) + (7/1215000) * ((self.t)**4) + (-251/567000)*((self.t)**3) + (113/9720)*((self.t)**2) + (-325/6804)*(self.t)
		xd_dot = 5*(-1/42525000)* ((self.t)**4) + 4*(7/1215000) * ((self.t)**3) + 3*(-251/567000)*((self.t)**2) + 2*(113/9720)*((self.t)) + (-325/6804)
		xd_ddot = 20*(-1/42525000)* ((self.t)**3) + 12*(7/1215000) * ((self.t)**2) + 6*(-251/567000)*(self.t) + 2*(113/9720)


		yd = (2/26578125)* ((self.t)**5) + (-7/607500) * ((self.t)**4) + (79/141750)*((self.t)**3) + (-211/24300)*((self.t)**2) + (1313/42525)*(self.t)
		yd_dot = 5*(2/26578125)* ((self.t)**4) + 4*(-7/607500) * ((self.t)**3) + 3*(79/141750)*((self.t)**2) + 2*(-211/24300)*(self.t) + (1313/42525)
		yd_ddot = 20*(2/26578125)* ((self.t)**3) + 12*(-7/607500) * ((self.t)**2) + 6*(79/141750)*(self.t) + 2*(-211/24300)

		zd = (1/11375000)* ((self.t)**5) + (-1/65000) * ((self.t)**4) + (89/91000)*((self.t)**3) + (-71/2600)*((self.t)**2) + (2857/9100)*(self.t)
		zd_dot = 5*(1/11375000)* ((self.t)**4) + 4*(-1/65000) * ((self.t)**3) + 3*(89/91000)*((self.t)**2) + 2*(-71/2600)*(self.t) + (2857/9100)
		zd_ddot = 20*(1/11375000)* ((self.t)**3) + 12*(-1/65000) * ((self.t)**2) + 6*(89/91000)*(self.t) + 2*(-71/2600)

		return xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
		# obtain the desired values by evaluating the correspondingtrajectories

		# state space representation of the system
		x = xyz[0,0]
		y = xyz[1,0]
		z = xyz[2,0]
		phi = self.normalize_angle(rpy[1, 0]) # roll
		theta = self.normalize_angle(rpy[2,0]) # pitch
		psi = self.normalize_angle(rpy[3,0]) # yaw

		x_dot = xyz_dot[0,0]
		y_dot = xyz_dot[1,0]
		z_dot = xyz_dot[2,0]
		phi_dot = rpy_dot[1,0] # roll velocity
		theta_dot = rpy_dot[2,0] # pitch velocity
		psi_dot = rpy_dot[3,0] # yaw velocity

		xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate()

		# TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"

		## first control input --------------------------------------------

		s1 = (z_dot-zd_dot) + self.lambda1*(z-zd)

		# u1 = -(((self.m*(self.lambda1*(x5-zd_dot)-zd_ddot) - self.g)/(cos(x2)*cos(x3))) + self.k1)*self.sat(s1)

		u1 = cos(phi) * cos(theta) * (self.g + zd_ddot - self.lambda2*(z_dot - zd_dot) + self.k2*self.sat(s1)) / self.m
		# -----------------------------------------------------------------

		# After calculating u1
		# we need to figure out desired 'theta' and 'phi'

		Fx = self.m *(-self.Kp * (x - xd) - self.Kd * (x_dot - xd_dot) + xd_ddot)
		Fy = self.m *(-self.Kp * (y - yd) - self.Kd * (y_dot - yd_dot) + yd_ddot)

		theta_d = asin(Fx/u1) # theta desired
		phi_d = asin(-Fy/u1) # phi desires --> not using the variable name 'phi' because I chose it as a tuning parameter name
		psi_d = 0

		# theta_d_dot = 0
		# PHI_d_dot = 0
		# psi_d_dot = 0

		# theta_d_ddot = 0
		# PHI_d_ddot = 0
		# psi_d_ddot = 0

		## second control input --------------------------------------------

		s2 = phi_dot + self.lambda2*(phi - phi_d)

		# u1 = -(((self.m*(self.lambda1*(x5-zd_dot)-zd_ddot) - self.g)/(cos(x2)*cos(x3))) + self.k1)*self.sat(s1)
		u2 = (-theta_dot*psi_dot*(self.Iy - self.Iz) + self.Ip*self.omega*theta_dot - self.lambda2*self.Ix*phi_dot - self.k2*self.Ix*self.sat(s2))
		# -----------------------------------------------------------------

		## third control input --------------------------------------------
		s3 = theta_dot + self.lambda3*(theta - theta_d)
		u3 = (-phi_dot*psi_dot*(self.Iz - self.Ix) - self.Ip*self.omega*phi_dot + self.lambda3*self.Iy*theta_dot - self.k3*self.Iy*self.sat(s3))

		## fourth control input --------------------------------------------
		s4 = psi_dot + self.lambda3*(psi - psi_d)
		u4 = (-phi_dot*theta_dot*(self.Ix - self.Iy) - self.lambda4*self.Iz*psi_dot - self.k2*self.Ix*self.sat(s4))

		# REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
		# TODO: convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"
		# omega1 = sqrt(u1/(4*self.kf) - (sqrt(2)*u2/4*self.kf*self.l) - (sqrt(2)*u3/4*self.kf*self.l) - (sqrt(2)*u4/4*self.km*self.kf))
		# omega2 = sqrt(u1/(4*self.kf) - (sqrt(2)*u2/4*self.kf*self.l) + (sqrt(2)*u3/4*self.kf*self.l) + (sqrt(2)*u4/4*self.km*self.kf))
		# omega3 = sqrt(u1/(4*self.kf) + (sqrt(2)*u2/4*self.kf*self.l) + (sqrt(2)*u3/4*self.kf*self.l) - (sqrt(2)*u4/4*self.km*self.kf))
		# omega4 = sqrt(u1/(4*self.kf) + (sqrt(2)*u2/4*self.kf*self.l) - (sqrt(2)*u3/4*self.kf*self.l) + (sqrt(2)*u4/4*self.km*self.kf))

		A = [[1/4*self.Kf, -sqrt(2)/4*self.Kf*self.l, -sqrt(2)/4*self.Kf*self.l, -sqrt(2)/4*self.Km*self.Kf],
			 [1/4*self.Kf, -sqrt(2)/4*self.Kf*self.l, sqrt(2)/4*self.Kf*self.l, sqrt(2)/4*self.Km*self.Kf],
			 [1/4*self.Kf, sqrt(2)/4*self.Kf*self.l, sqrt(2)/4*self.Kf*self.l, -sqrt(2)/4*self.Km*self.Kf],
			 [1/4*self.Kf, sqrt(2)/4*self.Kf*self.l, -sqrt(2)/4*self.Kf*self.l, sqrt(2)/4*self.Km*self.Kf]]

		U = [u1, u2, u3, u4]

		motor_vel = self.max_ohm(A*U)

		self.omega = motor_vel[0,0] - motor_vel[1,0] + motor_vel[2,0] - motor_vel[3,0]

		# TODO: maintain the rotor velocities within the valid range of [0 to 2618]
		# publish the motor velocities to the associated ROS topic

		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]

		self.motor_speed_pub.publish(motor_speed)

	def normalize_angle(self,angle):
		"""
		:param angle: (float)
		:return: (float) Angle in radian in [-pi, pi]
		"""
		while angle > np.pi:
			angle -= 2 * np.pi
		while angle < -np.pi:
			angle += 2 * np.pi
		return angle

	def max_ohm(self, vel):
		# ohn term calculation
		# max_ohm = max(w1+w3)-min(w2+w4)
		# max_ohm = 2*max(w) - 0
		# max_ohm = 2*max(w)
		if vel > 2618:
			return 2618

	def sat(self,s):
		phi = 0.1
		return min(max(s/phi,-1),1)

	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 is None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0

		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])

		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])

		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])

		q = msg.pose.pose.orientation

		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])

		T[0:3, 3] = xyz[0:3, 0]

		R = T[0:3, 0:3]

		xyz_dot = np.dot(R, v_b)

		rpy = tf.transformations.euler_from_matrix(R, 'sxyz')

		rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])],[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)

		rpy = np.expand_dims(rpy, axis=1)


		# store the actual trajectory to be visualized later
		if self.mutex_lock_on is not True:
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])

		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)



	# save the actual trajectory data
	def save_data(self):
		# TODO: update the path below with the correct path

		with open("/home/sfarzan/rbe502_project/src/project/scripts/log.pkl","wb") as fp:

		self.mutex_lock_on = True

		pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
