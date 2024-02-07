#!/usr/bin/python

# * Simplified complex trajectory tracking in OFFBOARD mode
# *
# * Author: Sean Smith <s.smith@dal.ca>
# *
# * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# * The above copyright notice and this permission notice shall be included in all
# * copies or substantial portions of the Software.


import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Imu, LaserScan

#from panel_functions import CLOVER_COMPONENTS, CLOVER_STREAM_GEOMETRIC_INTEGRAL, CLOVER_KUTTA, CLOVER_STREAMLINE, CLOVER_noOBSTACLE

from scipy.interpolate import griddata

from tf.transformations import euler_from_quaternion

import numpy as np

# Could plot the stored data in SITL (not hardware) if desired:
import matplotlib.pyplot as plt
from matplotlib import path

from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('clover_panel') # Figure8 is the name of the ROS node

# Define the Clover service functions, the only ones used in this application are navigate and land.

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Release service is used to allow for complex trajectory publishing i.e it stops the navigate service from publishing setpoints because you dont want two sources of publishing at the same time.
release = rospy.ServiceProxy('simple_offboard/release', Trigger)

# Deine math parameter
PI_2 = math.pi/2

# Debugging and logging
xf = []  # This gathers the 
yf = []
xdispf=[]
ydispf=[]
xa = []
ya = []
YawL = []
YawF = []
YawC = []

# Circle obstacle plot
xa = []
ya = []

# updated velocity field plot
u_field = []
v_field = []

# Analyze control input (see if error is being minimized )
velfx=[]
velfy=[]
velcx=[]
velcy=[]
U_infx = []
V_infy=[]
evx=[]
evy=[]
eyaw=[]


			
# This class categorizes all of the functions used for complex rajectory tracking
class clover:

	def __init__(self, FLIGHT_ALTITUDE, RATE, RADIUS, CYCLE_S, REF_FRAME): 


		# -------------------------------------------------------------------------------------------------------------------------------

		# global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.FRAME           = REF_FRAME                # Reference frame for complex trajectory tracking
		
		self.last_timestamp = None # make sure there is a timestamp
		
		# Generate the array of lidar angles
		self.lidar_angles = np.linspace(-45*(math.pi/180), 45*(math.pi/180), 32) # Adjust this number based on number defined in XACRO!!!!!

		# Initialize the ranges array for the controller as no object detected (i.e. an array of inf)
		self.obs_detect = np.full_like(self.lidar_angles, np.inf)

		# Subscribe to the Lidar readings
		self.lidar = rospy.Subscriber('/ray_scan',LaserScan,self.lidar_read)

		# Set a timer for the velocity field update function (runs periodically)
		# updates every 8 seconds (0.125 Hz)

		#rospy.Timer(rospy.Duration(3), self.velocity_field_update)

		# Set a flag, that will track if a change in envirnment occurred i.e. an object was detected
		# therefore if an object was not detected previously then there is no reason in re calculating
		# the velocity field based on source/sink strengths
		self.flag = False

		
		

		self.current_state = State()
		self.rate = rospy.Rate(20)
	
	def lidar_read(self,data):

		# Update the obstacle detect array
		self.obs_detect = data.ranges

		# Ensure there are actually lidar readings, no point in doing calculations if
		# nothing is detected:
		if any(not np.isinf(range_val) for range_val in self.obs_detect):
	
			# The angles and ranges start at -45 degrees i.e. at the right side, then go counter clockwise up to the top i.e. 45 degrees
			ranges = data.ranges
			
		
			angles = self.lidar_angles
		
			telem = get_telemetry(frame_id='map')
			
			x_clover = telem.x
			y_clover = telem.y
			yaw = telem.yaw
			
			# This transforms to the local Lidar frame where the y-axis is forward
			# and the x-axis is pointing right (the same reference frame used in the RM tracker paper, so this is the one we will use here):
			x_local2 = ranges*np.sin(angles)
			x_local2 = np.multiply(x_local2,-1)
			y_local2 = ranges*np.cos(angles)
			
			# Filter out the inf values in the data point arrays
			valid_indices = np.isfinite(x_local2) & np.isfinite(y_local2)
			
			# Filter x_local2, y_local2, and angles based on valid_indices
			x_local2_filtered = x_local2[valid_indices]
			y_local2_filtered = y_local2[valid_indices]
			angles_filtered = angles[valid_indices]
			# ranges_filtered = ranges[valid_indices]
			
			# Filter out the inf values in the data point arrays
			# x_local2 = x_local2[np.isfinite(x_local2)]
			# y_local2 = y_local2[np.isfinite(y_local2)]
			
			# The array of local data points starts at position 0 with the right most detected point, and ends with the left most detected point.
			# Therefore we will use the same setup as the RM paper and build the equation for l_1 off of the left most point (end point of the array)
			# In the local lidar reference frame, the lidar is at position (0,0)
			x_L = 0
			y_L = 0
			# Coordinate equation for stratight line l_1
			A_1 = -1*(x_local2[-1]-x_L)/(y_local2[-1]-y_L)
			B_1 = y_local2[-1] - A_1*x_local2[-1]
			# Coordinate equation for straight line l_2
			A_2 = -1*(x_local2[0] - x_L)/(y_local2[0] - y_L)
			
			# Divide Lidar range into nr intervals
			nr = 10
			
			# Initialize variables to store sector measurements and midpoints
			sector_ranges = [[] for _ in range(nr)]
			sector_xy = [[] for _ in range(nr)] # initialize sector_xy as a list of empy lists
			sector_mean = [] 
			
			# Iterate through LiDAR measurements and assign them to sectors
			for i, measurement in enumerate(ranges):
				if np.isfinite(measurement): # filter out invalid or inf measurements
					sector_index = int(i*nr/len(ranges))
					sector_ranges[sector_index].append(measurement)

					# Calculate x,y coordinates from measurements and angles (in the local Lidar ref frame 
					# where the y-axis is forward and the x-axis is pointing right)
					x_sec = measurement*np.sin(angles[i]) # select appropriate angle from angles array
					x_sec = np.multiply(x_sec,-1)
					y_sec = measurement*np.cos(angles[i])

					# Append [x, y] coordinates to sector_xy
					sector_xy[sector_index].append([x_sec,y_sec])
			
			


			# Average position per sector
			z_bar = []
			 # Calculate average x, and y for each sector for each sector
			for sector in sector_xy:
				# Convert sector to a numpy array
				sector_array = np.array(sector)

				# If sector has data, clculate [x_bar, y_bar], else set the mean to None
				if sector:
					# Calculate the x and y coordinates using np.mean along axis
					x_bar = np.mean(sector_array[:, 0]) # Calculate mean of x coordinate (column 0)
					y_bar = np.mean(sector_array[:, 1]) # Calculate mean of y coordinate
				else:
					x_bar, y_bar = None, None
					
				z_bar.append([x_bar, y_bar]) # append [x_bar, y_bar] to z_bar
			
			# Find the range of sectors with data (so we will disregard sectors before and after obstacle)
			start_sector = None
			end_sector = None
			for i, z in enumerate(z_bar):
				if z is not None:
					if start_sector is None:
						start_sector = i
					end_sector = i

			# # Ensure that the last sector with data is an even number (so we have paired sectors)
			# if end_sector is not None and end_sector % 2 != 0:
			# 	end_sector -= 1

			# Ensure that the total number of sectors with data is even
			if start_sector is not None and end_sector is not None and (end_sector - start_sector) % 2 == 0:
				end_sector -= 1

			# # Formulate z_bar_filter array that only includes values from sectors that have paired sector values
			
			# # Apply the algorithm considering only sectors with pairs of data
			# zbar_with_pairs = []
			# for i in range(start_sector, end_sector + 1, 2):
			# 	paired_index = (i + 1) if i % 2 == 0 else (i - 1)
			# 	if z_bar[i] is not None and z_bar[paired_index] is not None:
			# 		zbar_with_pairs.append(z_bar[i])
			# Apply the algorithm considering only sectors with pairs of data
			zbar_with_pairs = []
			if start_sector is not None and end_sector is not None:
				for i in range(start_sector, end_sector + 1, 2):
					paired_index = i + 1
					if z_bar[i] is not None and z_bar[paired_index] is not None:
						zbar_with_pairs.append(z_bar[i])

			print(sector_ranges)
			print(sector_xy)
			print(z_bar)
			print(zbar_with_pairs)

			# If n_bar <= threshold-----------------------------
			
			
			
			
			# Homogeneous transformation matrix for 2D (adjust global yaw by PI/2 because local Lidar frame is rotated 90 degrees relative to it):
			R_2 = np.array([[math.cos(yaw-PI_2), -math.sin(yaw-PI_2)], [math.sin(yaw-PI_2), math.cos(yaw-PI_2)]])  # rotation matrix
			
			# Combine rotation and translation into a single homogeneous transformation matrix
			T_2 = np.vstack([np.hstack([R_2, np.array([[x_clover], [y_clover]])]), [0, 0, 1]])  # Homogeneous transformation matrix
			
			# Lidar readings in homogenous coordinates
			readings_local2 = np.vstack([x_local2, y_local2, np.ones_like(x_local2)])
			
			# Transform all lidar readings to global coordinates
			readings_global2 = np.dot(T_2, readings_local2)
			
			# Extract the tranformed positions
			readings_global2 = readings_global2[:2,:].T
			
			
			
			
			
			#--------------------------------------Transformation assuming stadard polar coordinates and local y-left, local x-right---------------------
			
			# # Polar to Cartesion transformation for all readings (assuming angles are in standard polar coordinates) y-axis is left and x-axis is directly forward.
			# x_local = ranges*np.cos(angles)
			# y_local = ranges*np.sin(angles)
			
			# # Filter out the inf values in the data point arrays
			# x_local = x_local[np.isfinite(x_local)]
			# y_local = y_local[np.isfinite(y_local)]
			
			
			
			
			# # Homogenous transformation matrix for 2D
			# R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]) # rotation matrix
			

			# T = np.vstack([np.hstack([R, np.array([[x_clover], [y_clover]])]),[0,0,1]]) # Homogenous transformation matrix
			
			
			
			# # Lidar readings in homogenous coordinates
			# readings_local = np.vstack([x_local, y_local, np.ones_like(x_local)])
			# readings_local2 = np.vstack([x_local2, y_local2, np.ones_like(x_local2)])
			
			# # Transform all lidar readings to global coordinates
			# self.readings_global = np.dot(T, readings_local)
			# readings_global2 = np.dot(T_2, readings_local2)
			
			# # Extract the tranformed positions
			# self.readings_global = self.readings_global[:2,:].T
			# readings_global2 = readings_global2[:2,:].T
			# print(self.readings_global)
			# print(readings_global2)

			# # Dont want to update these here!! we are using in controller and if the values are updated mid calculation for velocity itll cause many issues
			# # This is updating faster then the velocity controller calculations
			# # self.xa = readings_global[:,0].T
			# # self.ya = readings_global[:,1].T

	


	def main(self):
	
		
		
		while not rospy.is_shutdown():
			
			
			
			
			# Get current state of this follower 
			#telem = get_telemetry(frame_id='map')
			
			# logging/debugging
			#xf.append(telem.x)
			#yf.append(telem.y)
			#evx.append(self.u-telem.vx)
			#evy.append(self.v-telem.vy)
			#eyaw.append(self.omega-telem.yaw)
			#YawC.append(self.omega*(180/math.pi))
			#YawF.append(telem.yaw*(180/math.pi))
			#xdispf.append(self.d_xi)
			#ydispf.append(self.d_yi)
			#velfx.append(telem.vx)
			#velfy.append(telem.vy)
			#velcx.append(self.u)
			#velcy.append(self.v)
			# U_infx.append(self.U_inf)
			# V_infy.append(self.V_inf)
			
			rospy.spin()
			
			

if __name__ == '__main__':
	try:
		# Define the performance parameters here which starts the script
		q=clover(FLIGHT_ALTITUDE = 1.0, RATE = 50, RADIUS = 2.0, CYCLE_S = 13, REF_FRAME = 'map')
		
		q.main()
		#print(xa)
		#print(xf)
		
		# Plot logged data for analyses and debugging
		plt.figure(1)
		plt.subplot(211)
		plt.plot(xf,yf,'r',label='x-fol')
		#plt.plot(xa,'b--',label='x-obs')
		plt.fill(xa[0],ya[0],'k') # plot first reading
		plt.legend()
		plt.grid(True)
		#plt.subplot(312)
		#plt.plot(yf,'r',label='y-fol')
		#plt.plot(ya,'b--',label='y-obs')
		#plt.legend()
		#plt.grid(True)
		#plt.ylabel('Position [m]')
		plt.subplot(212)
		plt.plot(YawF,'b',label='yaw-F')
		plt.plot(YawC,'g',label='yaw-C')
		plt.legend()
		plt.grid(True)
		plt.ylabel('yaw [deg]')
		plt.xlabel('Time [s]')
		
		# Velocity plot
		plt.figure(2)
		plt.subplot(311)
		plt.plot(velfx,'r',label='vx-vel')
		plt.plot(velcx,'b',label='vx-com')
		plt.ylabel('vel[m/s]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		plt.subplot(312)
		plt.plot(velfy,'r',label='vy-vel')
		plt.plot(velcy,'b--',label='vy-com')
		plt.legend()
		plt.grid(True)
		plt.ylabel('Position [m]')
		plt.subplot(313)
		plt.plot(evx,'r',label='evx')
		plt.plot(evy,'b',label='evy')
		plt.plot(eyaw,'g',label='eyaw')
		plt.ylabel('Error[m]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)

		plt.figure(3)
		for x_row, y_row in zip(xa, ya):
			plt.plot(x_row,y_row, '-o',label=f'Reading {len(plt.gca().lines)}')
			#plt.fill(xa,ya,'k')
		plt.grid(True)
		plt.legend()
		
		# plt.figure(3)
		# plt.plot(U_infx,'r',label='x_inf')
		# plt.plot(V_infy,'b',label='y_inf')
		# #plt.plot(velcx,'g',label='velcx')
		# #plt.plot(velcy,'m',label='velcy')
		# #plt.plot(velLx,'k',label='velLx')
		# #plt.plot(velLy,'y',label='velLy')
		# plt.ylabel('SMC Commands [m/s]')
		# plt.xlabel('Time [s]')
		# plt.legend()
		# plt.grid(True)
		
		#plt.figure(4)
		#plt.subplot(211)
		#plt.plot(ex,'r',label='ex-fol')
		#plt.plot(ey,'b--',label='ey-lead')
		#plt.plot(
		
		#plt.subplot(212)
		#plt.plot(YawL,'b--',label='yaw')
		
		#plt.show()
		
	except rospy.ROSInterruptException:
		pass
