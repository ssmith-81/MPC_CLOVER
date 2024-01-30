#!/usr/bin/python

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from CLOVER_MODEL import export_clover_model
from FUNCTIONS import CloverCost, acados_settings
from casadi import SX, vertcat, sin, cos, norm_2, diag, sqrt 
from utils.utils import quaternion_to_euler, euler_to_quaternion

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Quaternion
import tf

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

#from clover import srv
from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('Helix')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Release service to allow for complex trajectory publishing i.e stopping navigate service publishing because you dont want two sources of publishing at the same time.
release = rospy.ServiceProxy('simple_offboard/release', Trigger)

PI_2 = math.pi/2

# Global lof variables
X = []
VX = []
Y = []
UX = []
UZ = []
Z = []
VZ = []

class fcuModes:

	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException as e:
			print ("Service arming call failed: %s")

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print ("service set_mode call failed: %s. Offboard Mode could not be set.")

class clover:

	def __init__(self, FLIGHT_ALTITUDE = 1.0, RATE = 50, RADIUS = 2, CYCLE_S = 27, N_horizon=8, T_horizon=1/2): # rate = 50hz radius = 5m cycle_s = 25
        
 		
 		# Publisher which will publish to the topic '/mavros/setpoint_velocity/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)

		 # global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.RADIUS          = RADIUS                   # radius of figure 8 in meters
		self.CYCLE_S         = CYCLE_S                  # time to complete one figure 8 cycle in seconds
		self.STEPS           = int( self.CYCLE_S * self.RATE )

		# State = [x, y, z, q_0, q_1, q_2, q_3 , x_dot, y_dot, z_dot, p_dot, q_dot, r_dot] p-phi, q-theta, r-psi
		self.X0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Initialize the states [x,xdot,y,ydot,z,zdot]
		
		# MPC variables
		self.N_horizon = N_horizon # Define prediction horizone in terms of optimization intervals
		self.T_horizon = T_horizon # Define the prediction horizon in terms of time (s) --> Limits time and improves efficiency

		# Clover physical Characteristics
		self.mass = 1.55 #1.55 # kg
		Ixx = 0.02#1.72345e-2 # 0.03
		Iyy = 0.02#1.72345e-2 #0.03
		Izz = 0.032#3.0625e-2 # 0.06
		self.J = np.array([Ixx,Iyy,Izz])

		
		# Publisher which will publish to the topic '/mavros/setpoint_raw/local'.
		self.publisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
		
		# Subscribe to drone state
		self.state = rospy.Subscriber('mavros/state', State, self.updateState)

		self.current_state = State()
		self.rate = rospy.Rate(20)

	def updateState(self, msg):
		self.current_state = msg

	def main(self):#,x=0, y=0, z=2, yaw = float('nan'), speed=1, frame_id ='',auto_arm = True,tolerance = 0.2):
		i    = 0
		dt   = 1.0/self.RATE
		dadt = math.pi*2 / self.CYCLE_S # first derivative of angle with respect to time
		r    = self.RADIUS
		path = []
		
		 # load model
		acados_solver, acados_integrator, model = acados_settings(self.N_horizon, self.T_horizon, self.mass, self.J)
		# dimensions
		nx = model.x.size()[0]
		nu = model.u.size()[0]
		ny = nx + nu
		

		# Wait for 5 seconds
		rospy.sleep(3)
		
		# Takeoff with Clovers navigate function
		navigate(x=0, y=0, z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = 'body', auto_arm = True)
		tolerance =0.2
		
		# Base the flight on time and not performance so I can plot against one another in matlab (want aligned time) --> Only needed for non wind case because I include takeoff in that analysis
		# Base takeoff on performance when including wind because t aeks a while to find the starting point and reject wind
		#while not rospy.is_shutdown():
		#	telem = get_telemetry(frame_id = 'navigate_target')
		#	if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
		#		break
		#	rospy.sleep(0.2)

		# Wait for 11 seconds
		rospy.sleep(11)

		#k = get_telemetry()
		#print(k.mode)
		
		rospy.loginfo('start figure8')
		PI=math.pi
		start = get_telemetry()
		start_stamp = rospy.get_rostime()
		
		# create random time array
		t = np.arange(0,self.STEPS,1)
		
		posx = [1]*len(t)
		posy = [1]*len(t)
		posz = [1]*len(t)
		velx = [1]*len(t)
		vely = [1]*len(t)
		velz = [1]*len(t)
		afx = [1]*len(t)
		afy = [1]*len(t)
		afz = [1]*len(t)
		yawc = [1]*len(t)
		pitchc = [1]*len(t)
		rollc = [1]*len(t)
		yaw_ratec = [1]*len(t)
		
		for i in range(0, self.STEPS):
		
			# calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
			# through the curve
			a = (-math.pi/2) + i*(math.pi*2/self.STEPS)
			c = math.cos(a)
			c2a = math.cos(2.0*a)
			c4a = math.cos(4.0*a)
			c2am3 = c2a-3.0
			c2am3_cubed = c2am3*c2am3*c2am3
			s = math.sin(a)
			cc = c*c
			ss = s*s
			sspo = (s*s)+1.0 # sin squared plus one
			ssmo = (s*s)-1.0 # sin squared minus one
			sspos = sspo*sspo
			
			 # Position
			# https:#www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
			posx[i] = -(r*c*s) / sspo
			# https:#www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
			posy[i] =  (r*c)   / sspo
			posz[i] =  self.FLIGHT_ALTITUDE

			# Velocity
			# https:#www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			velx[i] =   dadt*r* ( ss*ss + ss + (ssmo*cc) ) / sspos
			# https:#www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			vely[i] =  -dadt*r* s*( ss + 2.0*cc + 1.0 )    / sspos
			velz[i] =  0.0

			# Acceleration
			# https:#www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			afx[i] =  -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/ c2am3_cubed
			# https:#www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			afy[i] =  dadt*dadt*r*c*((44.0*c2a) + c4a - 21.0) / c2am3_cubed
			afz[i] =  0.0

			# calculate yaw as direction of velocity
			yawc[i] = math.atan2(vely[i], velx[i])
			
			# calculate Pitch and Roll angles, if publishing acceleration isnt possible could send these low level commands
			pitchc[i] = math.asin(afx[i]/9.81)
			rollc[i] = math.atan2(afy[i], afz[i])
		
		# calculate yaw_rate by dirty differentiating yaw
		for i in range(0, self.STEPS):
			next = yawc[(i+1)%self.STEPS] # 401%400 = 1 --> used for this reason (when it goes over place 400 or whatever STEPS is)
			curr = yawc[i]
      			# account for wrap around +- PI
			if((next-curr) < -math.pi):
				next = next + math.pi*2
			if((next-curr) >  math.pi):
				next = next - math.pi*2
			yaw_ratec[i] = (next-curr)/dt
		
		# Define 
		target = AttitudeTarget()
		rr = rospy.Rate(self.RATE)
		k=0
		release() # stop navigate service from publishing

		# update initial condition (current pose of MPC problem)
		# State = [x, y, z, q_0, q_1, q_2, q_3 , x_dot, y_dot, z_dot, p_dot, q_dot, r_dot]
		telem = get_telemetry(frame_id = 'map')
		q_cur = euler_to_quaternion(telem.roll, telem.pitch, telem.yaw) # [qx, qy, qz, qw]
		self.x0 = np.array([telem.x, telem.y, telem.z, q_cur[0], q_cur[1], q_cur[2], q_cur[3], telem.vx, telem.vy, telem.vz, telem.roll_rate, telem.pitch_rate, telem.yaw_rate])
		acados_solver.set(0, "lbx", self.x0) # Update the zero shooting node position
		acados_solver.set(0, "ubx", self.x0) # update the zero shooting node control input

		while not rospy.is_shutdown():
		
			
			target.type_mask = 128 # ignore attitude
			#target.type_mask =  3576 # Use only position #POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |POSITION_TARGET_TYPEMASK_AZ_IGNORE | POSITION_TARGET_TYPEMASK_FORCE_IGNORE | POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE # = 3576
			#target.type_mask =  3520 # Use position and velocity
			#target.type_mask =  3072 # Use position, velocity, and acceleration
			#target.type_mask =  2048 # Use position, velocity, acceleration, and yaw
			
			# update reference
			for j in range(self.N_horizon):
				index = k + j
				# Check if index is within the bounds of the arrays
				if index < len(posx):
					# Convert euler setpoints to quaternion
					q_set = euler_to_quaternion(rollc[index], pitchc[index], yawc[index])
					yref = np.array([posx[index]+0.15, posy[index], posz[index], q_set[0], q_set[1], q_set[2], q_set[3], velx[index], vely[index], velz[index], 0, 0, yaw_ratec[index], 0, 0, 0,0])

				else:
					# Convert euler setpoints to quaternion
					q_set = euler_to_quaternion(rollc[-1], pitchc[-1], yawc[-1])
					# If index exceeds the length of the arrays, use the last elements
					yref = np.array([posx[-1]+0.15, posy[-1], posz[-1], q_set[0], q_set[1], q_set[2], q_set[3], velx[-1], vely[-1], velz[-1], 0, 0, yaw_ratec[-1], 0, 0, 0, 0])

    
				acados_solver.set(j, "yref", yref)

			index2 = k + self.N_horizon
			if index2 < len(posx):

				# Convert euler setpoints to quaternion
				q_set = euler_to_quaternion(rollc[k+self.N_horizon], pitchc[k+self.N_horizon], yawc[k+self.N_horizon])
				# Terminal components
				yref_N = np.array([posx[k+self.N_horizon]+0.15, posy[k+self.N_horizon], posz[k+self.N_horizon], q_set[0], q_set[1], q_set[2], q_set[3], velx[k+self.N_horizon], vely[k+self.N_horizon], velz[k+self.N_horizon], 0, 0, yaw_ratec[k+self.N_horizon]])
				
			else:
				# Convert euler setpoints to quaternion
				q_set = euler_to_quaternion(rollc[-1], pitchc[-1], yawc[-1])
				# Terminal components
				yref_N = np.array([posx[-1]+0.15, posy[-1], posz[-1], q_set[0], q_set[1], q_set[2], q_set[3], velx[-1], vely[-1], velz[-1], 0, 0, yaw_ratec[-1]])
				

			acados_solver.set(self.N_horizon, "yref", yref_N)

			# Solve ocp
			status = acados_solver.solve()

			# get solution
			x0 = acados_solver.get(0, "x")
			u0 = acados_solver.get(0, "u")

			# Gather position for publishing
			# target.position.x = posx[k] +0.15
			# target.position.y = posy[k]
			# target.position.z = posz[k]
			
			# Gather velocity for publishing
			# target.velocity.x = velx[k]
			# target.velocity.y = vely[k]
			# target.velocity.z = velz[k]
			
			# Gather acceleration for publishing
			# target.acceleration_or_force.x = afx[k]
			# target.acceleration_or_force.y = afy[k]
			# target.acceleration_or_force.z = afz[k]

			# Feedforward acceleration setpoints as well
			# target.acceleration_or_force.x = u0[0] + afx[k]
			# target.acceleration_or_force.y = u0[1] + afy[k]
			# target.acceleration_or_force.z = u0[2] + afz[k]
			
			# Gather angular rates for publishing
			target.body_rate.x = u0[0]
			target.body_rate.y = u0[1]
			target.body_rate.z = u0[2]
			
			# Gather thrust input for publishing
			target.thrust = u0[3]/15.2 # normalize thrust input
			
			
			self.publisher.publish(target)

			# update initial condition
			#x0 = acados_solver.get(1, "x")
			# update initial condition (current pose of MPC problem)
			# State = [x, y, z, q_0, q_1, q_2, q_3 , x_dot, y_dot, z_dot, p_dot, q_dot, r_dot]
			telem = get_telemetry(frame_id = 'map')
			q_cur = euler_to_quaternion(telem.roll, telem.pitch, telem.yaw) # [qx, qy, qz, qw]
			self.x0 = np.array([telem.x, telem.y, telem.z, q_cur[0], q_cur[1], q_cur[2], q_cur[3], telem.vx, telem.vy, telem.vz, telem.roll_rate, telem.pitch_rate, telem.yaw_rate])
			acados_solver.set(0, "lbx", self.x0) # Update the zero shooting node position
			acados_solver.set(0, "ubx", self.x0) # update the zero shooting node control input

			# logging/debugging
			X.append(telem.x)
			VX.append(telem.vx)
			Y.append(telem.y)
			UX.append(u0[0])
			UZ.append(u0[3]/41)
			Z.append(telem.z)
			VZ.append(telem.vz)
			
			
		
			#set_position(x=posx[k], y=posy[k], z=posz[k],frame_id='aruco_map')
			#set_velocity(vx=velx[k], vy=vely[k], vz=velz[k],frame_id='aruco_map')#,yaw = yawc[i]) 
			#set_attitude(yaw = yawc[k],pitch = pitchc[k], roll = rollc[k], thrust = float('nan'),frame_id='aruco_map')
			#set_rates(yaw_rate = yaw_ratec[k],thrust = float('nan'))
		
		
			
			k = k+1
			if k >= self.STEPS: 
				navigate(x=0, y=0, z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = 'aruco_map')
				rospy.sleep(5)
				break
			rr.sleep()

		# Wait for 5 seconds
		rospy.sleep(2)
		# Perform landing
		
		res = land()
		 
		if res.success:
			print('Drone is landing')

	# If we press control + C, the node will stop.
		rospy.sleep(6)
		# debug section
		plt.figure(1)
		#plt.plot(t,velx)
		plt.plot(t,posx)
		plt.plot(X)
		#plt.plot(t,afx)
		#plt.plot(t,yawc)
		#plt.plot(t,yaw_ratec)
		#plt.show()

		plt.figure(2)
		plt.subplot(311)
		plt.plot(UX,'r')
		plt.legend()
		plt.grid(True)
		plt.ylabel('yaw [deg]')
		plt.xlabel('Time [s]')
		plt.subplot(312)
		plt.plot(UZ,'r')
		plt.grid(True)
		plt.subplot(313)
		plt.plot(VZ,'r')
		plt.grid(True)
		plt.show()
		
		rospy.spin()

if __name__ == '__main__':
	try:
		#setArm()
		q=clover()
		q.main()#x=0,y=0,z=1,frame_id='aruco_map')
		
	except rospy.ROSInterruptException:
		pass

