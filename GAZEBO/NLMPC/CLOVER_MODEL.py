from acados_template import AcadosModel
from casadi import SX,MX, vertcat, sin, cos, norm_2, mtimes
from utils.utils import skew_symmetric, quaternion_to_euler, unit_quat, v_dot_q, quaternion_inverse
# Cadadi is used as a front end for Acados, Acados solver is much different and much faster/better than Casadi

import numpy as np

def export_clover_model(mass,J):

	model_name = 'clover_model'
	 # set up states & controls
	p = MX.sym('p',3) # Position
	q = MX.sym('q',4) # Quaternion

	v = MX.sym('v',3) # Velocity
	r = MX.sym('w', 3) # Angular rate
	
	x = vertcat(p, q, v, r) # State = [x, y, z, q_0, q_1, q_2, q_3 , x_dot, y_dot, z_dot, p_dot, q_dot, r_dot] p-phi, q-theta, r-psi
	
	# Control variables
	u1 = MX.sym('u1')
	u2 = MX.sym('u2')
	u3 = MX.sym('u3')
	u4 = MX.sym('u4')

	u = vertcat(u1, u2, u3, u4)
	
	# pdot (position dynamics)
	f_p = v
	
	# Quaternion (attitude dynamics)
	f_q = (1.0/2.0)*mtimes(skew_symmetric(r),q) # check on this operation

	# Velocity dynamics
	a_thrust = vertcat(0,0,u4) / mass
	# f_thrust = u*20 # u*max_thrust
	# a_thrust = vertcat(0,0,f_thrust[0] + f_thrust[1] + f_thrust[2] + f_thrust[3]) / mass
	
	g = vertcat(0,0,9.81)
	f_v = v_dot_q(a_thrust,q) - g

	# Angular rate dynamics ( set it up as angular rate control inputs, because 
	# with ROS1, or MAVROS, we can only send angular rate setpoints and thrust in
	# we can't send torque setpoints...)

	f_r = vertcat(u1 + (J[1] - J[2])*r[1]*r[2]/J[0], u2 + (J[2] - J[0])*r[0]*r[2]/J[1],
	u3 + (J[0] - J[1])*r[0]*r[1]/J[2])

	# Concated dynamics
	f_dyn = vertcat(f_p, f_q, f_v, f_r)
	
	# Parameters (CasADi variable descibing parameters of the DAE; Default [].
	# parameters
	p = vertcat([])

	# For implicit integrators (IRK) you need to provide an implicit ODE
	# for explicit integrators (ERK) you need to provide an explicit ODE
	
	# Explicit dynamics x_dot = f_expl(x,u,p)
	f_expl = f_dyn
	
	# implicit dynamics f_impl(xdot,x,u,z,p) = 0
	# f_impl = xdot - f_expl
	
	# define model
	model = AcadosModel()
	
	# model.f_impl_expr = f_impl
	model.f_expl_expr = f_expl
	model.x = x
	model.u = u
	# model.xdot = xdot
	model.p = p
	model.name = model_name
	
	

	return model
