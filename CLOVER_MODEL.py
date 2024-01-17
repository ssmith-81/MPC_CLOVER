from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, norm_2

import numpy as np

def export_clover_model(n_mass, m, D, L):

	model_name = 'clover_model'
	
	 # set up states & controls
	x = SX.sym('x')
	vx = SX.sym('vx')
	y = SX.sym('y')
	vy = SX.sym('vy')
	z = SX.sym('z')
	vz = SX.sym('vz')
	
	x = vertcat(x, vx, y, vy, z, vz)
	
	ux = SX.sym('ux')
	uy = SX.sym('uy')
	uz = SX.sym('uz')

	u = vertcat(ux, uy, uz)
	
	# xdot
	ax = SX.sym('ax')
	ay = SX.sym('ay')
	az = SX.sym('az')
	
	xdot = vertcat(vx, ax, vy, ay, vz, az) # Derivative of the state wrt time
	
	# Parameters (CasADi variable descibing parameters of the DAE; Default [].
	p = []
	
	# Explicit dynamics x_dot = f_expl(x,u,p)
	f_expl = vertcat(vx, ux, vy, uy, vz, uz)
	
	# implicit dynamics f_impl(xdot,x,u,z,p) = 0
	f_impl = xdot - f_expl
	
	# define model
	model = AcadosModel()
	
	model.f_impl_expr = f_impl
	model.f_expl_expr = f_expl
	model.x = x
	model.xdot = xdot
	model.p = p
	model.name = model_name
	
	

	return model

