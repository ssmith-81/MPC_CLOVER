import acados
import casadi as ca




def CloverCost(ocp):

	# Extract Parameters
	x = ocp.x  # States are N*n in dimensions where N are the shooting nodes and n are the states
	u = ocp.u
	N = ocp.dims.N  # Prediction horizon i.e. number of shooting nodes
	
	
	# State and input weight matrices
	
	Q = vertcat(10,10,10) # weight matrix for the tracking error
	R = vertcat(R_ele, R_ele, R_ele)  # Weight matrix for control input
	R_du = vertcat(0.0001, 0.0001, 0.0001) # Weight matrix for input smoothness
	
	# Extract the current state (not a decision variable) x,y positions
	X_cur = x[0, [0,2]]
	
	# TODO: fix array position definitions (refer to chatgpt for array positions)
	# Extract the decision variables
	X_dec = x[1:N+1, [0, 2, 4]] # Extract position [x, y, z]
	U_dec1 = u[0:N-1, :] #--> Fix: goes from positions 0->N-1

	U_dec2 = u[1:N, :] # Goes from 1->N
	U_dec2 = vertcat([0, 0, 0], U_dec2)

	# Error variables
	E_X = ref[1:N, :] - X_dec.T # Columns = outputs, rows = prediction horizon i.e. shooting nodes
	Delta_u = U_dec2 - U_dec1 # Change in u per step in prediction horizon, for smoother control input

	# Collision avoidance potential function
	r_des = 2.2

	# Avoidance via distance from center of circle
	X_crit = x_center*np.ones((N,1))
	Y_crit = y_center*np.ones((N,1))

	R_des = r_des*np.ones((N,1))

	E_Xc = XY_crit - X_dec[:, 0:2] # Uncludes column 0 and up to, but not including 2



	# Create a CasADi Function for the cost expression
	J = ca.Function('CloverCost', [x,u], [cost_expr])

	return J
	
	
	
