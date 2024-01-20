import acados
from casadi import vertcat, sum1, mtimes, Function




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
	
	# Remember from matlab,
	X_dec = x[1:N+1, [0, 2, 4]] # Extract position [x, y, z] from 1->N shooting nodes
	U_dec1 = u[0:N, :] #--> goes from positions 0->N-1 shooting nodes

	U_dec2 = u[1:N, :] # Goes from 1->N-1
	U_dec2 = vertcat([0, 0, 0], U_dec2)

	# Error variables
	E_X = ref[1:N+1, :] - X_dec.T # Columns = outputs, rows = prediction horizon i.e. shooting nodes
	Delta_u = U_dec2 - U_dec1 # Change in u per step in prediction horizon, for smoother control input

	# Collision avoidance potential function
	r_des = 2.2

	# Avoidance via distance from center of circle
	X_crit = x_center*np.ones((N,1)) # Size Nx1 array
	Y_crit = y_center*np.ones((N,1))
	XY_crit = vertcat(X_crit, Y_crit)

	R_des = r_des*np.ones((N,1))

	E_Xc = XY_crit - X_dec[:, 0:2] # Uncludes column 0 and up to, but not including 
									# = X_dec[:, [0, 1]]
	mag = sum1(E_Xc**2, axis=1) # sum1 from casadi
	d_mag = R_des**2 - mag

	# potential function parameters:
	lambda_val = 1500
	k = 5
	Joa = lambda_val/(1 + exp(-k*d_mag))

	# Compute the cost function
	J = 0

	for i in range(N):
		Je = mtimes([E_X[:,i].T, Q, E_X[:,i]]) # Tracking error cost component
		Ju = mtimes([U_dec1[:,i].T, R, U_dec1[:, 1]]) # Control input cost component
		Jdu = mtimes([Delta_u[:,i].T, R_du, Delta_u[:,i]]) # Smoothness of control input cost
		Jep = 0.0001*e # Slack variable to alleviate strickness

		J1 = Je + Ju + Jdu # Joa[i] # Include the relevant contributions
		J += J1

	# Optionally, include soft constraints penelty with slack (e)
	#J += 0.0001*e


	



	# Create a CasADi Function for the cost expression
	J_function = Function('CloverCost', [x,u], [J])

	return J_function
	
	
	
