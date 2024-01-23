import rospy
import json
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from CLOVER_MODEL import export_clover_model
from FUNCTIONS import CloverCost, acados_settings
import numpy as np
import scipy.linalg
from casadi import SX, vertcat, sin, cos, norm_2, diag, sqrt 



# This class categorizes all of the functions used for complex rajectory tracking
class clover:

    def __init__(self, N_horizon, T_horizon, Time): 
        self.X0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Initialize the states [x,xdot,y,ydot,z,zdot]
        self.N_horizon = N_horizon # Define prediction horizone in terms of optimization intervals
        self.T_horizon = T_horizon # Define the prediction horizon in terms of time (s) --> Limits time and improves efficiency
        # I think we will have to set this to 1/50 when using for position controller in PX4? Because we need the overall
        # constroller operating at 50 Hz
        self.Time = Time  # maximum simulation time[s]



    def main(self):

        # load model
        acados_solver, acados_integrator, model = acados_settings(self.N_horizon, self.T_horizon)


        # prepare simulation Matlab t = 0:1/50:10 -> length = 1:501
        
        # dimensions
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        Nsim = int(self.T_horizon * self.N_horizon / self.Time)

        # Arrays to store the results (chatgpt)
        # simX = np.ndarray((Nsim, nx))  # xHistory   or np.ndarray((Nsim+1,nx))??
        # simU = np.ndarray((Nsim - 1, nu))     # uHistory  or np.ndarray((Nsim, nu))??
        simX = np.ndarray((Nsim + 1, nx))  # xHistory   or np.ndarray((Nsim+1,nx))??
        simU = np.ndarray((Nsim, nu))     # uHistory  or np.ndarray((Nsim, nu))??

        # initialize data structs
        # simX = np.ndarray((Nsim, nx))
        # simU = np.ndarray((Nsim, nu))

        timings = np.zeros((Nsim,))

        x_current = self.X0
        simX[0,:] = x_current # Set the initial state

        # Set the reference trajectory for the cost function
        #self.ocp = self.generate_ref(self.N_sim)

        # Closed loop simulation

        for i in range(Nsim):

            # set initial state constraint
            acados_solver(0, 'lbx', x_current)
            acados_solver(0, 'ubx', x_current)

            # update reference
            for j in range(self.N_horizon):
               # yref = np.array([s0 + (sref - s0) * j / N, 0, 0, 0, 0, 0, 0, 0])
                yref=np.array([1,0,1,0,1,0,0,0,0]) # Set a constant reference of 1 for each position for now
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([1,0,1,0,1,0])
            # yref_N=np.array([0,0,0,0,0,0])
            acados_solver.set(self.N_horizon, "yref", yref_N)

            status = acados_solver.solve()
            timings[i] = acados_solver.get_status("time_tot")

            if status != 0:
                raise Exception('acados acados_ocp_solver returned status {} in time step {}. Exiting.'.format(status, i))

            simU[i,:] = acados_solver.get(0, "u")
            print("control at time", i, ":", simU[i,:])

            # simulate system
            acados_integrator.set("x", x_current)
            acados_integrator.set("u", simU[i,:])

            status = acados_integrator.solve()
            if status != 0:
                raise Exception('acados integrator returned status {}. Exiting.'.format(status))

            # update state
            x_current = acados_integrator.get("x")
            simX[i+1,:] = x_current

if __name__ == '__main__':
	try:
		# Define the performance parameters here which starts the script
		q=clover(N_horizon = 10, T_horizon = 1.0/50, Time = 10.00)
		
		q.main()
		#print(xa)
		#print(xf)
	
		
	except rospy.ROSInterruptException:
		pass
