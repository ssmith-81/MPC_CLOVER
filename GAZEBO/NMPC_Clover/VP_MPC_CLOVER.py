import rospy
import json
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from CLOVER_MODEL import export_clover_model
from FUNCTIONS import CloverCost, acados_settings
import numpy as np
import scipy.linalg
from casadi import SX, vertcat, sin, cos, norm_2, diag, sqrt 
import matplotlib.pyplot as plt

# Global lof variables
X = []
VX = []
Y = []
Ux = []
Uy = []
Uz = []
T = []
Z = []
VZ = []



# This class categorizes all of the functions used for complex rajectory tracking
class clover:

    def __init__(self, N_horizon, T_horizon, Time): 
        # State = [x, y, z, q_0, q_1, q_2, q_3 , x_dot, y_dot, z_dot, p_dot, q_dot, r_dot] p-phi, q-theta, r-psi
        self.X0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Initialize the states [x,xdot,y,ydot,z,zdot]
        self.N_horizon = N_horizon # Define prediction horizone in terms of optimization intervals
        self.T_horizon = T_horizon # Define the prediction horizon in terms of time (s) --> Limits time and improves efficiency
        # I think we will have to set this to 1/50 when using for position controller in PX4? Because we need the overall
        # constroller operating at 50 Hz
        self.Time = Time  # maximum simulation time[s]

        # Clover physical Characteristics
        self.mass = 1 #1.55 # kg
        Ixx = 1.72345e-2 # 0.03
        Iyy = 1.72345e-2 #0.03
        Izz = 3.0625e-2 # 0.06
        self.J = np.array([Ixx,Iyy,Izz])



    def main(self):

        # load model
        acados_solver, acados_integrator, model = acados_settings(self.N_horizon, self.T_horizon, self.mass, self.J)


        # prepare simulation Matlab t = 0:1/50:10 -> length = 1:501
        
        # dimensions
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        Nsim = int(self.Time * self.N_horizon / self.T_horizon)

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

        # # initialize solver
        # for stage in range(N_horizon + 1):
        #     acados_solver.set(stage, "x", 0.0 * np.ones(x_current.shape))
        # for stage in range(N_horizon):
        #     acados_solver.set(stage, "u", np.zeros((nu,)))

        acados_solver.set(0, "lbx", self.X0) # initialize
        acados_solver.set(0, "ubx", self.X0)

        # Closed loop simulation

        for i in range(Nsim):

            # # set initial state constraint
            # acados_solver.set(0, 'lbx', x_current) # The 0 represents the first shooting node, or initial position
            # acados_solver.set(0, 'ubx', x_current)

            # update reference
            for j in range(self.N_horizon):
               # State = [x, y, z, q_0, q_1, q_2, q_3 , x_dot, y_dot, z_dot, p_dot, q_dot, r_dot]
                #yref=np.array([1,0,1,0,1,0,0,0,0]) # Set a constant reference of 1 for each position for now
                yref=np.array([1.0, 1.0, 1.0, 0, 0, 0, 0.0, 0.5, 0.5, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Constant position
                acados_solver.set(j, "yref", yref)
            yref_N = np.array([1.0, 1.0, 1.0, 0, 0, 0, 0.0, 0.5, 0.5, 0, 0.0, 0.0, 0.0]) # Terminal position reference
            #yref_N = np.array([0,0.5,0,0.5,0,0.2]) # terminal velocity constraint
            # yref_N=np.array([0,0,0,0,0,0])
            acados_solver.set(self.N_horizon, "yref", yref_N)

            # Solve ocp
            status = acados_solver.solve()
            # timings[i] = acados_solver.get_status("time_tot")

            if status not in [0, 2]:
                acados_solver.print_statistics()
                raise Exception(
                    f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {x0}"
                )

            if status == 2:
                print(
                    f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {x0}"
                )

            # if status != 0:
            #     raise Exception('acados acados_ocp_solver returned status {} in time step {}. Exiting.'.format(status, i))

            # get solution
            x0 = acados_solver.get(0, "x")
            u0 = acados_solver.get(0, "u")
        
            print("control at time", i, ":", simU[i,:])
            for j in range(nx):
                simX[i, j] = x0[j]
            for j in range(nu):
                simU[i, j] = u0[j]
            # simU[i,:] = acados_solver.get(0, "u")
            

            # simulate system
            # acados_integrator.set("x", x_current)
            # acados_integrator.set("u", simU[i,:])

            # status = acados_integrator.solve()
            # if status != 0:
            #     raise Exception(
            #         f"acados integrator returned status {status} in closed loop instance {i}"
            #     )

            # update initial condition
            x0 = acados_solver.get(1, "x")
            acados_solver.set(0, "lbx", x0) # Update the zero shooting node position
            acados_solver.set(0, "ubx", x0)
            
			# logging/debugging
            X.append(simX[i,0])
            VX.append(simX[i,7])
            Y.append(simX[i,1])
            Ux.append(simU[i,0])
            Uy.append(simU[i,1])
            Uz.append(simU[i,2])
            T.append(simU[i,3])
            Z.append(simX[i,2])
            VZ.append(simX[i,9])

            # update state
            # x_current = acados_integrator.get("x")
            # simX[i+1,:] = x_current

if __name__ == '__main__':
    try:
		# Define the performance parameters here which starts the script
        N_horizon = 10
        T_horizon = 1.0/20
        Time = 10.00
        q=clover(N_horizon = N_horizon, T_horizon = T_horizon, Time = Time)
		
        q.main()
		#print(xa)
		#print(xf)
        
          
        # Plot logged data for analyses and debugging
        Nsim = int(Time * N_horizon / T_horizon)
        t = np.linspace(0.0, Nsim * T_horizon / N_horizon, Nsim)
        plt.figure(1)
        plt.subplot(311)
        plt.plot(t, X,'r')
        #plt.legend()
        plt.grid(True)
		#plt.subplot(312)
		#plt.plot(yf,'r',label='y-fol')
		#plt.plot(ya,'b--',label='y-obs')
		#plt.legend()
		#plt.grid(True)
        plt.ylabel('X Position [m]')
        plt.subplot(312)
        plt.plot(t, Y,'r')
        plt.ylabel('Y Position [m]')
        plt.grid(True)
        plt.subplot(313)
        plt.plot(t, VX,'r')
        plt.ylabel('VX [m/s]')
        plt.grid(True)

        plt.figure(2)
        plt.subplot(311)
        plt.plot(t, Ux,'r')
        plt.legend()
        plt.grid(True)
        plt.ylabel('T_input')
        plt.xlabel('Time [s]')
        plt.subplot(312)
        plt.plot(t, Z,'r')
        plt.ylabel('Z Position [m]')
        plt.grid(True)
        plt.subplot(313)
        plt.plot(t, VZ,'r')
        plt.ylabel('VZ [m/s]')
        plt.grid(True)

        plt.figure(3)
        plt.plot(X,Y,'b')
        plt.ylabel('Y [m]')
        plt.xlabel('X [m]')
        plt.grid(True)
        plt.show()

        plt.figure(4)
        plt.subplot(311)
        plt.plot(t, Ux,'r')
        plt.legend()
        plt.grid(True)
        plt.ylabel('Ux')
        plt.subplot(312)
        plt.plot(t, Uy,'b')
        plt.legend()
        plt.grid(True)
        plt.ylabel('Uy')
        plt.subplot(313)
        plt.plot(t, T/41,'r')
        plt.legend()
        plt.grid(True)
        plt.ylabel('T_input')
        plt.xlabel('Time [s]')
        plt.show()
	
		
    except rospy.ROSInterruptException:
        pass
