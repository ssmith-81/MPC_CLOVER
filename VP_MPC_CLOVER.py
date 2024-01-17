import json
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from CLOVER_MODEL import export_clover_model
import numpy as np
import scipy.linalg


X0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Initialize the states [x,xdot,y,ydot,z,zdot]
N_horizon = 10 # Define prediction horizone in terms of optimization intervals
T_horizon = 1.0/50 # Define the prediction horizon in terms of time (s) --> Limits time and improves efficiency
# I think we will have to set this to 1/50 when using for position controller in PX4? Because we need the overall
# constroller operating at 50 Hz


def create_ocp_solver_description():

   # optimal control problem
    # https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp.AcadosOcp
    ocp = AcadosOcp()

    # https://docs.acados.org/python_interface/index.html#acados_template.acados_model.AcadosModel
    model = export_clover_model()
    ocp.model = model
    x = model.x
    u = model.u
    nx = model.x.size()[0]  # number of states.
    nu = model.u.size()[0]  # number of inputs.


    # https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp.AcadosOcpDims
    ocp.dims.N = N_horizon # prediction horizon, other fields in dim are set automatically


    # Set the Cost (built in cost function):
    # ocp.cost.cost_type = "NONLINEAR_LS"  # LINEAR_LS  --> Cost type at intermediate shootig nodes (1 to N-1): Default LINEAR_LS
    # ocp.cost.cost_type_0 = "NONLINEAR_LS" # Cost type at initial shoting node (0): would be the same as intermediate shooting nodes if not set explicitly
    # # Therefore did not need to set the type_0 if it was the same
    # ocp.cost.cost_type_e = "NONLINEAR_LS" # Cost type at terminal shooting node (N): Default LINEAR_LS


    # Set the cost (Custom cost function):
     # https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp.AcadosOcpCost
    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.cost_expr_ext_cost = CloverCost(x,u,refTrajectory)
    ocp.cost_expr_ext_cost_e = 

    # https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp.AcadosOcpConstraints
    # constraints u = [ux,uy,uz] -> acceleration commands
    u_lb = np.array([-10, -10, -10])
    u_ub = np.array([10, 10, 10])

    ocp.constraints.constr_type = 'BGH'  # BGP is for convex over nonlinear.
    ocp.constraints.lbu = u_lb
    ocp.constraints.ubu = u_ub
    # ocp.constraints.idxbu = np.array([0, 1, 2]) # Constraints apply to u[0],u[1],u[2]
    # Nonlinear in equality constraints
    # ocp.constraints.lh = h_lb
    # ocp.constraints.uh = h_ub
    # ocp.constraints.lh_e = h_lb
    # ocp.constraints.uh_e = h_ub

    # Solver options
     # https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp_options.AcadosOcpOptions
   
    # solver_options.Tsim is time horizon for one integrator step. Automatically as tf/N. Deafault: None
    ocp.solver_options.tf = T_horizon
    ocp.solver_options.integrator_type = 'ERK' # ERK explicit numerical integration based on Runge-Kutta scheme, suitable for simple dynamics and fast sampling times
    # IRK implicit numerical integration based on runge-kutta scheme, suitable for complex dynamics
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    # SQP - sequential quadratic programming method, simple linear problems
    # SQP_RTI extension of SQP to real time applications, aims to reduce computational time
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES


    return ocp


def generate_ref():

    # Using just the straight line position trajectory

    t = np.arrange(0,10, T_horizon) # This stuff defined
    initial_point = np.array([0.5,0.5])
    final_point = np.array([10,10])
    x_values = np.interp(t,[0, 10], [initial_point[0], final_point[0]])
    y_values = np.interp(t, [0, 10], [initial_point[1], final_point[1]])
    z_values = np.ones_like(t)

    # todo: Using the VP velocity field...

    # todo: fgure out if T_horizon is the actualy total sampling time of controller


def main():

    # Create solvers
    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")
    acados_integrator = AcadosSimSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")

    # prepare simulation Matlab t = 0:1/50:10 -> length = 1:501
    


    # todo: set up a class so we can reference values from other functions
    Nsim = len(t) # number of steps in the simulation
    nx = ocp.model.x.size()[0] # numer of states
    nu = ocp.model.u.size()[0] # Number of control inputs

    # Arrays to store the results (chatgpt)
    simX = np.ndarray((Nsim, nx))  # xHistory   or np.ndarray((Nsim+1,nx))??
    simU = np.ndarray((Nsim - 1, nu))     # uHistory  or np.ndarray((Nsim, nu))??

