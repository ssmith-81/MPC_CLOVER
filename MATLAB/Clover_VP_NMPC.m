close all
clear all

% State space matrices:

A = [0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 1; 0 0 0 0 0 0];

B = [0 0 0; 1 0 0; 0 0 0; 0 1 0; 0 0 0; 0 0 1];

%C = eye(6); % Output all states
% D = zeros(6,3); % for all states output
% State matrix: [x vx y vy z vz]^t
C = [1 0 0 0 0 0;...
0 0 1 0 0 0;...
0 0 0 0 1 0]; % output positions x,y,z
D = zeros(3,3);

% C = [1 0 0 0 0 0;...
%  0 0 1 0 0 0;...
%  0 0 0 0 1 0];
%D = zeros(3,3);

% Determine the discrete state space model
Ts = 1/50; % PX4 position control module operating at 50hz
[A_m, B_m, C_m, D_m] = c2dm(A, B, C, D, Ts);

Co = ctrb(A_m,B_m);
rank(Co) % rank = n=6 for 6 states controllable
Mo = obsv(A,C);
rank(Mo)

% Define a nonlinear MPC controller
nx = 6; % Number of states
nu = 3; % Number of inputs
%ny = 6; % Number of outputs
ny = 3; % Number of outputs

%% Augmented Model

% [A_aug,B_aug,C_aug] = augment_mimo(A_m,B_m,C_m,nx,nu,3); % ny=3
% 
% 
% % Check controllability and observability of augmented system:
% Co = ctrb(A_aug,B_aug);
% rank(Co) % rank = n=9 for 6 states and integral states controllable
% Mo = obsv(A_aug,C_aug);
% rank(Mo)

% Create a nonlinear MPC controller with a prediction model that has six
% states, six outputs, and three inputs. The model has three MV signals:
% x-acceleration, y-acceleration, and z-acceleration:
nlobj = nlmpc(nx,ny,'MV',[1 2 3]);

p = 6; % Define the prediction horizon
c = 3; % Define the control horizon

nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;
nlobj.Model.IsContinuousTime = false;

%% Set a cost function in conjunction 

% Note: The same parameters are passed to the prediction model, custom cost 
% function, and custom constraint functions of the controller. For example, 
% if the state function uses only parameter p1, the constraint functions use 
% only parameter p2, and the cost function uses only parameter p3, then all 
% three parameters are passed to all of these functions.

% Define the number of parameters
numberOfParams = 1;

% Set the number of parameters for the controller
nlobj.Model.NumberOfParameters = numberOfParams;

%% PANEL POTENTIAL CONFIGURATION

 n = 60;
 % Set the uniform velocity value
 U_inf = 0.5;
 V_inf = 0.5;

% Source and sink strengths
g_source = 0.5;
g_sink = 2.5;

% Source and Sink Locations
xs = 0; ys = 0; % source
xsi = 10; ysi = 10; % sink
 

%% DEFINE THE OBSTACLE TO AVOID
%Circle instead of airfoil
% Circle 1 parameters
center1 = [5, 5];
radius1 = 2;

% Generate points for Circle 1
L = 0:Ts:10; % Time vector
theta1 = linspace(0, -2*pi, length(L)); % L points from 0 to -2pi (clockwise around circle)
xa = center1(1) + radius1 * cos(theta1);
ya = center1(2) + radius1 * sin(theta1);

%alpha = a*pi/180;
%This function calculates the location of the control points as well as the
%right hand side of the stream function equation:

[xmid,ymid,dx,dy,Sj,phiD,rhs] = CLOVER_COMPONENTS(xa,ya,U_inf, V_inf,g_source,g_sink,xs,ys,xsi,ysi,n);



Q = 0.01; % 0.0001
pass = [Q xa ya center1]; % Pass weight Q, obstacle locations xa,ya, as well as circles obstacle center 
params = {pass};

% Specify the state function:
%nlobj.Model.StateFcn = @(x,u) CloverStateFcn(x,u);
nlobj.Model.StateFcn = @(x,u,params) A_m*x + B_m*u;
% Specify the output function
nlobj.Model.OutputFcn = @(x,u,params) C_m*x; % or @(x,u) [x(1);x(2);x(3);x(4);x(5);x(6)];

% Set the constraints for manipulated variables.

nlobj.MV(1).Min = -10;      % Minimum x-acceleration -5 m/s^2
nlobj.MV(1).Max = 10;       % Maximum x-acceleration 5 m/s^2
nlobj.MV(2).Min = -5;   
nlobj.MV(2).Max = 5;   
nlobj.MV(3).Min = -5;   
nlobj.MV(3).Max = 5;  

% Define the obstacle avoidance constraint function

nlobj.Optimization.CustomIneqConFcn = ...
    @(X,U,e,data,params) CloverConstraint(X,U,e,data,params);



%% Using the standard cost function
 nlobj.Weights.ManipulatedVariables = [0.00001 0.00001 0.00001];
 nlobj.Weights.OutputVariables = [3 3 1]; %3 3 1

% % Set the cost function (example with quadratic cost) 
nlobj.Optimization.CustomCostFcn = @(X,U,e,data,params) CloverCost(X, U, e, data,params);
nlobj.Optimization.ReplaceStandardCost = true; % Setting this to true only uses the 
% custom cost functon, setting it to false will use the custom one in
% addition to the built in standard one from NLMPC


%Option to replace the standard cost function with the custom cost function, specified as one of the following:
%true — The controller uses the custom cost alone as the objective function during optimization. In this case, the Weights property of the controller is ignored.
%false — The controller uses the sum of the standard cost and custom cost as the objective function during optimization.
%If you do not specify a custom cost function using CustomCostFcn, then the controller ignores RepalceStandardCost.



%Validate prediction model functions at an arbitrary operating point using the validateFcns command
x0 = [0.5 0 0.5 0 0 0];
u0 = [0 0 0];
ref0 = [22 0 0];
md0 = 0.1;
validateFcns(nlobj,x0,u0,[],params);%,md0,{},ref0);


% Define desired trajectory (replace with your specific trajectory)
t = 0:Ts:10; % Time vector

% Define initial and final points
initial_point = [0.5, 0.5];
final_point = [10, 10];

% Interpolate x and y values based on the linear equation
x_values = interp1([0, 10], [initial_point(1), final_point(1)], t);
y_values = interp1([0, 10], [initial_point(2), final_point(2)], t);
z_values = ones(length(0:Ts:10),1);

% refTrajectory(:,1) = sin(t);%ones(1,length(t))*5; % Example: Sinusoidal trajectory
% refTrajectory(:,2) = cos(t);%ones(1,length(t))*3; % feedfrward velocity
% refTrajectory(:,3) = sin(t); % Example: Sinusoidal trajectory
% refTrajectory(:,4) = cos(t); % feedfrward velocity
% refTrajectory(:,5) = sin(t); % Example: Sinusoidal trajectory
% refTrajectory(:,6) = cos(t); % feedfrward velocity

% Define reference trajectory for the 3 position outputs
refTrajectory(:,1) = x_values;%ones(1,length(t))*5; % Example: Sinusoidal trajectory
refTrajectory(:,2) = y_values;%ones(1,length(t))*3; % feedfrward velocity
refTrajectory(:,3) = z_values; % Example: Sinusoidal trajectory


% Simulation loop
numSteps = numel(t);
xHistory = zeros(numSteps, nx); % Initialize state history
uHistory = zeros(numSteps-1, nu); % Initialize control input history

% initialize control input
uk = zeros(3,1);

% Create a default nlmpcmoveopt option set.
options = nlmpcmoveopt;
options.Parameters = params; % pass parameter list with the move object

for k = 1:numSteps
    % a. State Initialization
    if k == 1
        xk = x0'; % Initial state
    else
        xk = xNext; % Update state from previous iteration
    end

    % b. Controller Update
    [uk, info] = nlmpcmove(nlobj, xk, uk, refTrajectory(k,:),[],options);

    % c. System Simulation
     % Model
%     xh(:,k+1) = A*xh(:,k) + B*uk;
%     yh(:,k) = C*xh(:,k+1);% + dist(:,i);
%     Xf = xh(:,i+1);

    %xNext = your_system_model(xk, uk); % Replace with your system model

    xNext = A_m*xk + B_m*uk;

    % d. Data Logging
    xHistory(k, :) = xk;
    if k < numSteps
        uHistory(k, :) = uk;
    end

    % e. Iterate
end

% Plotting
% Plot the results (system states, control inputs, and desired trajectory)
% Customize the plotting based on your specific requirements

%  %% Plots
% 
%  % i=1:N_sim;
  i = 1:10/Ts;
% 
 % Output 1
 figure(1)
 subplot(2,1,1)
 plot(i,refTrajectory(i,1),'k',i,xHistory(i,1),'r')
 legend('reference trajectory','x-position')
 xlabel('Time {s}')
 ylabel('Position {m}')

 % Output 2
 subplot(2,1,2)
 plot(i,refTrajectory(i,2),'k',i,xHistory(i,3),'r')
 legend('reference trajectory','y-position')
 xlabel('Time {s}')
 ylabel('Position {m}')

 figure(2)
 plot(i,uHistory(i,1),'r')
 legend('x-control input')
 xlabel('Time {s}')
 ylabel('Acceleration {m/s^2}')

 figure(3)
 plot(xHistory(i,1),xHistory(i,3),'b')
 hold on
 plot(xa,ya,'r')
 xlabel('x-Position')
 ylabel('y-Position {m}')

%  % Output 3
%  subplot(3,1,3)
%  plot(i,r(3,i),'k',i,yh(3,i),'r')
%  legend('reference trajectory','actual output')
%  xlabel('Time {ms}')
%  ylabel('\psi {rad}')

%  figure (2)
%  subplot(3,1,1)
%  plot(i, umax(1)*ones(1,100), '--k', i, umin(1)*ones(1,100), '--k',i, u1(1,i), '--g')
%  title('Plots of input over a sample period')
% 
%  subplot(3,1,2)
%  plot(i, umax(2)*ones(1,100), '--k', i, umin(2)*ones(1,100), '--k',i, u1(2,i), '--g')
% 
%  subplot(3,1,3)
%  plot(i, umax(3)*ones(1,100), '--k', i, umin(3)*ones(1,100), '--k',i, u1(3,i), '--g')
% 
%  figure (3)
%  subplot(3,1,1)
%  plot(i, Dumax(1)*ones(1,100), '--k', i, -Dumax(1)*ones(1,100), '--k', i, delt(1,i), '--g')
%  title('Change in input over a sample period')
% 
% 
%  subplot(3,1,2)
%  plot(i, Dumax(2)*ones(1,100), '--k', i, -Dumax(2)*ones(1,100), '--k', i, delt(2,i), '--g')
% 
%  subplot(3,1,3)
%  plot(i, Dumax(3)*ones(1,100), '--k', i, -Dumax(3)*ones(1,100), '--k', i, delt(3,i), '--g')