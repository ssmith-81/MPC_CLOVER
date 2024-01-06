function J = CloverCost(X, U, e, data,params)

% Refer to https://www.mathworks.com/help/mpc/ug/optimization-problem.html#bujxw9t
% to understand the basic cost function formulation in this function
    % Extract
    R_ele = params(1);
    x_obs = params(2:502);
    y_obs = params(503:1003);

    p = data.PredictionHorizon;  % Read the prediction horizon p from the data
    ref = data.References;       % Take in the reference trajectory from data

    % Unpack relevant information
    %Q = [1,1,1,1,1,1].*eye(6); % State weight matrix
    Q = [10,10,10].*eye(3); % State weight matrix (3 states for consideration)
    R = [R_ele,R_ele,R_ele].*eye(3); % Input weight matrix
    R_du = [0.0001,0.0001,0.0001].*eye(3); % change in input weight matrix
    
%     Q = kron(eye(p),Q);
%     R = kron(eye(p),R);
%     R_du = kron(eye(p),R_du);

    % Extract the current state position values (used for the collision avoidance
    % potential field cost component
    X_cur = X(1,[1,3]);

    % Extract decision variables (excluding duplicates)

    % All of the states are used for cost function miniization:
%      X_dec = X(2:p+1, :);
    X_dec = X(2:p+1,[1,3,5]); % Extract position [x,y,z]
     U_dec1 = U(1:p, :);

     U_dec2 = U(2:p, :);
     U_dec2 = [zeros(1, size(U_dec2, 2));U_dec2];
     
%     X_dec = X(2:end, :);
%     U_dec = U(1:end-1, :);

     % error varibles
     E_X = ref(1:p,:) - X_dec;  % Size 6x6. columns = outputs, rows = ny prediction horizon
     Delta_u = U_dec2 - U_dec1;

    U_dec1 = U_dec1';
    Delta_u = Delta_u';
    E_X = E_X';

    % Collision avoidance potential function
    r_des = 0.2;
    
%      disp(size(Q));
%      disp(size(U_dec1));
%          disp(size(R));
%      disp(size(Delta_u));
%      disp(size(R_du));
    % Compute the quadratic cost
    J = 0;
    for i=1:p
    Je = E_X(:,i)' * Q * E_X(:,i); % Tracking error cost component
    Ju = U_dec1(:,i)' * R * U_dec1(:,i); % Control input cost component
    Jdu = Delta_u(:,i)' * R_du * Delta_u(:,i); % smoothness of control input cost
    Jep = 0.0001 * e;
    % Collision avoidance contribution
   
    J1 = Je + Ju;% + Jdu; % Include the relevant contrabutions
    J = J + J1;
    end
    % Optionally, you can include soft constraints penalty with slack (e)
    % Assuming soft constraints are part of the cost function
    %J = J + params.SoftConstraintWeight * e;
    J = J + 0.0001 * e;

    % Obstacle avoidance contribution:
    lambda = 1500000000000000;
    k = 15;
    % Fine the minimim distance to the obstacle
%     disp(size(X_cur));
%     disp(size([x_obs,y_obs]));
    p_to = X_cur - [x_obs',y_obs'];
    %disp(size(p_to));
    squared_distance = p_to.*p_to;
    
    min_sd = min(squared_distance);

    d_t = r_des^2 - (min_sd(1,1)+min_sd(1,2));

    Joa = lambda/(1+exp(-k*d_t));
   % disp(Joa);
    % Include obstacle avoidance contribution
    %J = J+Joa;

    % Info:
%     in general, for cost functions, do not use the following values, 
% since they are not part of the decision variables used by the solver:
% 
% U(end,:) — This row is a duplicate of the preceding row.
% 
% X(1,:) — This row contains the current state values.
end
