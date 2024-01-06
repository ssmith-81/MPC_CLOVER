function cineq = CloverConstraint(X,U,e,data,params)

 % Extract
    R_ele = params(1);
    x_obs = params(2:502);
    y_obs = params(503:1003);

    x_center = params(1004);
    y_center = params(1005);

p = data.PredictionHorizon;  % Read the prediction horizon p from the data

 % Extract the current state position values (used for the collision
 % avoidance). For now, we will calculate the current critical distance
 % (closest distance from the current drone position to the static
 % obsticle, i.e. identify the obstacle critical point). Later we will
 % consider this point as moving, and it will be based on LiDar feedback.
    X_cur = X(1,[1,3]);

    %% avoidance via minimum distance from single data point
 % Find the current minimum distance to the obstacle

 p_to = X_cur - [x_obs',y_obs'];

 squared_distance = sum(p_to.*p_to,2);

 % Find the minimum value and its index
[minValue, minIndex] = min(squared_distance);

% We now have the current critical point on the obstacle. 
X_crit = x_obs(minIndex);
Y_crit = y_obs(minIndex);

%% Avoidance via distance from center of circle
% Set the critical object location via center
% X_crit = x_center;
% Y_crit = y_center;

% disp(minIndex);
% disp(size(x_obs));
% We will consider this critical point stationary for now, i.e. the same
% value/position at each prediction horizon value:

X_crit = X_crit.*ones(length(2:p+1),1);
Y_crit = Y_crit.*ones(length(2:p+1),1);

XY_crit = [X_crit, Y_crit];

% Extract decision variables (excluding duplicates)
    X_dec = X(2:p+1,[1,3,5]); % Extract position [x,y,z]

% Create a desired distance from the critical point (an array for each step
% in the prediction horizon)
R_des = 1.*ones(length(2:p+1),1);

% Error between drones prediction horizon and critical point
% error varibles

E_X =  XY_crit - X_dec(:,[1,2]);  



mag = sum((E_X).*(E_X),2);


cineq = R_des.^2 - mag;



end