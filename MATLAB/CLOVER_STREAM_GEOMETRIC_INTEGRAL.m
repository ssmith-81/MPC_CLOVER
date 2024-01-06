function [I] = CLOVER_STREAM_GEOMETRIC_INTEGRAL(xmid,ymid,xa,ya,phi,Sj,n)

%% Evaluate Geometric integral for vortex panel strengths


% This loop calculates the coefficients to find the matrix I_ij for equation
% 1.Variables are as given in report.
for i = 1:n    % iterate through each control point
    for j = 1:n+1  % Influence of element j on control point i
        if j==n+1
            I(i,j) = 1; %(i=row,j=column) equation (16)
        else
        % Transform the current control point i into the current panel j reference
        % frame:
        x_o = xmid(i) - xa(j);
        y_o = ymid(i) - ya(j);
        phi(j) = phi(j);
        % Rotate about origin o to complete transformation
         xbar = x_o*cos(phi(j)) + y_o*sin(phi(j));
         ybar = -x_o*sin(phi(j)) + y_o*cos(phi(j));

        % First calculate r1 and r2 between the current panel j endpoints
        % and the current controlpoint i
        r1 = sqrt((xbar)^2 + (ybar)^2);
        r2 = sqrt((xbar-Sj(j))^2 + (ybar)^2);

        % Calculate omega angles between each control point i
        % and each panel endpoints j (store for calculating normal and tangential velocities later):
        omega1 = atan2(ybar,xbar);
        omega2 = atan2(ybar,xbar-Sj(j));
        

        % Compute I_(i,j) geometric values for each panel j on
        % control point i. (ex on control point 1=i iterate over each panel
        % j)
        
        % MATLAB: log() is natural log, log2() is base 2, log10() is base 10, log1p() is natural log of 1 + x
        I(i,j) = -(1/(2*pi))*(xbar*log(r2/r1)-Sj(j)*log(r2)+ybar*(omega1-omega2)); 
       
        end
    end
end