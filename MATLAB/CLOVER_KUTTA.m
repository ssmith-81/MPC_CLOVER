function [I,rhs] = CLOVER_KUTTA(I,trail_point,xa,ya,phi,Sj,n,flagKutta,rhs, U_inf, V_inf,xs,ys,xsi,ysi,g_source,g_sink)

% Form the last line of equation with the kutta condition
if (flagKutta(1) == 1)
    I(n+1,1)=1;
    I(n+1,n)=1;
    I(n+1,n+1) = 0;
    rhs(n+1)=0;
    for j=2:n-1
        I(n+1,j)=0;
    end
end

if (flagKutta(2) == 1)
    %rhs(n+1)= trail_point(2)*cos(alpha)-trail_point(1)*sin(alpha); 
    rhs(n+1)= trail_point(2)*U_inf-trail_point(1)*V_inf +(g_source/(2*pi))*atan2(trail_point(2)-ys,trail_point(1)-xs)...
        - (g_sink/(2*pi))*atan2(trail_point(2)-ysi,trail_point(1)-xsi);
    for j = 1:n+1  % Influence of element j on control point i
        if j==n+1
            I(n+1,j) = 1; %(i=row,j=column) equation (16)
        else
        x_o = trail_point(1) - xa(j);
        y_o = trail_point(2) - ya(j);
      
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
        I(n+1,j) = -(1/(2*pi))*(xbar*log(r2/r1)-Sj(j)*log(r2)+ybar*(omega1-omega2)); 
        
        end
    end
end
