function [xmid,ymid,dx,dy,Sj,phiD,rhs] = CLOVER_COMPONENTS(xa,ya,U_inf, V_inf,g_source,g_sink,xs,ys,xsi,ysi,n)
% obs- number of obstacles in the environment
% Find geometric quantities of the obstacle
for i = 1:1:n                                                          % Loop over all panels
    xmid(i) = (xa(i)+xa(i+1))/2;                                            % X-value of control point
    ymid(i) = (ya(i)+ya(i+1))/2;                                            % Y-value of control point
    dx      = xa(i+1)-xa(i);                                                % Change in X between boundary points
    dy      = ya(i+1)-ya(i);                                                % Change in Y between boundary points
    Sj(i)    = (dx^2 + dy^2)^0.5;                                           % Length of the panel
	phiD(i) = atan2d(dy,dx);                                                % Angle of the panel (positive X-axis to inside face) [deg]
    if (phiD(i) < 0)                                                        % Make all panel angles positive [deg]
        phiD(i) = phiD(i) + 360;
    end
    rhs(i)= U_inf*ymid(i)-V_inf*xmid(i) ...
        +(g_source/(2*pi))*atan2(ymid(i)-ys,xmid(i)-xs)...
        - (g_sink/(2*pi))*atan2(ymid(i)-ysi,xmid(i)-xsi);                          % RHS of stream function equation
end

end