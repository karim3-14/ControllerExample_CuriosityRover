function dX = descentODE1_4(t,x,param)
%DESCENTODE Summary of this function goes here
%   Detailed explanation goes here

%Kinematics of sky crane
%[x1;x2;x3;x4] = [x;dx;z;dz]; 
%[x1;x2;x3;x4] = [x;dx;z;dz];
%properties of lander
g = 3.7;
m1 = 100; %kg
m2 = 1110; %kg
air_rho = 610 / (.1921 * (273.1-31)); %kg/m^3
Cd = 90 * (1.35^2*pi)*air_rho; %1/A^2
dl = 0;
l = 15;
lambda = 2*pi*sqrt(l/g);
f = 1/lambda;
q = sin(f*t);
dq = f*cos(f*t);


%param = [Kp,Kd,COEFF1,COEFF2]

Kp = cell2mat(param(1));
Kd = cell2mat(param(2));
% Zp1C = cell2mat(param(3));
% Zp2C = cell2mat(param(4));
Zp1C = cell2mat(param(3));
Zp2C = cell2mat(param(4));
B = cell2mat(param(5));
%%
% Figure out how to do interrupted ODEs
%xf = [x,z,dx,dz]
% if B == 1    
%     %X position and velocity equations for phase 1
%     xf(1) = -300 + 5*t;
%     %xf(3) = 100 - 0.027*t^2;
%     xf(3) = 0;
%     %Z position and velocity equations for phase 1
%     xf(2) = Zp1C(1) + Zp1C(2).*t + Zp1C(3).*t.^2 + Zp1C(4).*t.^3;
%     %xf(4) = 0+Zp1C(2) + 2*Zp1C(3).*t + 3*Zp1C(4).*t.^2;
%     xf(4)=0;
% else
%     %X position and velocity equations for phase 2
%     xf(1) = 0*t;
%     xf(3) = 0*t;
%     %Z position and velocity equations for phase 2
%     xf(2) = Zp2C(1) + Zp2C(2).*t + Zp2C(3).*t.^2 + Zp2C(4).*t.^3;
%     %xf(4) = 0+Zp2C(2) + 2*Zp2C(3).*t + 3*Zp2C(4).*t.^2;
%     xf(4) = 0;
% end 

% xf(1) = Xp1C(1) + Xp1C(2).*t + Xp1C(3).*t.^2 + Xp1C(4).*t.^3;
% xf(3) = 0+Xp1C(2) + 2*Xp1C(3).*t + 3*Xp1C(4).*t.^2;

% X trajectories
xf(1) = (-300 + 0.083*t^2)-(15*sin(q));
xf(3) = 10-0.1667*t - (15*cos(q));
% Z trajectories
xf(2) = Zp1C(1) + Zp1C(2).*t + Zp1C(3).*t.^2 + Zp1C(4).*t.^3 ;
xf(4) = 0+Zp1C(2) + 2*Zp1C(3).*t + 3*Zp1C(4).*t.^2 ;
%xf(4)=0;

% % Q trajectories
% xf(5) = 0;
% xf(7) = 0;

% L trajectories
% xf(5) = 0.1;
% xf(6) = 0;

a = -Kp*([x(1);x(3)]-[(xf(1));(xf(2))]) - ...
    Kd*([x(2);x(4)]-[xf(3);xf(4)]);
%a is an approximation for the acceleration terms, [a1;a2]=[ddx;ddz];
dX = zeros(4,1);
% Fx = -a(1)*m1*x(2)*(Cd-1);
% Fz = (49*m1)/5 + (49*m2)/5 + a(2)*(x(4)*m1 - Cd*x(4)*m1); %First eqns
% Fx = -a(1)*m1*(Cd-1);
% Fz = 9.8*(m1+m2) + a(2)*(m1-Cd*m1); %2nd attempt

Fx = 0.5*(Cd*m1*x(2)^2)+a(1)*(m1+m2-Cd*m1*x(1))+l*dq*m2*sin(q);
Fz = 3.7*m1+3.7*m2 + a(2)*(m1+m2-Cd*m1) + l*dq*m2*cos(q);
% Tau = l*(l*a(3) + 3.7*m2*sin(q) - m2*x(3)*cos(q) - ...
%     m2*x(1)*sin(q) + dq*m2*x(1)*cos(q) - dq*m2*x(3)*cos(q));
% Fl = m2*(a(3)-3.7*cos(5) - l*dq^2 + dq*x(3)*cos(q) + ...
%     dq*x(1)*sin(q));

% dX(1) = x(2);
% dX(2) = -Fx/(m1*(Cd-1));
% dX(3) = x(4);
% dX(4) = (Fz - 9.8*(m1+m2))/(m1*(1-Cd));

dX(1) = x(2);
dX(2) = (Fx - (0.5*Cd*m1*x(2)^2) - ...
    l*dq*m2*sin(q))/(m1 + m2 - Cd*m1*x(1));
dX(3) = x(4);
dX(4) = (Fz - 3.7*(m1 +m2) - l*dq*cos(q)*m2)/(m1+m2-Cd*m1); 

% dl= 1;
% ddl= 0;
end

