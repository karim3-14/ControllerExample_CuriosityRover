

%% LANDING (BODY)
%
% First we need to plot a path, using figures from the paper we are able to
% estimate the trajectory in x and z coordinates for the module during
% different phases of the Landing
% 
% Trajectory x landing phase 1 (t=0-60 seconds), trinomial seems to fit 
% the curve best from the paper. Afterwards phase 2 in landing starts, 
% which is straight down. After generating the polynomial paths, we will
% compound them to have one unique trajectory.

t0 = 0; %units seconds
t1 = 30;
t2 = 60;

%[pos0;V0;pos1;V1;pos2;V2]
phase1 = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 t1 t1^2 t1^3; ...
    0 1 2*t1 3*t1^2; 1 t2 t2^2 t2^3; 0 1 2*t2 3*t2^2]; %time eqns

%Z position and velocity, meters and m/s
Zp1 = [1400; -20; 500; 30; 200; -32];
Zp1C = phase1\Zp1;
tspan1 = 0:0.1:60; %time span for phase 1

%X position and velocity equations for phase 1
Xpos1 = @(t,Xp1C) Xp1C(1) + Xp1C(2).*t + Xp1C(3).*t.^2 + Xp1C(4).*t.^3;
XV1 = @(t,Xp1C) 0+Xp1C(2) + 2*Xp1C(3).*t + 3*Xp1C(4).*t.^2;

%Z position and velocity equations for phase 1
Zpos1 = @(t,Zp1C) Zp1C(1) + Zp1C(2).*t + Zp1C(3).*t.^2 + Zp1C(4).*t.^3;
ZV1 = @(t,Zp1C) 0+Zp1C(2) + 2*Zp1C(3).*t + 3*Zp1C(4).*t.^2;

%phase2 straight down
tspan2 = 60.1:0.1:160;
t0 = 60; %units seconds
t1 = 120;
t2 = 160;

phase2 = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 t1 t1^2 t1^3; ...
    0 1 2*t1 3*t1^2; 1 t2 t2^2 t2^3; 0 1 2*t2 3*t2^2]; %time eqns
Zp2 = [200; -32; 23; -0.75; 10; 0];
Zp2C = phase2\Zp2;
Zpos2 = @(t,Zp2C) Zp2C(1) + Zp2C(2).*t + Zp2C(3).*t.^2 + Zp2C(4).*t.^3;
ZV2 = @(t,Zp2C) 0+Zp2C(2) + 2*Zp2C(3).*t + 3*Zp2C(4).*t.^2;

%compounding paths
Z2 = Zpos2(tspan2,Zp2C);
Z1 = Zpos1(tspan1,Zp1C);
Z = zeros(1,length(Z1)+length(Z2));
Z(1:length(Z1)) = Z1;
Z(length(Z1)+1:length(Z2)+length(Z1)) = Z2;
X1 = -300:(300/length(tspan1)):-(300/length(tspan1));
X2 = zeros(1,length(Z2));
X = zeros(1,length(Z1)+length(Z2));
X(1:length(Z1)) = X1;
X(length(Z1)+1:length(Z2)+length(Z1)) = X2;
tspan3=0:0.1:160;


Path = figure;
plot(X,Z,'k');
grid on;
xlim([-300 300]);
xlabel('meters traveled in powered descent along X axis');
ylabel('meters traveled in powered descent along Z axis');
legend('Trajectory of Powered Descent');




%% Controlling for L, length of pendulum

t0 = 0;
t1 = 30;
t2 = 60;
%[pos0;V0;pos1;V1;pos2;V2]
phase1 = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 t1 t1^2 t1^3; ...
    0 1 2*t1 3*t1^2; 1 t2 t2^2 t2^3; 0 1 2*t2 3*t2^2]; %time eqns
%Z position and velocity, meters and m/s
Zp1 = [1400; -20; 500; 30; 200; -32];
Zp1C = phase1\Zp1;
tspan1 = 0:0.1:60; %time span for phase 1
%X position and velocity equations for phase 1
Xpos1 = @(t,Xp1C) Xp1C(1) + Xp1C(2).*t + Xp1C(3).*t.^2 + Xp1C(4).*t.^3;
XV1 = @(t,Xp1C) 0+Xp1C(2) + 2*Xp1C(3).*t + 3*Xp1C(4).*t.^2;
%Z position and velocity equations for phase 1
Zpos1 = @(t,Zp1C) Zp1C(1) + Zp1C(2).*t + Zp1C(3).*t.^2 + Zp1C(4).*t.^3;
ZV1 = @(t,Zp1C) 0+Zp1C(2) + 2*Zp1C(3).*t + 3*Zp1C(4).*t.^2;

%phase2 straight down
tspan2 = 60.1:0.1:160;
t0 = 60;
t1 = 120;
t2 = 160;

phase2 = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 t1 t1^2 t1^3; ...
    0 1 2*t1 3*t1^2; 1 t2 t2^2 t2^3; 0 1 2*t2 3*t2^2]; %time eqns
Zp2 = [200; -32; 23; -0.75; 10; 0];
Zp2C = phase2\Zp2;
Zpos2 = @(t,Zp2C) Zp2C(1) + Zp2C(2).*t + Zp2C(3).*t.^2 + Zp2C(4).*t.^3;
ZV2 = @(t,Zp2C) 0+Zp2C(2) + 2*Zp2C(3).*t + 3*Zp2C(4).*t.^2;

%compounding paths
Z2 = Zpos2(tspan2,Zp2C);
Z1 = Zpos1(tspan1,Zp1C);
Z = zeros(1,length(Z1)+length(Z2));
Z(1:length(Z1)) = Z1;
Z(length(Z1)+1:length(Z2)+length(Z1)) = Z2;
X1 = -300:(300/length(tspan1)):-(300/length(tspan1));
X2 = zeros(1,length(Z2));
X = zeros(1,length(Z1)+length(Z2));
X(1:length(Z1)) = X1;
X(length(Z1)+1:length(Z2)+length(Z1)) = X2;
tspan3=0:0.1:160;

% 
% Path = figure;
% plot(X,Z,'k');
% grid on;
% xlim([-300 300]);
% xlabel('meters traveled in powered descent along X axis');
% ylabel('meters traveled in powered descent along Z axis');
% legend('Trajectory of Powered Descent');

%powered_descent_traj;

kp1 = 1.5; % PD controller gains
kp2 = 1.1;
kp3 = 10.5;
kp4 = 1.1;
kd1 = 1;
kd2 = 1.5;
kd3 = 1;
kd4 = 1.5;
kp = [kp1, 0, 0; 0, kp2, 0; 0, 0, kp3];
kd = [kd1, 0, 0; 0, kd2, 0; 0, 0, kd3];


%param = {kp,kd,COEFF1,COEFF2,B} B is phase1 or phase 2 (1 or 2)
param = {kp,kd,Zp1C,Zp2C,1};

%Initial condition

init = [-300; 110; 1800; -20; 0.2; 0];
tspan = [1 60];
[T1,Xv1] = ode15s(@(t,x) descentODE1_3(t,x,param),tspan,init);

% tspan = [0 160];
% [T,Xv] = ode15s(@(t,x) descentODE1(t,x,param),tspan,init);

%Initial condition
%phase2
param = {kp,kd,Zp1C,Zp2C,2};
init = [Xv1(length(Xv1),1); Xv1(length(Xv1),2); Xv1(length(Xv1),3);...
    Xv1(length(Xv1),4); Xv1(length(Xv1),5); Xv1(length(Xv1),6)];
% init = [Xv1(length(Xv1),1); Xv1(length(Xv1),2); 100;...
%     Xv1(length(Xv1),4); Xv1(length(Xv1),5); Xv1(length(Xv1),6); ...
%     Xv1(length(Xv1),7); Xv1(length(Xv1),8)];

%init = [0; 5; 200; -32];
tspan = [60 160];
[T2,Xv2] = ode15s(@(t,x) descentODE2_3(t,x,param),tspan,init);

%compound trajectory into single variable
Xv = zeros(length(Xv1)+length(Xv2),6);
Xv(1:length(Xv1),:) = Xv1;
Xv((length(Xv1)+1):length(Xv),:)=Xv2;
T=zeros(length(T1)+length(T2),1);
T(1:length(T1)) = T1;
T((length(T1)+1):length(T))=T2;

Position=figure;
plot(Xv(:,1),Xv(:,3),'b--');
hold;
plot(X,Z,'r--');
hold;
grid on;
%xlim([-300 300]);
ylim([0 2000]);
xlabel('meters traveled in powered descent along X axis');
ylabel('meters traveled in powered descent along Z axis');
legend('Controlled','Desired Trajectory');
title('Position trajectories of the Sky-Crane: L Control');


% Velocity=figure;
% plot(T,Xv(:,2));
% hold;
% plot(T,Xv(:,4),'k');
% hold;
% grid on;
% xlabel('Time (s)');
% ylabel('Amplitude');
% legend('Vx (m/s)','Vz (m/s)');
% title('Velocity trajectories of the MER');

Velocity=figure;
plot(T,Xv(:,2) + sin((1/2.*pi.*sqrt((Xv(:,5)./3.7))).*cos(T./2.*pi.*sqrt((Xv(:,5)./3.7)))));
hold;
plot(T,Xv(:,4) + cos((1/2.*pi.*sqrt((Xv(:,5)./3.7))).*cos(T./2.*pi.*sqrt((Xv(:,5)./3.7)))),'k');
hold;
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
xlim([5 160]);
ylim([-50 50]);
legend('Vx (m/s)','Vz (m/s)');
title('Velocity trajectories of the Rover: L Control');

Velocity=figure;
plot(T,abs(Xv(:,2) + sin((1/2*pi*sqrt((15/3.7)))*cos(T./2*pi*sqrt((15/3.7))))-0.5),'r');
% hold;
% plot(T,Xv(:,4) + cos((1/2*pi*sqrt((15/3.7)))*cos(T./2*pi*sqrt((15/3.7)))),'k');
% hold;
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
xlim([100 160]);
ylim([0 5]);
legend('Vx (m/s)');
title('Velocity trajectories minus Touchdown Constraints of the Rover: L Control');

Position=figure;
plot(T,Xv(:,1));
hold;
plot(T,Xv(:,3),'k');
plot(tspan3,X,'--');
plot(tspan3,Z,'--');
hold;
grid on;
xlabel('Time (s)');
%xlim([-10 170]);
ylabel('Amplitude');
legend('X traveled(m)','Z traveled(m)','X trajectory','Z trajectory');
title('Position trajectories of the Sky-Crane: L Control');
 

PositionRover=figure;
plot(Xv(:,1)-(Xv(:,5).*cos((pi/2)*sin((Xv(:,5)./3.7).*T))),Xv(:,3)-(Xv(:,5).*sin((pi/2)*sin((Xv(:,5)./3.7).*T))),'b--');
hold;
plot(X,Z,'r--');
hold;
grid on;
%xlim([-300 300]);
ylim([0 2000]);
xlabel('meters traveled in powered descent along X axis');
ylabel('meters traveled in powered descent along Z axis');
legend('Controlled','Desired Trajectory');
title('Position Trajectory for Rover Pendulum: L Control');

%% Without controlling for L, length of pendulum

t0 = 0;
t1 = 30;
t2 = 60;
%[pos0;V0;pos1;V1;pos2;V2]
phase1 = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 t1 t1^2 t1^3; ...
    0 1 2*t1 3*t1^2; 1 t2 t2^2 t2^3; 0 1 2*t2 3*t2^2]; %time eqns
%Z position and velocity, meters and m/s
Zp1 = [1400; -20; 500; 30; 200; -32];
Zp1C = phase1\Zp1;
tspan1 = 0:0.1:60; %time span for phase 1
%X position and velocity equations for phase 1
Xpos1 = @(t,Xp1C) Xp1C(1) + Xp1C(2).*t + Xp1C(3).*t.^2 + Xp1C(4).*t.^3;
XV1 = @(t,Xp1C) 0+Xp1C(2) + 2*Xp1C(3).*t + 3*Xp1C(4).*t.^2;
%Z position and velocity equations for phase 1
Zpos1 = @(t,Zp1C) Zp1C(1) + Zp1C(2).*t + Zp1C(3).*t.^2 + Zp1C(4).*t.^3;
ZV1 = @(t,Zp1C) 0+Zp1C(2) + 2*Zp1C(3).*t + 3*Zp1C(4).*t.^2;

%phase2 straight down
tspan2 = 60.1:0.1:160;
t0 = 60;
t1 = 120;
t2 = 160;

phase2 = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 t1 t1^2 t1^3; ...
    0 1 2*t1 3*t1^2; 1 t2 t2^2 t2^3; 0 1 2*t2 3*t2^2]; %time eqns
Zp2 = [200; -32; 23; -0.75; 10; 0];
Zp2C = phase2\Zp2;
Zpos2 = @(t,Zp2C) Zp2C(1) + Zp2C(2).*t + Zp2C(3).*t.^2 + Zp2C(4).*t.^3;
ZV2 = @(t,Zp2C) 0+Zp2C(2) + 2*Zp2C(3).*t + 3*Zp2C(4).*t.^2;

%compounding paths
Z2 = Zpos2(tspan2,Zp2C);
Z1 = Zpos1(tspan1,Zp1C);
Z = zeros(1,length(Z1)+length(Z2));
Z(1:length(Z1)) = Z1;
Z(length(Z1)+1:length(Z2)+length(Z1)) = Z2;
X1 = -300:(300/length(tspan1)):-(300/length(tspan1));
X2 = zeros(1,length(Z2));
X = zeros(1,length(Z1)+length(Z2));
X(1:length(Z1)) = X1;
X(length(Z1)+1:length(Z2)+length(Z1)) = X2;
tspan3=0:0.1:160;

% 
% Path = figure;
% plot(X,Z,'k');
% grid on;
% xlim([-300 300]);
% xlabel('meters traveled in powered descent along X axis');
% ylabel('meters traveled in powered descent along Z axis');
% legend('Trajectory of Powered Descent');

%powered_descent_traj;

kp1 = 1.5; % PD controller gains
kp2 = 1;
kp3 = 1.5;
kp4 = 1.1;
kd1 = 1;
kd2 = 1;
kd3 = 1;
kd4 = 1.5;
kp = [kp1, 0; 0, kp2];
kd = [kd1, 0; 0, kd2];


%param = {kp,kd,COEFF1,COEFF2,B} B is phase1 or phase 2 (1 or 2)
param = {kp,kd,Zp1C,Zp2C,1};

%Initial condition

init = [-300; 110; 1800; -20];
tspan = [1 60];
[T1,Xv1] = ode15s(@(t,x) descentODE1_4(t,x,param),tspan,init);

% tspan = [0 160];
% [T,Xv] = ode15s(@(t,x) descentODE1(t,x,param),tspan,init);

%Initial condition
%phase2
param = {kp,kd,Zp1C,Zp2C,2};
init = [Xv1(length(Xv1),1); Xv1(length(Xv1),2); Xv1(length(Xv1),3);...
    Xv1(length(Xv1),4)];
% init = [Xv1(length(Xv1),1); Xv1(length(Xv1),2); 100;...
%     Xv1(length(Xv1),4); Xv1(length(Xv1),5); Xv1(length(Xv1),6); ...
%     Xv1(length(Xv1),7); Xv1(length(Xv1),8)];

%init = [0; 5; 200; -32];
tspan = [60 160];
[T2,Xv2] = ode15s(@(t,x) descentODE2_4(t,x,param),tspan,init);

%compound trajectory into single variable
Xv = zeros(length(Xv1)+length(Xv2),4);
Xv(1:length(Xv1),:) = Xv1;
Xv((length(Xv1)+1):length(Xv),:)=Xv2;
T=zeros(length(T1)+length(T2),1);
T(1:length(T1)) = T1;
T((length(T1)+1):length(T))=T2;

Position=figure;
plot(Xv(:,1),Xv(:,3),'b--');
hold;
plot(X,Z,'r--');
hold;
grid on;
%xlim([-300 300]);
ylim([0 2000]);
xlabel('meters traveled in powered descent along X axis');
ylabel('meters traveled in powered descent along Z axis');
legend('Controlled','Desired Trajectory');
title('Position Trajectory for the Sky-Crane: Control');


% Velocity=figure;
% plot(T,Xv(:,2));
% hold;
% plot(T,Xv(:,4),'k');
% hold;
% grid on;
% xlabel('Time (s)');
% ylabel('Amplitude');
% legend('Vx (m/s)','Vz (m/s)');
% title('Velocity trajectories of the MER');

Velocity=figure;
plot(T,Xv(:,2) + sin((1/2*pi*sqrt((15/3.7)))*cos(T./2*pi*sqrt((15/3.7)))));
hold;
plot(T,Xv(:,4) + cos((1/2*pi*sqrt((15/3.7)))*cos(T./2*pi*sqrt((15/3.7)))),'k');
hold;
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
xlim([5 160]);
ylim([-50 50]);
legend('Vx (m/s)','Vz (m/s)');
title('Velocity trajectories of the Rover: Control');

Velocity=figure;
plot(T,abs(Xv(:,2) + sin((1/2*pi*sqrt((15/3.7)))*cos(T./2*pi*sqrt((15/3.7))))-0.5),'r');
% hold;
% plot(T,Xv(:,4) + cos((1/2*pi*sqrt((15/3.7)))*cos(T./2*pi*sqrt((15/3.7)))),'k');
% hold;
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
xlim([100 160]);
ylim([0 5]);
legend('Vx (m/s)');
title('Velocity trajectories minus Touchdown Constraints of the Rover: Control');

Position=figure;
plot(T,Xv(:,1));
hold;
plot(T,Xv(:,3),'k');
plot(tspan3,X,'--');
plot(tspan3,Z,'--');
hold;
grid on;
xlabel('Time (s)');
%xlim([-10 170]);
ylabel('Amplitude');
legend('X traveled(m)','Z traveled(m)','X trajectory','Z trajectory');
title('Position trajectories of the Sky-Crane: Control');

PositionRover=figure;
plot(Xv(:,1)-(15*cos((pi/2)*sin((15/3.7)*T))),Xv(:,3)-(15*sin((pi/2)*sin((15/3.7)*T))),'b--');
hold;
plot(X,Z,'r--');
hold;
grid on;
%xlim([-300 300]);
ylim([0 2000]);
xlabel('meters traveled in powered descent along X axis');
ylabel('meters traveled in powered descent along Z axis');
legend('Controlled','Desired Trajectory');
title('Position Trajectory for Rover Pendulum: Control');