% Script for testing different vehicle models (see examples in chap. 11 of documentation)

%% set parameters and options
clear
addpath(genpath('../vehiclemodels'))

% load parameters
p = parameters_vehicle2;
g = 9.81; %[m/s^2]

% set options
tStart = 0; %start time
tFinal = 1; %start time

delta0 = 0;
vel0 = 5;
Psi0 = 0;
dotPsi0 = 0;
beta0 = 0;
sy0 = 0;
initialState = [0,sy0,delta0,vel0,Psi0,dotPsi0,beta0]; %initial state for simulation
x0_KS = init_KS(initialState); %initial state for kinematic single-track model
x0_ST = init_ST(initialState); %initial state for single-track model
x0_MB = init_MB(initialState, p); %initial state for multi-body model
x0_STD = init_STD(initialState, p);



%% simulate steady-state cornering 
v_delta = 0.15;
u = [v_delta 0];
%simulate MB
[t_coast_mb,x_coast_mb] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);
%simulate ST
[t_coast_st,x_coast_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);
%simulate KS
[t_coast_ks,x_coast_ks] = ode45(getfcn(@vehicleDynamics_KS,u,p),[tStart, tFinal],x0_KS);
%simulate STD
[t_coast_std,x_coast_std] = ode45(getfcn(@vehicleDynamics_STD,u,p),[tStart, tFinal],x0_STD);

%% simualte accelerating
v_delta = 0.15;
acc = 0.63*g;
u = [v_delta acc];
% simulate MB
[t_acc_mb,x_acc_mb] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);
% simulate ST
[t_acc_st,x_acc_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);
% simulate STD
[t_acc_std,x_acc_std] = ode45(getfcn(@vehicleDynamics_STD,u,p),[tStart, tFinal],x0_STD);

%% simulate braking
v_delta = 0.15;
acc = -0.7*g;
u = [v_delta acc];
% simulate MB
[t_brake_mb,x_brake_mb] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);
% simulate ST
[t_brake_st,x_brake_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);
% simulate STD
[t_brake_std,x_brake_std] = ode45(getfcn(@vehicleDynamics_STD,u,p),[tStart, tFinal],x0_STD);



%% compare steady-state cornering
figure %position
hold on
plot(x_coast_mb(:,1),x_coast_mb(:,2))
plot(x_coast_st(:,1),x_coast_st(:,2))
plot(x_coast_ks(:,1),x_coast_ks(:,2))
plot(x_coast_std(:,1),x_coast_std(:,2))
legend('MB', 'ST', 'KS', 'STD')
title('position')
figure %slip angle
hold on
plot(t_coast_mb,atan(x_coast_mb(:,11)./x_coast_mb(:,4)))
plot(t_coast_st,x_coast_st(:,7))
plot(t_coast_std,x_coast_std(:,7))
legend('MB', 'ST', 'STD')
title('slip angle')


%% compare braking
figure %position
hold on
plot(x_brake_mb(:,1),x_brake_mb(:,2))
plot(x_brake_std(:,1),x_brake_std(:,2))
legend('MB','STD')
title('position')
figure % velocity
hold on
plot(t_brake_mb,x_brake_mb(:,4))
plot(t_brake_std,x_brake_std(:,4).*cos(x_brake_std(:,7)))
figure % wheel spin MB
hold on
plot(t_brake_mb,x_brake_mb(:,24));
plot(t_brake_mb,x_brake_mb(:,25));
plot(t_brake_mb,x_brake_mb(:,26));
plot(t_brake_mb,x_brake_mb(:,27));
title('wheel spin MB')
figure % wheel spin STD
hold on
plot(t_brake_std, x_brake_std(:, 8));
plot(t_brake_std, x_brake_std(:, 9));
title('wheel spin STD')


%% oversteer understeer MB
figure % position
hold on
plot(x_coast_mb(:,1),x_coast_mb(:,2))
plot(x_brake_mb(:,1),x_brake_mb(:,2))
plot(x_acc_mb(:,1),x_acc_mb(:,2))
legend('coast','brake','acc')
title('position')
figure % slip angles
hold on
plot(t_coast_mb,atan(x_coast_mb(:,11)./x_coast_mb(:,4)))
plot(t_brake_mb,atan(x_brake_mb(:,11)./x_brake_mb(:,4)))
plot(t_acc_mb,atan(x_acc_mb(:,11)./x_acc_mb(:,4)))
legend('coast','brake','acc')
title('slip angles')
figure % orientation
hold on
plot(t_coast_mb,x_coast_mb(:, 5))
plot(t_brake_mb,x_brake_mb(:, 5))
plot(t_acc_mb,x_acc_mb(:, 5))
legend('coast','brake','acc')
title('orientation')
figure % pitch
hold on
plot(t_coast_mb,x_coast_mb(:, 9))
plot(t_brake_mb,x_brake_mb(:, 9))
plot(t_acc_mb,x_acc_mb(:, 9))
legend('coast','brake','acc')
title('pitch angle')


%% oversteer understeer STD
figure % position
hold on
plot(x_coast_std(:,1),x_coast_std(:,2))
plot(x_brake_std(:,1),x_brake_std(:,2))
plot(x_acc_std(:,1),x_acc_std(:,2))
legend('coast','brake','acc')
title('position')
figure % slip angles
hold on
plot(t_coast_std,x_coast_std(:,7))
plot(t_brake_std,x_brake_std(:,7))
plot(t_acc_std,x_acc_std(:,7))
legend('coast','brake','acc')
title('slip angles')
figure % orientation
hold on
plot(t_coast_std,x_coast_std(:, 5))
plot(t_brake_std,x_brake_std(:, 5))
plot(t_acc_std,x_acc_std(:, 5))
legend('coast','brake','acc')
title('orientation')


%% oversteer understeer ST
figure % position
hold on
plot(x_coast_st(:,1),x_coast_st(:,2))
plot(x_brake_st(:,1),x_brake_st(:,2))
plot(x_acc_st(:,1),x_acc_st(:,2))
legend('coast','brake','acc')
title('position')
figure % slip angles
hold on
plot(t_coast_st,x_coast_st(:,7))
plot(t_brake_st,x_brake_st(:,7))
plot(t_acc_st,x_acc_st(:,7))
legend('coast','brake','acc')
title('slip angles')
figure % orientation
hold on
plot(t_coast_st,x_coast_st(:, 5))
plot(t_brake_st,x_brake_st(:, 5))
plot(t_acc_st,x_acc_st(:, 5))
legend('coast','brake','acc')
title('orientation')


%% comparison KS KS2
% steady-state cornering 
v_delta = 0;
u = [v_delta 0];
x0_KS(3) = 0.1;
%simulate KS
[t_coast_ks,x_coast_ks] = ode45(getfcn(@vehicleDynamics_KS,u,p),[tStart, tFinal],x0_KS);
%simulate KS2
%initialState = [p.b,sy0,delta0,vel0,Psi0,dotPsi0,beta0];
%x0_KS = init_KS(initialState);
[t_coast_ks2,x_coast_ks2] = ode45(getfcn(@vehicleDynamics_KS2,u,p),[tStart, tFinal],x0_KS);
figure %position
hold on
plot(x_coast_ks2(:,1),x_coast_ks2(:,2))
plot(x_coast_ks(:,1),x_coast_ks(:,2))
figure %orientation
hold on
plot(t_coast_ks2,x_coast_ks2(:,5))
plot(t_coast_ks,x_coast_ks(:,5))


%% add input and parameters to ode 
function [handle] = getfcn(fctName,u,p)
    
    function dxdt = f(t,x)
        dxdt = fctName(x,u,p);
    end

    handle = @f;
end