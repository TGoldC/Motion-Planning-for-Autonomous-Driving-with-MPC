function res = test_zeroInitialVelocity()
% test_zeroInitialVelocity - unit_test_function for starting with zero 
% initial velocity
%
% Some vehicle models have a singularity when the vehicle is not moving.
% This test checks whether the vehicles can handle this situation
%
% Syntax:  
%    res = test_zeroInitialVelocity()
%
% Inputs:
%    ---
%
% Outputs:
%    res - boolean result (0/empty: test not passed, 1: test passed)
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: ---

% Author:       Matthias Althoff
% Written:      15-December-2017
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------

% initialize result
res = [];

% load parameters
p = parameters_vehicle2;
g = 9.81; %[m/s^2]

% set options --------------------------------------------------------------
tStart = 0; %start time
tFinal = 1; %start time

delta0 = 0;
vel0 = 0;
Psi0 = 0;
dotPsi0 = 0;
beta0 = 0;
sy0 = 0;
initialState = [0,sy0,delta0,vel0,Psi0,dotPsi0,beta0]; %initial state for simulation
x0_KS = init_KS(initialState); %initial state for kinematic single-track model
x0_ST = init_ST(initialState); %initial state for single-track model
x0_MB = init_MB(initialState, p); %initial state for multi-body model
%--------------------------------------------------------------------------

% set input: rolling car (velocity should stay constant)
u = [0 0];

% simulate multi-body model
[~,x_roll] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);

% simulate single-track model
[~,x_roll_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);

% simulate kinematic single-track model
[~,x_roll_ks] = ode45(getfcn(@vehicleDynamics_KS,u,p),[tStart, tFinal],x0_KS);

% check correctness
% ground truth
x_roll_gt = [ ...
0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, -0.0000000003174207, 0.0000000848065981, -0.0013834133396573, -0.0020336367252011, -0.0000000247286655, 0.0176248518072475, 0.0071655470428753, 0.0000000006677358, -0.0000001709775865, 0.0000001839820148, 0.0186763737562366, 0.0003752526345970, 0.0000000006728055, -0.0000001734436431, 0.0000001850020879, 0.0154621865353889, 0.0000251622262094, -0.0000174466440656, -0.0000174466440656, -0.0000014178345014, -0.0000014178345014, 0.0000000008088692, 0.0000000008250785];

% comparison
res(end+1) = all(abs(x_roll(end,:) - x_roll_gt) < 1e-14);
res(end+1) = all(x_roll_st(end,:) == x0_ST);
res(end+1) = all(x_roll_ks(end,:) == x0_KS);
%--------------------------------------------------------------------------

% set input: decelerating car ---------------------------------------------
v_delta = 0;
acc = -0.7*g;
u = [v_delta acc];

% simulate multi-body model
[~,x_dec] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);

% simulate single-track model
[~,x_dec_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);

% simulate kinematic single-track model
[~,x_dec_ks] = ode45(getfcn(@vehicleDynamics_KS,u,p),[tStart, tFinal],x0_KS);

% check correctness
% ground truth 
x_dec_gt = [ ...
3.9830932439714242, -0.0601543816394752, 0.0000000000000000, -8.0013986587693893, -0.0026467910011601, -0.0053025639381128, -0.0019453336082831, -0.0002270008481489, -0.0431740570135472, -0.0305313864800172, 0.1053033709671266, 0.0185102262795369, 0.0137681838589760, -0.0003400843778018, -0.0000161129034342, 0.0994502177784091, 0.0256268504637763, 0.0034700280714177, -0.0002562443897593, -0.0000034699487919, 0.1128675292571417, 0.0086968977905411, -0.0020987862166353, -0.0000183158385631, -0.0000183158385631, -0.0000095073736467, -0.0000095073736467, -0.0016872664171374, -0.0012652511246015];
x_dec_st_gt = [ ...
-3.4335000000000013, 0.0000000000000000, 0.0000000000000000, -6.8670000000000018, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000];
x_dec_ks_gt = [ ...
-3.4335000000000013, 0.0000000000000000, 0.0000000000000000, -6.8670000000000018, 0.0000000000000000];

% comparison
res(end+1) = all(abs(x_dec(end,:) - x_dec_gt) < 1e-14);
res(end+1) = all(abs(x_dec_st(end,:) - x_dec_st_gt) < 1e-14);
res(end+1) = all(abs(x_dec_ks(end,:) - x_dec_ks_gt) < 1e-14);
%--------------------------------------------------------------------------


% set input: accelerating car (wheel spin and velocity should increase; more wheel spin at rear)
v_delta = 0.15;
acc = 0.63*g;
u = [v_delta acc];

% simulate multi-body model
[~,x_acc] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);

% simulate single-track model
[~,x_acc_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);

% simulate kinematic single-track model
[~,x_acc_ks] = ode45(getfcn(@vehicleDynamics_KS,u,p),[tStart, tFinal],x0_KS);

% check correctness
% ground truth
x_acc_gt = [ ...
1.6869441956852231, 0.0041579276718349, 0.1500000000000001, 3.1967387404602654, 0.3387575860582390, 0.8921302762726965, -0.0186007698209413, -0.0556855538608812, 0.0141668816602887, 0.0108112584162600, -0.6302339461329982, 0.0172692751292486, 0.0025948291288222, -0.0042209020256358, -0.0115749221900647, 0.4525764527765288, 0.0161366380049974, -0.0012354790918115, -0.0023647389844973, -0.0072210348979615, -1.8660984955372673, 0.0179591511062951, 0.0010254111038481, 11.1322413877606117, 7.5792605585643713, 308.3079237740076906, 310.8801727728298374, -0.0196922024889714, -0.0083685253175425];
x_acc_st_gt = [ ...
3.0731976046859701, 0.2869835398304387, 0.1500000000000000, 6.1802999999999999, 0.1097747074946324, 0.3248268063223300, 0.0697547542798039];
x_acc_ks_gt = [ ...
3.0845676868494931, 0.1484249221523043, 0.1500000000000000, 6.1803000000000026, 0.1203664469224163];

%comparison
res(end+1) = all(abs(x_acc(end,:) - x_acc_gt) < 1e-14);
res(end+1) = all(abs(x_acc_st(end,:) - x_acc_st_gt) < 1e-14);
res(end+1) = all(abs(x_acc_ks(end,:) - x_acc_ks_gt) < 1e-14);
%--------------------------------------------------------------------------


% steering to left---------------------------------------------------------
v_delta = 0.15;
u = [v_delta 0];

% simulate multi-body model
[~,x_left] = ode45(getfcn(@vehicleDynamics_MB,u,p),[tStart, tFinal],x0_MB);

% simulate single-track model
[~,x_left_st] = ode45(getfcn(@vehicleDynamics_ST,u,p),[tStart, tFinal],x0_ST);

% simulate kinematic single-track model
[~,x_left_ks] = ode45(getfcn(@vehicleDynamics_KS,u,p),[tStart, tFinal],x0_KS);

% check correctness
% ground truth
x_left_gt = [ ...
0.0000000000000000, 0.0000000000000000, 0.1500000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0003021160057306, 0.0115474648881108, -0.0013797955031689, -0.0019233204598741, -0.0065044050021887, 0.0176248291065725, 0.0071641239008779, 0.0001478513434683, 0.0092020911982902, -0.0178028732533553, 0.0186751057310096, 0.0003566948613572, 0.0001674970785214, 0.0015871955172538, -0.0175512251679294, 0.0154636630992985, 0.0000482191918813, -0.0000173442953338, -0.0000174708138706, -0.0000014178345014, -0.0000014178345014, 0.0002293337149155, 0.0003694012334077];
x_left_st_gt = [ ...
0.0000000000000000, 0.0000000000000000, 0.1500000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0000000000000000]; 
x_left_ks_gt = [ ...
0.0000000000000000, 0.0000000000000000, 0.1500000000000000, 0.0000000000000000, 0.0000000000000000]; 

% comparison
res(end+1) = all(abs(x_left(end,:) - x_left_gt) < 1e-14);
res(end+1) = all(abs(x_left_st(end,:) - x_left_st_gt) < 1e-14);
res(end+1) = all(abs(x_left_ks(end,:) - x_left_ks_gt) < 1e-14);
%--------------------------------------------------------------------------

% obtain final result
res = all(res);

end

% add input and parameters to ode 
function [handle] = getfcn(fctName,u,p)
    
    function dxdt = f(t,x)
        dxdt = fctName(x,u,p);
    end

    handle = @f;
end



%------------- END OF CODE --------------
