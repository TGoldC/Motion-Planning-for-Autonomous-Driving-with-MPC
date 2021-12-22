function x0 = init_KS(initState)
% init_KS - generates the initial state vector for the kinematic
% single-track model
%
% Syntax:  
%     x0 = init_KS(initState, p)
%
% Inputs:
%     initState - core initial states
%
% Outputs:
%     x0 - initial state vector
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: ---

% Author:       Matthias Althoff
% Written:      11-January-2017
% Last update:  15-December-2017
% Last revision:---

%------------- BEGIN CODE --------------

%states
%x1 = s_x x-position in a global coordinate system
%x2 = s_y y-position in a global coordinate system
%x3 = δ steering angle of front wheels
%x4 = u velocity in x-direction
%x5 = Ψ yaw angle

%u1 = v_delta steering angle velocity of front wheels
%u2 = ax longitudinal acceleration


%obtain initial states from vector
sx0 = initState(1);
sy0 = initState(2);
delta0 = initState(3);
vel0 = initState(4);
Psi0 = initState(5);

%sprung mass states
x0(1) = sx0; % s_x x-position in a global coordinate system
x0(2) = sy0; % s_y y-position in a global coordinate system
x0(3) = delta0; % steering angle of front wheels
x0(4) = vel0; % velocity
x0(5) = Psi0; % Ψ yaw angle

%------------- END OF CODE --------------
