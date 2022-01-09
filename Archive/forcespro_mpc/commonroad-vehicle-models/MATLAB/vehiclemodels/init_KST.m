function x0 = init_KST(initState, alpha0)
% init_KST - generates the initial state vector for the kinematic
% single-track model with on-axle trailer
%
% Syntax:  
%     x0 = init_KST(initState, p)
%
% Inputs:
%     initState - core initial states
%     alpha0 - initial hitch angle
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

% Author:       Gerald Würsching
% Written:      26-October-2020
% Last update:  26-October-2020
% Last revision:---

%------------- BEGIN CODE --------------
% states
% x1 = s_x x-position in a global coordinate system
% x2 = s_y y-position in a global coordinate system
% x3 = δ steering angle of front wheels
% x4 = u velocity in x-direction
% x5 = Ψ yaw angle
% x6 = hitch angle

% obtain initial states from vector
sx0 = initState(1);
sy0 = initState(2);
delta0 = initState(3);
vel0 = initState(4);
Psi0 = initState(5);

x0 = [sx0, sy0, delta0, vel0, Psi0, alpha0];

%------------- END OF CODE --------------

