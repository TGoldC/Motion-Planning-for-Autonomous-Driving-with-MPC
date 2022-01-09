function x0 = init_STD(initState, p)
% init_STD - generates the initial state vector for the drift single
% track model
%
% Syntax:  
%     x0 = init_STD(initState, p)
%
% Inputs:
%     initState - core initial states
%     p - parameter vector
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
% x4 = v velocity at vehicle center
% x5 = Ψ yaw angle
% x6 = Ψ yaw rate
% x7 = β slip angle at vehicle center
% x8 = ωF front wheel angular speed
% x9 = ωR rear wheel angular speed

% obtain initial states from input
x0 = initState;
x0(8) = x0(4)*cos(x0(7))/(cos(x0(3))*p.R_w);
x0(9) = x0(4)*cos(x0(7))/(p.R_w);

%------------- END OF CODE --------------

