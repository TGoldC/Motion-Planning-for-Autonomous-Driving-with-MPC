function f = vehicleDynamics_KS(x,u,p)
% vehicleDynamics_KS - kinematic single-track vehicle dynamics 
% reference point: rear axle
%
% Syntax:  
%    f = vehicleDynamics_KS(x,u,p)
%
% Inputs:
%    x - vehicle state vector
%    u - vehicle input vector
%    p - vehicle parameter structure
%
% Outputs:
%    f - right-hand side of differential equations
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: ---

% Author:       Matthias Althoff
% Written:      12-January-2017
% Last update:  15-December-2017
% Last revision:---

%------------- BEGIN CODE --------------

%create equivalent kinematic single-track parameters
l = p.a + p.b;

%states
%x1 = s_x x-position in a global coordinate system
%x2 = s_y y-position in a global coordinate system
%x3 = δ steering angle of front wheels
%x4 = u velocity in x-direction
%x5 = Ψ yaw angle

%u1 = v_delta steering angle velocity of front wheels
%u2 = ax longitudinal acceleration

%consider steering constraints
u(1) = steeringConstraints(x(3),u(1),p.steering);

%consider acceleration constraints
u(2) = accelerationConstraints(x(4),u(2),p.longitudinal);

%system dynamics
f(1,1) = x(4)*cos(x(5));
f(2,1) = x(4)*sin(x(5));
f(3,1) = u(1);
f(4,1) = u(2);
f(5,1) = x(4)/l*tan(x(3));

%------------- END OF CODE --------------
