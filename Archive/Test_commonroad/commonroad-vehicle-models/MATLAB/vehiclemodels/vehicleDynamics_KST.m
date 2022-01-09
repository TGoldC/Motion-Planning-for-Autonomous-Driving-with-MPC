function f = vehicleDynamics_KST(x,u,p)
% vehicleDynamics_KST - kinematic single-track with on-axle trailer 
% reference point: rear axle
%
% Syntax:  
%    f = vehicleDynamics_KST(x,u,p)
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

% Author:       Gerald Würsching
% Written:      26-October-2020
% Last update:  26-October-2020
% Last revision:---

%------------- BEGIN CODE --------------

%create equivalent kinematic single-track parameters
l_wb = p.a + p.b;       % wheel base
l_wbt = p.trailer.l_wb; % wheel base trailer

%states
%x1 = s_x x-position in a global coordinate system
%x2 = s_y y-position in a global coordinate system
%x3 = δ steering angle of front wheels
%x4 = u velocity in x-direction
%x5 = Ψ yaw angle
%x6 = hitch angle

%inputs
%u1 = v_delta steering angle velocity of front wheels
%u2 = ax longitudinal acceleration

%consider steering constraints
u(1) = steeringConstraints(x(3),u(1),p.steering);
%consider acceleration constraints
u(2) = accelerationConstraints(x(4),u(2),p.longitudinal);

% hitch angle constraints
if -pi/2 <= x(6) <= pi/2
    d_alpha = -x(4) * (sin(x(6))/l_wbt + tan(x(3))/l_wb);
else
    d_alpha = 0;
    if x(6) < -pi/2
        x(6) = -pi/2;
    else
        x(6) = pi/2;
    end
end

%system dynamics
f(1,1) = x(4)*cos(x(5));
f(2,1) = x(4)*sin(x(5));
f(3,1) = u(1);
f(4,1) = u(2);
f(5,1) = x(4)/l_wb*tan(x(3));
f(6,1) = d_alpha;

%------------- END OF CODE --------------
