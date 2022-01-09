function f = vehicleDynamics_STD(x,u,p)
% vehicleDynamics_STD - drift single track model vehicle dynamics
% reference point: center of mass
%
% Syntax:  
%    f = vehicleDynamics_STD(x,u,p)
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

% set gravity constant
g = 9.81; %[m/s^2]

% create equivalent bicycle parameters
lf = p.a;
lr = p.b;
h = p.h_s;
m = p.m;
I = p.I_z;

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

% inputs
% u1 = v_delta steering angle velocity of front wheels
% u2 = acceleration

%consider steering constraints
u(1) = steeringConstraints(x(3),u(1),p.steering);
%consider acceleration constraints
u(2) = accelerationConstraints(x(4),u(2),p.longitudinal);



% switch to kinematic model for small velocities
if abs(x(4)) < 0.1
    % use kinematic model with ref point at center of mass
    lwb = p.a + p.b;
    %system dynamics
    f(1:5,1) = vehicleDynamics_KS_cog(x(1:5),u,p);
    % derivative of beta and yaw rate
    d_beta = (p.b * u(1)) / (lwb*cos(x(3))^2 * (1 + (tan(x(3))^2 * p.b/lwb)^2));
    dd_psi = 1/lwb * (u(2)*cos(x(7))*tan(x(3)) - x(3)*sin(x(7))*d_beta*tan(x(3)) + x(4)*cos(x(7))*u(1)/cos(x(3))^2);
    % derivative of angular speeds
    d_omega_f = 1/(cos(x(3))*p.R_w) * (u(2)*cos(x(7)) - x(4)*sin(x(7))*d_beta + x(4)*cos(x(7))*tan(x(3))*u(1));
    d_omega_r = 1/p.R_w * (u(2)*cos(x(7)) - x(4)*sin(x(7))*d_beta);
    f(6,1) = dd_psi;
    f(7,1) = d_beta;
    f(8,1) = d_omega_f;
    f(9,1) = d_omega_r;
else
    % compute lateral tire slip angles
    alpha_f = atan((x(4)*sin(x(7)) + x(6)*lf) / (x(4)*cos(x(7)))) - x(3);
    alpha_r = atan((x(4)*sin(x(7)) - x(6)*lr) / (x(4)*cos(x(7))));
    
    % compute vertical tire forces
    F_zf = (m*g*lr) / (lr+lf);
    F_zr = (m*g*lf) / (lr+lf);
    
    % compute front and rear tire speeds
    u_wf = max(0, x(4)*cos(x(7))*cos(x(3)) + (x(4)*sin(x(7)) + lf*x(6))*sin(x(3)));
    u_wr = max(0, x(4)*cos(x(7)));
    
    % compute longitudinal tire slip
    s_f = 1 - p.R_w*x(8)/u_wf;
    s_r = 1 - p.R_w*x(9)/u_wr;
    
    
    % compute tire forces (Pacejka)
    % pure slip longitudinal forces
    F0_xf = mFormulaLongitudinal(s_f, 0, F_zf, p.tire);
    F0_xr = mFormulaLongitudinal(s_r, 0, F_zr, p.tire);
    
    % pure slip lateral forces
    [F0_yf, mu_yf] = mFormulaLateral(alpha_f, 0, F_zf, p.tire);
    [F0_yr, mu_yr] = mFormulaLateral(alpha_r, 0, F_zr, p.tire);
    
    % combined slip longitudinal forces
    F_xf = mFormulaLongitudinalComb(s_f, alpha_f, F0_xf, p.tire);
    F_xr = mFormulaLongitudinalComb(s_r, alpha_r, F0_xr, p.tire);
    
    % combined slip lateral forces
    F_yf = mFormulaLateralComb(s_f, alpha_f, 0, mu_yf, F_zf, F0_yf, p.tire);
    F_yr = mFormulaLateralComb(s_r, alpha_r, 0, mu_yr, F_zr, F0_yr, p.tire);
    
    % convert acceleration input to brake and engine torque
    if u(2)>0
        T_B = 0;
        T_E = m*p.R_w*u(2);
    else
        T_B = m*p.R_w*u(2);
        T_E = 0;
    end
    
    
    % system dynamics
    d_v = 1/m * (-F_yf*sin(x(3)-x(7)) + F_yr*sin(x(7)) + F_xr*cos(x(7)) + F_xf*cos(x(3)-x(7)));
    dd_psi = 1/I * (F_yf*cos(x(3))*lf - F_yr*lr + F_xf*lf*sin(x(3)));
    d_beta = -x(6) + 1/(m*x(4)) * (F_yf*cos(x(3)-x(7)) + F_yr*cos(x(7)) - F_xr*sin(x(7)) + F_xf*sin(x(3)-x(7)));
    
    % wheel dynamics (negative wheel spin forbidden)
    if x(8)>=0
        d_omega_f = 1/p.I_y_w * (-p.R_w*F_xf + p.T_sb*T_B + p.T_se*T_E);
    else
        d_omega_f = 0;
        x(8)=0;
    end
    
    if x(9)>=0
        d_omega_r = 1/p.I_y_w * (-p.R_w*F_xr + (1-p.T_sb)*T_B + (1-p.T_se)*T_E);
    else
        d_omega_r = 0;
        x(9)=0;
    end
    
    % construct output vector
    f(1,1) = x(4)*cos(x(7)+x(5));
    f(2,1) = x(4)*sin(x(7)+x(5));
    f(3,1) = u(1);
    f(4,1) = d_v;
    f(5,1) = x(6);
    f(6,1) = dd_psi;
    f(7,1) = d_beta;
    f(8,1) = d_omega_f;
    f(9,1) = d_omega_r;
end

%------------- END OF CODE --------------


