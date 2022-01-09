function [F_y, mu_y] = mFormulaLateral(alpha, gamma, F_z, p)
%Pacejka lateral tire forces (pure slip)

%turn slip is neglected, so xi_i=1;
%all scaling factors lambda = 1;

%coordinate system transformation
%alpha = -alpha;

S_hy = sign(gamma)*(p.p_hy1 + p.p_hy3*abs(gamma));
S_vy = sign(gamma)*F_z*(p.p_vy1 + p.p_vy3*abs(gamma));

alpha_y = alpha + S_hy;
mu_y = p.p_dy1*(1-p.p_dy3*gamma^2);

C_y = p.p_cy1;
D_y = mu_y*F_z;
E_y = p.p_ey1;
K_y = F_z*p.p_ky1; %simplify K_y0 to p.p_ky1*F_z
B_y = K_y/(C_y*D_y);

%magic tire formula
F_y = D_y*sin(C_y*atan(B_y*alpha_y - E_y*(B_y*alpha_y - atan(B_y*alpha_y)))) + S_vy;