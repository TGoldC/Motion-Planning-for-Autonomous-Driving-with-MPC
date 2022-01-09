function F_x = mFormulaLongitudinal(kappa, gamma, F_z, p)
%Pacejka longitudinal tire forces (pure slip)

%turn slip is neglected, so xi_i=1;
%all scaling factors lambda = 1;

%coordinate system transformation
kappa = -kappa;

S_hx = p.p_hx1;
S_vx = F_z*p.p_vx1;

kappa_x = kappa + S_hx;
mu_x = p.p_dx1*(1-p.p_dx3*gamma^2);

C_x = p.p_cx1;
D_x = mu_x*F_z;
E_x = p.p_ex1;
K_x = F_z*p.p_kx1;
B_x = K_x/(C_x*D_x);

%magic tire formula
F_x = D_x*sin(C_x*atan(B_x*kappa_x - E_x*(B_x*kappa_x - atan(B_x*kappa_x))) + S_vx);
