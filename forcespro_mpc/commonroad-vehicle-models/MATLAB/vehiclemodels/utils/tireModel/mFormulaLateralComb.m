function F_y = mFormulaLateralComb(kappa, alpha, gamma, mu_y, F_z, F0_y, p)
%lateral tire forces (combined slip)

%turn slip is neglected, so xi_i=1;
%all scaling factors lambda = 1;

S_hykappa = p.r_hy1; 

kappa_s = kappa + S_hykappa;

B_ykappa = p.r_by1*cos(atan(p.r_by2*(alpha-p.r_by3)));
C_ykappa = p.r_cy1;
E_ykappa = p.r_ey1;
D_ykappa = F0_y/(cos(C_ykappa*atan(B_ykappa*S_hykappa - E_ykappa*(B_ykappa*S_hykappa - atan(B_ykappa*S_hykappa)))));

D_vykappa = mu_y*F_z*(p.r_vy1 + p.r_vy3*gamma) * cos(atan(p.r_vy4*alpha));
S_vykappa = D_vykappa*sin(p.r_vy5*atan(p.r_vy6*kappa));

%magic tire formula
F_y = D_ykappa*cos(C_ykappa*atan(B_ykappa*kappa_s - E_ykappa*(B_ykappa*kappa_s - atan(B_ykappa*kappa_s)))) + S_vykappa;