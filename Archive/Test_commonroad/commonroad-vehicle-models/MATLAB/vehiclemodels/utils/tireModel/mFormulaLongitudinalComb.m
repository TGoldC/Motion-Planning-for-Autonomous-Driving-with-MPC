function F_x = mFormulaLongitudinalComb(kappa, alpha, F0_x, p)
%Pacejka longitudinal tire forces (combined slip)

%turn slip is neglected, so xi_i=1;
%all scaling factors lambda = 1;

S_hxalpha = p.r_hx1; 

alpha_s = alpha + S_hxalpha;

B_xalpha = p.r_bx1*cos(atan(p.r_bx2*kappa));
C_xalpha = p.r_cx1;
E_xalpha = p.r_ex1;
D_xalpha = F0_x/(cos(C_xalpha*atan(B_xalpha*S_hxalpha - E_xalpha*(B_xalpha*S_hxalpha - atan(B_xalpha*S_hxalpha)))));

%magic tire formula
F_x = D_xalpha*cos(C_xalpha*atan(B_xalpha*alpha_s - E_xalpha*(B_xalpha*alpha_s - atan(B_xalpha*alpha_s))));