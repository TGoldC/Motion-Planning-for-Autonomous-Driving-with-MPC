function post_val = rad_ft_lb_IN_rad_sec2_kg_m2(prev_val)

%original: [rad/(ft lb)]
%new: [rad/(N m)] = [rad s^2/(kg m^2)]

% 1lb is 4.4482216152605 N
% 1ft is 0.3048 m

post_val = 1/(4.4482216152605*0.3048)*prev_val;