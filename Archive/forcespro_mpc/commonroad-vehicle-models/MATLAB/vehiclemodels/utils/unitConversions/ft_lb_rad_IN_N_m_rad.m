function post_val = ft_lb_rad_IN_N_m_rad(prev_val)
%original: [lb ft/rad]
%new: [N m/rad]

% 1lb is 4.4482216152605 N
% 1ft is 0.3048 m

post_val = 4.4482216152605*0.3048*prev_val;