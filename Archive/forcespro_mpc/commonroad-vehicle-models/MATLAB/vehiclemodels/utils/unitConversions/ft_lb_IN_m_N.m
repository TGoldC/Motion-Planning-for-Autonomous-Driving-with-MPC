function post_val = ft_lb_IN_m_N(prev_val)
%original: [ft/lb]
%new: [m/N]

% 1lb is 4.4482216152605 N
% 1ft is 0.3048 m

post_val = 0.3048/4.4482216152605*prev_val;