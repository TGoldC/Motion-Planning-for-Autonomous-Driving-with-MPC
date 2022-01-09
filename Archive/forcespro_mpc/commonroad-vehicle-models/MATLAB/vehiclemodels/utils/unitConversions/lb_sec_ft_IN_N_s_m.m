function post_val = lb_sec_ft_IN_N_s_m(prev_val)
% original: [lb sec/ft]
% new: [N sec/m]

% 1lb is 4.4482216152605 N
% 1ft is 0.3048 m

post_val = 4.4482216152605/0.3048*prev_val;