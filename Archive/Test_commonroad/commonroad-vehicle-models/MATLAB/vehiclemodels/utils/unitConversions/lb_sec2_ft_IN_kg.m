function post_val = lb_sec2_ft_IN_kg(prev_val)

% 1lb is 4.4482216152605 N
% 1ft is 0.3048 m

% lb.sec^2/ft --> kg

post_val = 4.4482216152605/0.3048*prev_val;