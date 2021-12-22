function post_val = lb_ft_sec2_IN_kg_m2(prev_val)

%[kg m^2] = [N m sec^2]

% 1lb is 4.4482216152605 N
% 1ft is 0.3048 m

% lb.ft.s^2 --> kg.m^2

post_val = 4.4482216152605*0.3048*prev_val;