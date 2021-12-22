function post_val = rad_ft_IN_rad_m(prev_val)
%original: [rad/ft]
%new: [rad/m]

% 1ft is 0.3048 m

post_val = 1/0.3048*prev_val;