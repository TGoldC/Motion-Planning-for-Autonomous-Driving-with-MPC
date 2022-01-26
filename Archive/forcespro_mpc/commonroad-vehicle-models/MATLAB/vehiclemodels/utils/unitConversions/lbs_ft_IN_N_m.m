function post_val = lbs_ft_IN_N_m(prev_val)
% original: [lbs/ft]
% new: [N/m]

% 1lbs is 0.45359237 kg
% 1kg is around 9.81 N assuming being close to sea level
% 1ft is 0.3048 m

post_val = 0.45359237*9.81/0.3048*prev_val;