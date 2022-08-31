% Measurement function
% Input state has [now_pos_x, now_vel_x, new_vel_x, now_pos_y, ...
%                  now_vel_y, new_vel_y, theta, old_angular_vel, new_angular_vel, 
%                  elapsed_time]
% Input state size is [10, 1]

function meas = measurement_function(state)
    meas = state;
end