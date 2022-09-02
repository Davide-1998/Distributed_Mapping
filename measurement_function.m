% Measurement function
% Input state has [current_pose, controller_outputs, ...
%                  [wheel_radius, wheel_separation], ...
%                  start_time, end_time]
% Input state size is [1, 5]

function meas = measurement_function(state, controller_out, robot_params, time)
    meas = state;
end