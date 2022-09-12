%  EKF

function next_state = EKF(prev_state, controller_out, robot_parameters, ...
                          elapsed_time, measurement)
if ~exist('measurement', 'var')
    measurement = zeros(3);
end

kinematicModel = differentialDriveKinematics("WheelRadius", robot_parameters(1), ...
                                             "TrackWidth", robot_parameters(2), ...
                                             "VehicleInputs", ...
                                             "VehicleSpeedHeadingRate");
tspan = 0:elapsed_time:elapsed_time;

[t, y] = ode45(@(t,y)derivative(kinematicModel, y, controller_out), ...
                        tspan, prev_state);
next_state = [y(end, 1); y(end, 2); y(end, 3)];

model_derivative = diag(derivative(kinematicModel, next_state, controller_out));
model_derivative = model_derivative/sum(model_derivative, 'all');

H = eye(3); % Jacobian of measurement model
Q = zeros(3);  % Process noise covariance
Pi = eye(3); % Priori estimate of covariance
R = zeros(3); % Measurement noise covariance

state_cov = (model_derivative * Pi * model_derivative') + Q;
K_gain = state_cov * H' / ((H * state_cov * H') + R);
next_state = state_cov * K_gain * (measurement - measurement_function(prev_state))';
next_state = next_state';
end