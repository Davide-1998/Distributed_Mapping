% Measurement function
% state is a 1x3 vector stating the X and Y positon and the
% rotation theta around the Z axis.
% controller_out is a 1x2 vector stating the linear and angular
% velocities given by the controller to reach the next waypoint.
% robot_params is a 1x2 vector stating the wheel radius and wheel
% separation on the axle of the robot.
% time is a 1x2 vector stating the initial and ending computation time
% for the kinematic model of the robot.

function meas = measurement_function(state, controller_out, robot_params, time)
    meas = state;
end