function n_state = tran_function(prev_state, controller_out, robot_params, time)
    % prev_state is a 1x3 vector stating the X and Y positon and the
    % rotation theta around the Z axis.
    % controller_out is a 1x2 vector stating the linear and angular
    % velocities given by the controller to reach the next waypoint.
    % robot_params is a 1x2 vector stating the wheel radius and wheel
    % separation on the axle of the robot.
    % time is a 1x2 vector stating the initial and ending computation time
    % for the kinematic model of the robot.
    
    wheel_r = robot_params(1);
    wheel_sep = robot_params(2);
    kinematicModel = differentialDriveKinematics("WheelRadius", wheel_r, ...
                                                 "TrackWidth", wheel_sep, ...
                                                 "VehicleInputs", ...
                                                 "VehicleSpeedHeadingRate");
    tspan = time(1):time(2)/5:time(1)+time(2);

    [t, y] = ode45(@(t,y)derivative(kinematicModel, y, controller_out), tspan, prev_state);
    n_state = [y(end, 1); y(end, 2); y(end, 3)];
end