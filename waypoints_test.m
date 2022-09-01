%% Waypoints maneuvers test
utility_functions.setup_environment("192.168.1.97", "192.168.1.126");
obj = agent("My_Agent", 5, [0, 0, 0], [-1, 1, -pi/4], [0 0], 10, 20, 0.105);
obj = obj.set_factory_setup(0.06, 0.24);

x_lims = [-2, 2];
y_lims = [3, 7];

ag_over = agentOverviewer;
ag_over = ag_over.set_no_gps_areas(x_lims, y_lims);

ag_over = ag_over.register_agent(obj);
obj = obj.set_overviewer(ag_over);

obj = obj.ros_connect();

waypoints = [0, 0;
             5, 0;
             5, 5;
             0, 5];
for k=1:size(waypoints, 1)
    [new_x, new_y] = utility_functions.H_trans_2D(obj.absolute_pose(1:2), ...
                                                   waypoints(k, :), obj.absolute_pose(3));
    waypoints(k, :) = [new_x; new_y];
end

gps_lims_points = [[2, 3]; [2, 7]; [-2, 7]; [-2, 3]];
for k=1:size(gps_lims_points, 2)
     [g_x, g_y] = utility_functions.H_trans_2D(obj.absolute_pose(1:2), ...
                                               gps_lims_points(k, :), obj.absolute_pose(3));
    gps_lims_points(k, :) = [g_x, g_y];
end

path.States = waypoints; % - obj.current_relative_pose(1:2);

% Controller routine
controller = controllerPurePursuit;
controller.Waypoints = path.States(:, 1:2);

controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 10;
controller.LookaheadDistance = 2;

current_pose = obj.current_relative_pose;
goal_pose = path.States(end, :);
disp(["State start: ", current_pose])
disp(["State end: ", goal_pose])

goal_th = 0.15;

dist = utility_functions.euclidean_2D(current_pose, goal_pose);

initial_pose = obj.get_current_pose("XYR"); % Absolute pose

% kinematicModel = differentialDriveKinematics("WheelRadius", 0.06, ...
%                                              "TrackWidth", 0.24, ...
%                                              "VehicleInputs", "VehicleSpeedHeadingRate");

kf = trackingEKF(@tran_function, @measurement_function, current_pose, ...
                 "MeasurementNoise", 0.2, ...
                 'ProcessNoise', diag([50,50,1]));

true_poses = [current_pose(1:2)];
diff_drive = [];
predicted_poses = [];
track_err = [];
traj = [];

while dist > goal_th
    [v_l, v_a] = obj.get_current_vels();
    [new_v, new_a] = controller(current_pose);
    obj = obj.set_velocity([new_v 0 0], [0, 0, new_a]);   
    start_time = datetime('now');
   
    new_pose = obj.get_current_pose("XYR"); % Absolute pose
    
    if obj.overviewer.is_gps_available(new_pose(1:2))
        disp("GPS ON")

        [new_x, new_y] = utility_functions.H_trans_2D(initial_pose(1:2), ...
                                                      new_pose(1:2), ...
                                                      obj.absolute_pose(3));
        
        [wr, ws] = obj.get_factory_setup();
        
        theta = new_pose(3) - initial_pose(3);
        current_pose = [new_x, new_y, theta];
        % disp(["Curr pose", current_pose])


        now_time = datetime('now');
        est_pose = kf.predict([new_v, new_a], [wr, ws], [start_time, now_time]);
        
        x_corr = kf.correct(current_pose, [new_v, new_a], [wr, ws], [start_time, now_time]);
        
    else
        disp("GPS OFF - KF in use");
        now_time = datetime('now');
        est_pose = kf.predict([new_v, new_a], [wr, ws], [start_time, now_time]);
        % disp(["Estimate pose", est_pose])

        current_pose = est_pose;
    end

    traj = [traj; current_pose(1:2)];
    dist = utility_functions.euclidean_2D(current_pose, ...
                                          goal_pose);
    % disp(["Current_pose", current_pose])

    % initial_pose = new_pose;
    disp(["Current pose: ", current_pose, "Dist: ", dist])
    % disp(["current vels: ", obj.get_current_vels()]);
    % disp(["new_vels: ", new_v, new_a]);
end
obj.set_velocity();

%%
% track_err = true_poses(1:end-1, :) - predicted_poses;
% plot_err = zeros(size(track_err, 1), 1);
% for k=1:size(track_err, 1)
%     plot_err(k) = norm(track_err(k, :));
% end


f_1 = figure;
f_1.Position = [0 0 1000 1000];
plot(traj(:, 1), traj(:, 2), 'LineWidth', 2);
% plot(true_poses(:, 1), true_poses(:, 2), 'LineWidth', 2);
% hold on
% plot(predicted_poses(:, 1), predicted_poses(:, 2), 'LineWidth', 1.1);
hold on
plot(waypoints(:, 1), waypoints(:, 2), '^-', 'LineWidth', 2);
% hold on;
% plot(diff_drive(:, 1), diff_drive(:, 2));
% no_gps_area = polyshape(gps_lims_points(:, 1), gps_lims_points(:, 2));
% plot(no_gps_area, "FaceColor", 'g', "FaceAlpha", 0.1);
title("Trajectories: Real VS EKF")
hold off;
legend('Robot trajectory', 'Waypoints', 'Diff_drive', "Location", 'southoutside');

% f_2 = figure;
% f_2.Position = [0 0 1000 500];
% plot(plot_err, 'LineWidth', 2);
% hold on;
% plot(zeros(1, size(plot_err, 1)) + mean(plot_err), 'LineWidth', 2);
% title("Tracking Error");
% 
% saveas(f_1, "Report_images/Traj_KF", "png");
% saveas(f_2, "Report_images/KF_Error", "png");
