%% Waypoints maneuvers test
% Use this code to test the efficiency of the EKF filter in the agent

utility_functions.setup_environment("192.168.1.97", "192.168.1.126");
obj = agent("My_Agent", 5, [0, 0, 0], [-1, 1, -pi/4], 10, 20, 0.105);
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

kf = trackingEKF(@tran_function, @measurement_function, current_pose, ...
                 "MeasurementNoise", 0.1, ...
                 'ProcessNoise', diag([1,1,1]));

true_poses = [current_pose(1:2)];
kf_no_gps = [];
corr_kf = [];
est_kf = [];

while dist > goal_th
    [v_l, v_a] = obj.get_current_vels();
    [new_v, new_a] = controller(current_pose);
    obj = obj.set_velocity([new_v 0 0], [0, 0, new_a]);   
    start_time = datetime('now');
    pause(0.1);
   
    new_pose = obj.get_current_pose("XYR"); % Absolute pose
    [new_x, new_y] = utility_functions.H_trans_2D(initial_pose(1:2), ...
                                                      new_pose(1:2), ...
                                                      obj.absolute_pose(3));

    if obj.overviewer.is_gps_available(new_pose(1:2))
        disp("GPS ON")

%         [new_x, new_y] = utility_functions.H_trans_2D(initial_pose(1:2), ...
%                                                       new_pose(1:2), ...
%                                                       obj.absolute_pose(3));
        
        [wr, ws] = obj.get_factory_setup();
        
        theta = new_pose(3) - initial_pose(3);
        current_pose = [new_x, new_y, theta];
        % disp(["Curr pose", current_pose])


        now_time = datetime('now');
        elapsed_time = seconds(now_time - start_time);
        est_pose = kf.predict([new_v, new_a], [wr, ws], [0, elapsed_time]);
        est_kf = [est_kf; est_pose(1:2)];

        x_corr = kf.correct(current_pose, [new_v, new_a], [wr, ws], [0, elapsed_time]);
        corr_kf = [corr_kf; x_corr(1:2)];
        
    else
        disp("GPS OFF - KF in use");
        now_time = datetime('now');
        elpsed_time = seconds(now_time - start_time);
        est_pose = kf.predict([new_v, new_a], [wr, ws], [0, elapsed_time]);
        % disp(["Estimate pose", est_pose])
        kf_no_gps = [kf_no_gps; est_pose(1:2)];
        current_pose = est_pose;
    end

    true_poses = [true_poses; [new_x, new_y]];
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
plot(true_poses(:, 1), true_poses(:, 2), 'LineWidth', 2, "Color", 'g');
hold on
plot(est_kf(:, 1), est_kf(:, 2), "x", "Color", 'b', "LineWidth", 2);
hold on
plot(corr_kf(:, 1), corr_kf(:, 2), "o", "Color", 'b', "LineWidth", 2);
hold on
plot(waypoints(:, 1), waypoints(:, 2), '^-', 'LineWidth', 2);
hold on;
plot(kf_no_gps(:, 1), kf_no_gps(:, 2), "--", "Color", 'r', "LineWidth", 2);
% hold on
% no_gps_area = polyshape(gps_lims_points(1:3, 1), gps_lims_points(1:3, 2));
% plot(no_gps_area, "FaceColor", 'g', "FaceAlpha", 0.1);
title("Trajectories: Real VS EKF")
hold off;
legend('Robot trajectory', 'Predicted EKF', 'Corrected EKF', ...
        'Waypoints', 'EKF no GPS', "Location", 'southoutside');

% f_2 = figure;
% f_2.Position = [0 0 1000 500];
% plot(plot_err, 'LineWidth', 2);
% hold on;
% plot(zeros(1, size(plot_err, 1)) + mean(plot_err), 'LineWidth', 2);
% title("Tracking Error");
% 
% saveas(f_1, "Report_images/Traj_KF", "png");
% saveas(f_2, "Report_images/KF_Error", "png");
