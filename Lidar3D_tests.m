%% Offline 3D Lidar scan test and merging

%% Just show data
utility_functions.setup_environment("192.168.1.97", "192.168.1.126");
obj = agent("My_Agent_1", 15, [-4 -6 3.14], [-4 -6 3.14], 10, 20, 0.135);
% obj = agent("My_Agent_2", 15, [-2 6.4 0], [-2 6.4 0], 15, 20, 0.135);
obj = obj.set_lidar_parameters(15, 0.135, 360, 16);
obj = obj.initialize_err_KF([0.03, 0.03, 0.03], [0, 0, 0]);

obj = obj.ros_connect();

obj = obj.set_velocity([1, 0, 0]);

poses_history = zeros(11, 3);

poses_history(1, :) = [obj.current_est_pose(1:2), 0];
for k=1:5
    pause(1)
    if k==2
         obj = obj.set_velocity([1, 0, 0], [0, 0, -4]);
    end
%     if k==10
%         obj = obj.set_velocity([1, 0, 0], [0, 0, 0]);
%     end
    obj = obj.set_current_est_pose(obj.get_current_pose('XYR'));

    obj = obj.compute_map();
end
obj.set_velocity();

obj.show_map();

rosshutdown;

%%
ag_1_player = pcplayer([-40, 40], [-40, 40], [-40, 40]);
view(ag_1_player, cell2mat(obj.global_3D_point_cloud{1, 5}));

%% Online data sharing -> This used for report images
utility_functions.setup_environment("192.168.1.97", "192.168.1.126");
ag_over = agentOverviewer;

ag_1 = agent("My_Agent_1", 15, [-4 -6 3.14], [-4 -6 3.14], 15, 20, 0.135);
ag_2 = agent("My_Agent_2", 15, [-2 6.4 0], [-2 6.4 0], 15, 20, 0.135);

ag_1 = ag_1.ros_connect();
ag_2 = ag_2.ros_connect();

ag_over = ag_over.register_agent(ag_1);
ag_1 = ag_1.set_overviewer(ag_over);

ag_over = ag_over.register_agent(ag_2);
ag_2 = ag_2.set_overviewer(ag_over);

disp("Processing agent_1")
ag_1 = ag_1.compute_map();

disp("Processing agent_2")
ag_2 = ag_2.compute_map();

occ_1 = ag_1.global_occupancy;
occ_2 = ag_2.global_occupancy;

builder_1 = ag_1.slam_builder;
builder_2 = ag_2.slam_builder;

s1 = ag_1.local_scans;
s2 = ag_2.local_scans;

s2{end+1} = s1{1};
poses = [ag_2.current_est_pose; ag_1.current_est_pose];
merged_occ = buildMap(s2, poses, 10, 15);

singles_f = figure;
singles_f.Position = [0 0 1800 1000];
tiledlayout(1, 2);
nexttile
show(occ_1);
title("Occupancy agent 1", "FontSize", 18);
nexttile
show(occ_2);
title("Occupancy agent 2", "FontSize", 18);
saveas(singles_f, "Report_images/Single_occupancies", "png");

merged_f = figure;
merged_f.Position = [0 0 1800 1000];
show(merged_occ)
title("Merged scans", "FontSize", 18);
hold on
plot([-4, -2], [-6, 6.4], 'x r');
saveas(merged_f, "Report_images/Merged_occupancies", "png");

% c_test = pointCloud(ag_2.local_cloud);
% viewer = pcplayer(c_test.XLimits, c_test.YLimits, c_test.ZLimits);
% view(viewer, c_test);

% ag_2.show_map()
%%
cloud_ag1 = cell2mat(ag_1.global_3D_point_cloud{end});
cloud_ag2 = cell2mat(ag_2.global_3D_point_cloud{end});
% c_1 = zeros(n_orig, 3) + [0, 0, 255];
% c_2 = zeros(n_el_ag_1, 3) + [255, 0, 0];
% colors = [c_1; c_2];

% ag_1_player = pcplayer([-20, 20], [-20, 20], [-20, 20]);
% view(ag_1_player, cloud_ag1);

% ag_2_player = pcplayer([-20, 20], [-20, 20], [-20, 20]);
% view(ag_2_player, cell2mat(ag_2.global_3D_point_cloud{end}), colors);
ag_2.show_map();
