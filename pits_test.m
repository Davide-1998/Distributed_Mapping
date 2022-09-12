% Pit approach test
utility_functions.setup_environment("192.168.1.97", "192.168.1.126");
ag_over = agentOverviewer;

ag_1 = agent("My_Agent_1", 15, [-4 -6 3.14], [-4 -6 3.14], 10, 20, 0.135);
ag_2 = agent("My_Agent_2", 15, [-2 6.4 0], [-2 6.4 0], 10, 20, 0.135);

ag_1 = ag_1.ros_connect();
ag_2 = ag_2.ros_connect();

ag_over = ag_over.register_agent(ag_1);
ag_1 = ag_1.set_overviewer(ag_over);

ag_over = ag_over.register_agent(ag_2);
ag_2 = ag_2.set_overviewer(ag_over);

data = receive(ag_1.scanned_data_sub, 3);

%%
data_in = [data.Points(:).X; data.Points(:).Y; data.Points(:).Z];
[temp_cloud, ground, pits] = utility_functions.pre_process_cloud3D(data, ...
                                                                   ag_1);
local_cloud = [temp_cloud; ground; pits];

% Make occupancy map
full_occupancy = [temp_cloud; pits];
[ranges, angles] = utility_functions.cartesian_to_polar_2D(full_occupancy);
scan_in = lidarScan(ranges, angles);

occMap = buildMap({scan_in}, ag_1.current_est_pose, 10, 10);
tran_pits = [];
tran_pits(:, 1:2) = utility_functions.H_trans_2D_new(ag_1.current_est_pose(1:2), ...
                                                     pits(:, 1:2), ...
                                                     ag_1.current_est_pose(3));
setOccupancy(occMap, tran_pits(:, 1:2), 1);

local_cloud(:, 1:2) = utility_functions.H_trans_2D_new(ag_1.current_est_pose(1:2), ...
                                                       local_cloud(:, 1:2), ...
                                                       ag_1.current_est_pose(3));

f = figure;
f.Position = [0, 0, 1000, 1000];
show(occMap);
title("Occupancy Map", "FontSize", 18);
% hold on
% plot(tran_pits(:, 1), tran_pits(:, 2), '. r');
saveas(f, "Report_images/Occupancy_pits", "png");

colors = [zeros(size(temp_cloud, 1), 3) + [100, 100, 100];
          zeros(size(ground, 1), 3) + [0, 255, 0];
          zeros(size(pits, 1), 3) + [255, 0, 0]];

c_test = pointCloud(local_cloud, "Color", colors);
viewer = pcplayer(c_test.XLimits, c_test.YLimits, c_test.ZLimits);
view(viewer, c_test);
