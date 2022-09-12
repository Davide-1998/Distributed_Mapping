% Exploration script
%% Agents motion
% while cycle do:
% get data and clean them
% update point cloud and occupancy map
% compute waypoints
% compute next goal
% compute path waypoints
% make agent execute maneuvers
% after N iterations set velocity to 0 -> end of SLAM
% Keep doing for other agents

%% Make models files and copy to host machine (Only if needed)
for k=1:numel(ids)
    utility_functions.make_agent_model(ids(k));
end
utility_functions.copy_models_to_gazebo('~/.gazebo/models', ...
                                        "192.168.1.97", ...
                                        'davide');

%% Do things
% Set up environment ->  <ipaddr of ROS machine>, <ipaddr of Local Machine>
utility_functions.setup_environment("192.168.1.97", "192.168.1.126");
ids = ["My_Agent_1", "My_Agent_2"];

% Make agents
custom_ab_poses = containers.Map;
custom_ab_poses("My_Agent_1") = [2 0 3.14];
custom_ab_poses("My_Agent_2") = [-2 6.4 0];

agents = [];
ag_over = agentOverviewer;

for k=1:numel(ids)
    c_ab_pose = [0 0 0];
    if isKey(custom_ab_poses, ids(k))
        c_ab_pose = custom_ab_poses(ids(k));
    end
    temp = agent(ids(k), 10, c_ab_pose, c_ab_pose, 10, 20, 0.135);
    temp = temp.ros_connect();
    ag_over = ag_over.register_agent(temp);
    temp = temp.set_overviewer(ag_over);
    agents = [agents, temp];
end

% Perform exploration
disp("Exploring");
agents = fliplr(agents);

% Ros messages with parpool are not supported!
% parpool("local", size(agents, 2));
% parfor k=1:size(agents, 2)
% for k=1:numel(agents)*2

    disp(agents(2).id);
%     agents(2) = agents(2).compute_map();
%     agents(2).show_map();
    agents(2).do_slam(2);

% end
% delete(gcp('nocreate'));
