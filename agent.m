% Agent Script
% The agent acceleration parameter has been estimated empirically counting
% the time necessay to reach the desired velocity starting from a still
% position.
% The test have been performed in the world of tree_corridor_2_agents.
% and 7 acquisition were made. All data were round to 3 significant digits.
% Best time: 5.505
% Worst time: 6.108
% Mean value over 7 acquisitions: 5.7431
% Acceleration estimated using the equation V = V0 + a*t st. V0 = 0 & V = 1
% Acceleration found: a = 1/5.7431 = 0.1741 = 0.174

classdef agent
    properties
        id = "Agent_default";
        current_relative_pose = [0; 0; 0];  % x y rotation
        prev_relative_pose = [0; 0; 0];
        current_linear_vel = 0;
        absolute_pose = [0, 0, 0];
        current_angular_vel = 0;
        lidar_range = 0;
        max_lidar_map_range = 0;
        communication_range = 0;
        scanned_data_sub;
        odometry_sub;
        vel_command;
        
        lidar_origin_height = 0;

        % SLAM Attributes
        map_cloud = []; % Cloud 3D representation of map
        poses_graph = poseGraph3D;
        prev_tf;
        
        % Motion model Parameters
        estimated_acceleration = 0.1741;
        last_set_vel_t = 0;
        vel_desired = 0

        known_agents = [];
        overviewer;

        no_scans = true;
        % current_map = [];
        ros_conn = false;
        slam_builder;
    end
    
    methods
        function obj = set_lidar_origin_height(obj, h)
            obj.lidar_origin_height = h;
        end
        function obj = set_max_lidar_map(obj, value)
            obj.max_lidar_map_range = value;
        end
        function obj = set_slam_builder(obj, slam_b)
            obj.slam_builder = slam_b;
        end
        function obj = set_overviewer(obj, ov)
            obj.overviewer = ov;

        end
        function [linear, angular] = get_current_vels(obj)
            odomData = receive(obj.odometry_sub, 3);
            linear = [odomData.Twist.Twist.Linear.X, ...
                      odomData.Twist.Twist.Linear.Y, ...
                      odomData.Twist.Twist.Linear.Z];
            angular = [odomData.Twist.Twist.Angular.X, ...
                       odomData.Twist.Twist.Angular.Y, ...
                       odomData.Twist.Twist.Angular.Z];
        end
        function pose = get_current_pose(obj)
            odomData = receive(obj.odometry_sub, 3);
            position = odomData.Pose.Pose.Position;
            pose = [position.X, position.Y, position.Z];
        end
        function orientation = get_current_orientation(obj)
            odomData = receive(obj.odometry_sub, 3);
            or_ = odomData.Pose.Pose.Orientation;
            quat_orientation = [or_.X, or_.Y, or_.Z, or_.W];
            orientation = quat2eul(quat_orientation, 'ZYX');
        end
        function obj = agent(id_, scan_range, current_pose, ...
                             absolute_p, current_vels, comm_range, ...
                             max_lidar_map_range_, lidar_orig_h, est_acc)
            if ~exist("id_", 'var')
                id_ = "Default_id";
            end
            if ~exist("est_acc", 'var')
                est_acc = 0.1741;
            end
            if ~exist("scan_range", 'var')
                scan_range = 1;  % meters
            end
            if ~exist("comm_range", 'var')
                comm_range = scan_range; 
            end
            if ~exist("current_pose", 'var')
                current_pose = [0, 0, 0];
            end
            if ~exist("absolute_p", 'var')
                absolute_p = [0, 0, 0];
            end
            if ~exist("current_vels", 'var')
                current_vels = [0, 0];
            if ~exist("lidar_orig_h", 'var')
                lidar_orig_h = 0.105;
            end
            elseif numel(current_vels) < 2
                msg = "Expected 2 values for linear and angular\n" + ...
                      "Only the linear velocity will be set";
                disp(msg);
                current_vels = [current_vels, 0];
            end
            if ~exist("max_lidar_map_range_", 'var')
                max_lidar_map_range_ = scan_range;
            end

            obj.id = id_;
            obj.lidar_range = scan_range;
            obj.communication_range = comm_range;
            obj.max_lidar_map_range = max_lidar_map_range_;
            obj.current_relative_pose = current_pose;
            obj.absolute_pose = absolute_p;
            obj.current_linear_vel = current_vels(1);
            obj.current_angular_vel = current_vels(2);
            obj.lidar_origin_height = lidar_orig_h;
            obj.estimated_acceleration = est_acc;
        end

        function obj = set_id(obj, new_id)
            obj.id = new_id;
        end

        function obj = set_velocity(obj, vel_linear, vel_angular)
            if ~obj.ros_conn
                disp("Connect to ROS before sending messages")
                return
            end
            if ~exist("vel_linear", "var")
                vel_linear = [0 0 0];
            end
            if ~exist("vel_angular", "var")
                vel_angular = [0 0 0];
            end

            msg = rosmessage(obj.vel_command);

            msg.Linear.X = vel_linear(1);
            msg.Linear.Y = vel_linear(2);
            msg.Linear.Z = vel_linear(3);

            msg.Angular.X = vel_angular(1);
            msg.Angular.Y = vel_angular(2);
            msg.Angular.Z = vel_angular(3);

            obj.current_linear_vel = vel_linear(1);
            obj.current_angular_vel = vel_angular(3);

            send(obj.vel_command, msg);
            obj.last_set_vel_t = now;
        end
        function obj = ros_connect(obj, agent_id)
            if ~exist('agent_id', 'var')
                agent_id = obj.id;
            end
            scan_topic = strcat('/', agent_id, '/ScanResults');
            scan_sub = utility_functions.subscriber_to_topic(scan_topic);
            
            od_topic = strcat('/', agent_id, '/odometry');
            od_sub = utility_functions.subscriber_to_topic(od_topic);

            vel_topic = strcat('/', agent_id, '/vel');
            % LidarData = receive(scan_sub, 3);

            obj.scanned_data_sub = scan_sub;
            obj.odometry_sub = od_sub;
            obj.vel_command = rospublisher(vel_topic);
            obj.ros_conn = true;
            obj.slam_builder = lidarSLAM(10, ...
                                         obj.max_lidar_map_range);
            obj.slam_builder.LoopClosureThreshold = 210;  
            obj.slam_builder.LoopClosureSearchRadius = 5;
        end
        function obj = refine_3D_map(obj)
            prevTF = [];
            for k=1:size(obj.map_cloud, 1)
                pcl_wogrd = obj.map_cloud(k);
                % pcl_wogrd = pointCloud(local_cloud);
                % Data downsampling for speed
                pcl_wogrd_sampled = pcdownsample(pcl_wogrd, 'random', 0.25);

                % Registering point cloud
                scanAccepted = 1;
                count = size(obj.map_cloud, 1);
                if count == 0
                    tform = [];
                else
                    if count == 1
                        moving = obj.map_cloud(count);
                        tform = pcregisterndt(pcl_wogrd_sampled, moving, 2.5);
                    else
                        tform = pcregisterndt(pcl_wogrd_sampled, obj.map_cloud(count), 2.5, ...
                            'InitialTransform', prevTF);
                    end

                    relPose = [tform2trvec(tform.T') tform2quat(tform.T')];
                    addRelativePose(obj.poses_graph,relPose);
                    %{
                    if sqrt(norm(relPose(1:3))) > distanceMovedThreshold
                        addRelativePose(pGraph,relPose);

                    else
                        scanAccepted = 0;
                    end
                    %}
                end

                if scanAccepted == 1
                    count = count + 1;

                    % obj.map_cloud = [obj.map_cloud; pcl_wogrd_sampled];

                    % pose graph optimization
                    % if mod(count, 5) == 0 %size(obj.map_cloud, 1) >= 2
                    obj.poses_graph = optimizePoseGraph(obj.poses_graph);
                    % nd
                    prevTF = tform;
                end
            end
        end

        function obj = compute_map(obj, rotation_adjust)
            if obj.ros_conn == false
                disp("Not connected to ROS");
                return;
            else
                LidarData = receive(obj.scanned_data_sub, 3);
                % array_of_collisions = LidarData.Points;
                local_cloud = utility_functions.pre_process_cloud3D(LidarData, ...
                                                                    obj.lidar_range);
                obj.map_cloud = [obj.map_cloud; pointCloud(local_cloud)];
                
                [ranges, angles] = utility_functions.cartesian_to_polar_2D(local_cloud);

                %{
                local_cloud = local_cloud + obj.current_relative_pose(1:3);
                disp(obj.current_relative_pose)
                obj.map_cloud = [obj.map_cloud; local_cloud];

                occupied_2D = [];
                ranges = [];
                angles = [];
                
                for k=1:size(local_cloud, 1)
                    if local_cloud(k, 3) < 1.1*obj.lidar_origin_height
                        occupied_2D(k, :) = local_cloud(k, 1:2);
                        x_local = local_cloud(k, 1);
                        y_local = local_cloud(k, 2);
                        theta = atan2(y_local, x_local);
                        rho = norm([x_local, y_local]);
                        ranges = [ranges, rho];
                        angles = [angles, theta];
                    end
                end
                %}

                % res_step = LidarData.AngleIncrement;
                % max_range = LidarData.RangeMax;
                
                % Due to simulated sensor, data acquisition may vary
                % Need a correction parameter
                %{
                if ~exist('rotation_adjust', 'var')
                    rotation_adjust = 0;
                end

                ranges = zeros(numel(array_of_collisions, 1));
                angles = zeros(numel(array_of_collisions), 1);
                for k = 1:numel(array_of_collisions)
                        theta = rotation_adjust + (k-1)*res_step;
                        angles(k, 1) = theta;
                        rho = array_of_collisions(k);
                        ranges(k, 1) = rho;
                end
                %}

                % Check if any agent is nearby
                [nearby_ranges, nearby_angles] = utility_functions.agent_data_to_local_system(obj);
                
                % Update with the data received by the other agents
                if numel(nearby_ranges) > 0 && numel(nearby_angles) > 0
                    ranges = [ranges; nearby_ranges];
                    angles = [angles; nearby_angles];
                end

                % Update the scan results
                scan_in = lidarScan(ranges, angles);
                if obj.no_scans == true
                    obj.no_scans = false;
                end
                addScan(obj.slam_builder, scan_in, obj.current_relative_pose(1:3));

                [scans, poses] = scansAndPoses(obj.slam_builder);
            end

        end
        function obj = set_current_relative_pose(obj, pose)
            obj.prev_relative_pose = obj.current_relative_pose;
            obj.current_relative_pose = pose;
            obj.absolute_pose = obj.absolute_pose + pose;
            if ~isempty(obj.overviewer)
                obj.overviewer.registered_agents(obj.id) = obj;
            end
        end
        function [pthObj, solnInfo] = compute_roadmap(obj)
            % Take the readings
            [scans, poses] = scansAndPoses(obj.slam_builder);
            
            % Build occupancy map
            occMap = buildMap(scans, poses, 10, obj.max_lidar_map_range);
            inflate(occMap, 0.1);
            occMap.FreeThreshold = 0.50;
            
            % Generate search space and node validator
            now_pose = obj.current_relative_pose;
            
            low_bound_x = now_pose(1); % - obj.slam_builder.MaxLidarRange;
            high_bound_x = now_pose(1) + obj.slam_builder.MaxLidarRange;
            low_bound_y = now_pose(2); % - obj.slam_builder.MaxLidarRange;
            high_bound_y = now_pose(2) + obj.slam_builder.MaxLidarRange;
            low_bound_rot = now_pose(3) - (pi/2);
            high_bound_rot = now_pose(3) + (pi/2);

            space = stateSpaceSE2([low_bound_x high_bound_x; ...
                                   low_bound_y high_bound_y; ...
                                   low_bound_rot high_bound_rot]);
            validator = validatorOccupancyMap(space);
            validator.Map = occMap;

            % set up planner
            planner = plannerRRTStar(space, validator);
            planner.BallRadiusConstant = 0.5; % same?
            planner.MaxNumTreeNodes = 50;  % Make it adaptive?
            planner.MaxConnectionDistance = 1; % same?
            planner.ContinueAfterGoalReached = true;

            % Randomly sample next location to look up
            next_state = sampleUniform(space);
            while ~validator.isStateValid(next_state)
                next_state = sampleUniform(space);
            end

            % Find path
            start_state = now_pose;
            while ~validator.isStateValid(start_state)
                start_state = sampleGaussian(space, start_state, [1 1 1], 1);
            end
            
            rng(100, 'twister')
            [pthObj, solnInfo] = plan(planner, start_state, next_state);

            
            figure;
            show(occMap);
            hold on;
            plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-');
            hold on;
            plot(pthObj.States(:, 1), pthObj.States(:, 2), '.-');
            
        end
        function obj = execute_maneuvers(obj, path)
            % A path input is made of X Y and Rotation
            if numel(path.States) == 0
                return;
            end
            controller = controllerPurePursuit;
            controller.Waypoints = path.States(:, 1:2);

            controller.DesiredLinearVelocity = 0.5;
            controller.MaxAngularVelocity = 10;
            % controller.LookaheadDistance = 0.3;

            current_pose = obj.current_relative_pose; % path.States(1, :);
            goal_pose = path.States(end, :);

            goal_th = 1;

            dist = utility_functions.euclidean_2D(current_pose, goal_pose);

            initial_position = obj.get_current_pose();

            [lin_v, ang_v] = obj.get_current_vels();
            prev_l_v = lin_v;
            prev_a_v = ang_v;
            while dist > goal_th
                [new_v, new_a] = controller(current_pose);
                obj = obj.set_velocity([new_v 0 0], [0, 0, new_a]);
                % pause(sample_time);

                new_position = obj.get_current_pose();
                eul = obj.get_current_orientation();

                delta_position = new_position(1:2) - initial_position(1:2);

                delta_pose = [delta_position(1), delta_position(2), 0];
                current_pose = current_pose + delta_pose;
                current_pose(3) = eul(3);

                dist = utility_functions.euclidean_2D(current_pose, ...
                                                      goal_pose);
                % Movement estimation
                % elapsed_time = now - obj.last_set_vel_t;
                % space_l = (pre_l_v(1) * elapsed_time) + ...
                %           (obj.estimated_accelleration*(elasped_time)^2);
                initial_position = new_position;
                disp([current_pose, "Dist: ", dist])
            end
            obj = obj.set_current_relative_pose(current_pose);
        end

        function [] = do_slam(obj, iterations, correction_angle)
            if ~exist("iterations", "var")
                iterations = 100;
            end

            if ~exist("correction_angle", "var")
                correction_angle = 0;
            end

            k = 1;
            while k < iterations
                obj = obj.compute_map(correction_angle);
                [path, roadmap] = obj.compute_roadmap();
                obj = obj.execute_maneuvers(path);

                k = k+1;
            end
            % [scans, poses] = scansAndPoses(obj.slam_builder);
            % save("slam_scans_" + obj.id + ".mat", 'scans');
            % save("slam_poses_" + obj.id + ".mat", 'poses');
            obj = obj.set_velocity([0 0 0]);  % Stop agent after slamming
            obj.show_map();
        end
        function [] = show_map(obj)
            figure;
            title("Occupancy Map");
            [scans, poses] = scansAndPoses(obj.slam_builder);
            occMap = buildMap(scans, poses, 10, obj.max_lidar_map_range);
            show(occMap);

            obj = obj.refine_3D_map();
            
            player = pcplayer([-20, +20], [-20, +20], [-10, 10]);
            my_cloud_ag = [];
            nodesPositions = nodes(obj.poses_graph);
            for k=1:numel(obj.map_cloud)
                t_cloud = obj.map_cloud(k).Location;
                t_pose_angle = quat2eul(nodesPositions(k, 4:end));
                t_pose = [nodesPositions(k, 1:2), t_pose_angle(3)];
                t_cloud = t_cloud + t_pose;
                my_cloud_ag = [my_cloud_ag; t_cloud];
            end
            while isOpen(player) 
                view(player, my_cloud_ag);           
             end 
        end
    end
end