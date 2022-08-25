%Agent Script
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
        vel_command;
        
        known_agents = [];
        overviewer;

        no_scans = true;
        % current_map = [];
        ros_conn = false;
        slam_builder;
    end
    
    methods
        function obj = set_max_lidar_map(obj, value)
            obj.max_lidar_map_range = value;
        end
        function obj = set_slam_builder(obj, slam_b)
            obj.slam_builder = slam_b;
        end
        function obj = set_overviewer(obj, ov)
            obj.overviewer = ov;

        end
        function obj = agent(id_, scan_range, current_pose, ...
                             absolute_p, current_vels, comm_range, ...
                             max_lidar_map_range_)
            if ~exist("id_", 'var')
                id_ = "Default_id";
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
        end

        function obj = set_id(obj, new_id)
            obj.id = new_id;
        end

        function set_velocity(obj, vel_linear, vel_angular)
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
        end
        function obj = ros_connect(obj, agent_id)
            if ~exist('agent_id', 'var')
                agent_id = obj.id;
            end
            scan_topic = strcat('/', agent_id, '/ScanResults');
            scan_sub = utility_functions.subscriber_to_topic(scan_topic);
            vel_topic = strcat('/', agent_id, '/vel');
            % LidarData = receive(scan_sub, 3);

            obj.scanned_data_sub = scan_sub;
            obj.vel_command = rospublisher(vel_topic);
            obj.ros_conn = true;
            obj.slam_builder = lidarSLAM(10, ...
                                         obj.max_lidar_map_range);
            obj.slam_builder.LoopClosureThreshold = 210;  
            obj.slam_builder.LoopClosureSearchRadius = 5;
        end
        function obj = compute_map(obj, rotation_adjust)
            if obj.ros_conn == false
                disp("Not connected to ROS");
               
            else

                LidarData = receive(obj.scanned_data_sub, 3);
                array_of_collisions = LidarData.Ranges;
                res_step = LidarData.AngleIncrement;
                % max_range = LidarData.RangeMax;
                
                % Due to simulated sensor, data acquisition may vary
                % Need a correction parameter
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
                addScan(obj.slam_builder, scan_in, obj.current_relative_pose);
                % disp("Scan Pose");
                % obj.current_relative_pose
                [scans, poses] = scansAndPoses(obj.slam_builder);
                
                % Update agent relative position
                % obj = obj.set_current_relative_pose(poses(end, :));
                % disp("Pose after estimation")
                % obj.current_relative_pose
            end

        end
        function obj = set_current_relative_pose(obj, pose)
            obj.prev_relative_pose = obj.current_relative_pose;
            obj.current_relative_pose = pose;
            obj.absolute_pose = obj.absolute_pose + pose;
            obj.overviewer.registered_agents(obj.id) = obj;
        end
        function [pthObj, solnInfo] = compute_roadmap(obj)
            % Take the readings
            [scans, poses] = scansAndPoses(obj.slam_builder);
            
            % Build occupancy map
            occMap = buildMap(scans, poses, 10, obj.max_lidar_map_range);
            inflate(occMap, 0.1);
            occMap.FreeThreshold = 0.49;
            
            % Generate search space and node validator
            now_pose = obj.current_relative_pose;
            low_bound_x = now_pose(1) - obj.slam_builder.MaxLidarRange;
            high_bound_x = now_pose(1) + obj.slam_builder.MaxLidarRange;
            low_bound_y = now_pose(2) - obj.slam_builder.MaxLidarRange;
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
            planner.BallRadiusConstant = 0.4; % same?
            planner.MaxNumTreeNodes = 100;  % Make it adaptive?
            planner.MaxConnectionDistance = 0.4; % same?
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

            %{
            figure;
            show(occMap);
            hold on;
            plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-');
            %}
        end
        function obj = execute_maneuvers(obj, path, roadmap)
            % A path input is made of X Y and Rotation
            if numel(path.States) == 0
                return;
            end
            controller = controllerPurePursuit;
            controller.Waypoints = path.States(:, 1:2);
            
            controller.DesiredLinearVelocity = 3;
            controller.MaxAngularVelocity = 1;
            % controller.LookaheadDistance = 1.2;

            
            current_pose = path.States(1, :);
            goal_pose = path.States(end, :);

            goal_th = 0.4;  % Same as maximum connection distance
            
            dist = utility_functions.euclidean_2D(current_pose, ...
                                                  goal_pose);
            
            k = 1;
            while dist > goal_th &&  k < numel(path.States)
                [new_v, new_a] = controller(current_pose);
                next_pose = path.States(k, :);

                p_dist = utility_functions.euclidean_2D(current_pose, ...
                                                        next_pose);

                obj.set_velocity([new_v 0 0], [0 0 new_a]);
                
                % Stop execution for the time needed to reach the state
                pause(p_dist/new_v);  % Is linear vel, should find other
                
                if path.States(k, :) ~= path.States(end, :)
                    k = k+1;
                end
                current_pose = path.States(k, :);
                dist = utility_functions.euclidean_2D(current_pose, ...
                                                      goal_pose);
            end
            obj.set_current_relative_pose(current_pose);
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
                obj = obj.execute_maneuvers(path, roadmap);

                k = k+1;
            end
            % [scans, poses] = scansAndPoses(obj.slam_builder);
            % save("slam_scans_" + obj.id + ".mat", 'scans');
            % save("slam_poses_" + obj.id + ".mat", 'poses');
            obj.set_velocity([0 0 0]);  % Stop agent after slamming
            obj.show_map();
        end
        function [] = show_map(obj)
            figure;
            show(obj.slam_builder);
            title("slam for agent " + obj.id);
            drawnow;
        end
    end
end