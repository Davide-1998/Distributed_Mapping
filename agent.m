%Agent Script
classdef agent
    properties
        id = "Agent_default";
        current_relative_pose = [0; 0; 0];  % x y rotation
        prev_relative_pose = [0; 0; 0];
        scanned_data_sub;
        vel_command;

        current_map = [];
        ros_conn = false;
        slam_builder;
    end
    
    methods
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
            
            send(obj.vel_command, msg);
        end
        function obj = ros_connect(obj, agent_id)
            scan_topic = strcat('/', agent_id, '/ScanResults');
            scan_sub = utility_functions.subscriber_to_topic(scan_topic);
            vel_topic = strcat('/', agent_id, '/vel');
            LidarData = receive(scan_sub, 3);

            obj.scanned_data_sub = scan_sub;
            obj.vel_command = rospublisher(vel_topic);
            obj.ros_conn = true;
            obj.slam_builder = lidarSLAM(20, ...
                                         LidarData.RangeMax);
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

                ranges = zeros(1, numel(array_of_collisions));
                angles = zeros(1, numel(array_of_collisions));
                for k = 1:numel(array_of_collisions)
                        theta = rotation_adjust + (k-1)*res_step;
                        angles(k) = theta;
                        rho = array_of_collisions(k);
                        ranges(k) = rho;
                end 
                scan_in = lidarScan(ranges, angles);
               
                addScan(obj.slam_builder, scan_in);
                [scans, poses] = scansAndPoses(obj.slam_builder);
                % Update agent relative position
                obj.prev_relative_pose = obj.current_relative_pose;
                obj.current_relative_pose = poses(end, :);
            end

        end
        function [] = compute_roadmap(obj)
            % Take last reading
            [scans, poses] = scansAndPoses(obj.slam_builder);
            occMap = buildMap(scans, poses, 20, 5);
            occMap.FreeThreshold = 0.65;
            
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
            planner.BallRadiusConstant = 0.4;
            planner.MaxNumTreeNodes = 200;
            planner.MaxConnectionDistance = 0.4;
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
            pthObj;

            % map = scans(end);
            % prob_occupied_map = binaryOccupancyMap(map/255);
            % prob_occupied_map.setOccupancy(map/255);
            % inflate(prob_occupied_map, 0.5);
        end
        function [] = do_slam(obj, iterations, vel, correction_angle)
            if ~exist("iterations", "var")
                iterations = 100;
            end
            if ~exist("vel", "var")
                vel = [1 0 0];
            end
            if ~exist("correction_angle", "var")
                correction_angle = 3.14;
            end

            obj.set_velocity(vel);
            k = 1;
            while true && k < iterations
                obj = obj.compute_map(correction_angle);
                if mod(k, 10)
                    obj.set_velocity(vel/10);
                    obj.compute_roadmap();
                    obj.set_velocity(vel);
                end
                k = k+1;
            end
            % [scan, poses] = scansAndPoses(obj.slam_builder);
            % save('scan.mat', 'scan');
            obj.set_velocity([0 0 0]);  % Stop agent after slamming
            obj.show_map();
        end
        function [] = show_map(obj)
            figure;
            show(obj.slam_builder);
            drawnow;
        end
    end
end