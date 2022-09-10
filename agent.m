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

        % Factory Details
        wheel_radius = 0.06;
        wheel_separation = 0.24;
        height = 0.15;

        % Motion
        current_est_pose = [0, 0, 0];  % x y rotation
        prev_est_pose = [0, 0, 0];
        absolute_origin = [0, 0, 0];        
        tracking_EKF;
        poses_history = [];
        
        % Lidar Sensor
        lidar_range = 0;
        hz_resolution = 360;
        vt_resolution = 16;
        minmax_hz_angles = [-3.14, 3.14];
        minmax_vt_angles = [-0.26, 0.26];
        lidar_origin_height = 0.135;
        max_lidar_map_range = 0;
        expected_input_size = [5760, 3];

        % Communications
        communication_range = 0;
        known_agents = [];
        overviewer;
        scanned_data_sub;
        odometry_sub;
        vel_command;
        ros_conn = false;
        
        % SLAM Attributes
        slam_builder;
        global_3D_point_cloud = {}; % Cloud 3D representation of map
        global_occupancy;
        local_cloud = [];
        poses_graph = poseGraph3D;
        err_KF;
        kf_th = 10;
        no_scans = true;
        local_scans = {};

        % Others
        prev_tf;
        LidarData;
        estimated_acceleration = 0.1741;
    end
    
    methods
        function obj = set_lidar_parameters(obj, range, sensor_height, hz_res, ...
                                            vt_res, hz_angles, vt_angles, ...
                                            max_render_distance)

            if ~exist('range', 'var'); range=10; end
            if ~exist('sensor_height', 'var'); sensor_height=0.135; end
            if ~exist('hz_res', 'var'); hz_res=360; end
            if ~exist('vt_res', 'var'); vt_res=16; end
            if ~exist('hz_angles', 'var'); hz_angles=[-3.14, 3.14]; end
            if ~exist('vt_angles', 'var'); vt_angles=[-0.26, 0.26]; end
            if ~exist('max_render_distance', 'var'); max_render_distance=range*2; end

            obj.lidar_range = range;
            obj.hz_resolution = hz_res;
            obj.vt_resolution = vt_res;
            obj.minmax_hz_angles = hz_angles;
            obj.minmax_vt_angles = vt_angles;
            obj.lidar_origin_height = sensor_height;
            obj.max_lidar_map_range = max_render_distance;

            obj.expected_input_size = [hz_res*vt_res, 3];
        end

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

        function obj = set_factory_setup(obj, wr, ws)
            obj.wheel_radius = wr;
            obj.wheel_separation = ws;
        end

        function [wr, ws] = get_factory_setup(obj)
            wr = obj.wheel_radius;
            ws = obj.wheel_separation;
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
        function pose = get_current_pose(obj, vals)
            if ~exist("vals", "var")
                vals="full";
            end
            mustBeText(vals);

            odomData = receive(obj.odometry_sub, 3);
            position = odomData.Pose.Pose.Position;
            orientation = obj.get_current_orientation();
            if vals == "XYR"
                pose = [position.X, position.Y, orientation(3)];
            elseif vals == "XYZR"
                pose = [position.X, position.Y, position.Z, orientation(3)];
            elseif vals == "full"
                pose = [position.X, position.Y, position.Z, orientation];
            end
        end
        function orientation = get_current_orientation(obj)
            odomData = receive(obj.odometry_sub, 3);
            or_ = odomData.Pose.Pose.Orientation;
            quat_orientation = [or_.X, or_.Y, or_.Z, or_.W];
            orientation = quat2eul(quat_orientation, 'ZYX');
        end
        function obj = agent(id_, scan_range, current_pose, ...
                             absolute_o, comm_range, ...
                             max_lidar_map_range_, lidar_orig_h, est_acc, ...
                             kf_threshold)
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
            if ~exist("absolute_o", 'var')
                absolute_o = [0, 0, 0];
            end
            if ~exist("lidar_orig_h", 'var')
                lidar_orig_h = 0.135;
            end
            if ~exist("max_lidar_map_range_", 'var')
                max_lidar_map_range_ = scan_range;
            end
            if ~exist("kf_threshold", 'var')
                kf_threshold = 10;
            end

            obj.id = id_;
            obj.lidar_range = scan_range;
            obj.communication_range = comm_range;
            obj.max_lidar_map_range = max_lidar_map_range_;
            obj = obj.set_current_est_pose(current_pose);
            obj.absolute_origin = absolute_o;
            obj.lidar_origin_height = lidar_orig_h;
            obj.estimated_acceleration = est_acc;
            obj.tracking_EKF = trackingEKF(@tran_function, ...
                                           @measurement_function, ...
                                           current_pose, ...
                                           "MeasurementNoise", 0.1, ...
                                           'ProcessNoise', diag([1,1,1]));
            obj.err_KF = KF2(3, 0.01, 1);
        end

        function obj = modify_KF_params(R_value, Q_value, input_dim)
            if ~exist("R_value", "var"); R_value = 1; end
            if ~exist("Q_value", "var"); Q_value = 1; end
            if exist("input_dim", "var")
                obj.err_KF = KF2(input_dim, R_value, Q_value);
            else
                obj.err_KF = KF2(3, R_value, Q_value);
            end
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

            % obj.current_linear_vel = vel_linear(1);
            % obj.current_angular_vel = vel_angular(3);

            send(obj.vel_command, msg);
            % obj.last_set_vel_t = now;
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

            obj.scanned_data_sub = scan_sub;
            obj.odometry_sub = od_sub;
            obj.vel_command = rospublisher(vel_topic);
            obj.ros_conn = true;
            obj.slam_builder = lidarSLAM(10, obj.max_lidar_map_range);
            obj.slam_builder.LoopClosureThreshold = 210;  
            obj.slam_builder.LoopClosureSearchRadius = 5;
        end
        
        function obj = compute_map(obj, mov_th)
            if ~exist('mov_th', 'var')
                mov_th = 5;
            end
            if obj.ros_conn == false
                disp("Not connected to ROS");
                return;
            else
                data = receive(obj.scanned_data_sub, 3);
                
                data_in = [data.Points(:).X; data.Points(:).Y; data.Points(:).Z];
                
                num_clouds = size(obj.global_3D_point_cloud, 2);
                
                % Correct noise KF
                [data_corr, obj.err_KF] = obj.err_KF.estimate(data_in);

                data = data_corr;
                [temp_cloud, ~, pits] = utility_functions.pre_process_cloud3D(data, ...
                                                                              obj);
                obj.local_cloud = temp_cloud;

                % Make occupancy map
                full_occupancy = [temp_cloud; pits];
                [ranges, angles] = utility_functions.cartesian_to_polar_2D(full_occupancy);          
                scan_in = lidarScan(ranges, angles);
                
                obj.local_scans{end+1} = scan_in;

                addScan(obj.slam_builder, scan_in, obj.current_est_pose);
                % [scans, poses] = scansAndPoses(obj.slam_builder);
                
                occMap = buildMap(obj.local_scans, obj.poses_history, 10, obj.max_lidar_map_range);

                obj.global_occupancy = occMap;
                
                obj.local_cloud(:, 1:2) = utility_functions.H_trans_2D_new(obj.current_est_pose(1:2), ...
                                                                           temp_cloud(:, 1:2), ...
                                                                           obj.current_est_pose(3));
                
                % Clouds from nearby agents
                data_n = utility_functions.get_nearby_data(obj);
                if ~isempty(data_n.nearby_cloud)
                    obj.local_cloud = [obj.local_cloud; data_n.nearby_cloud];
                end
                if ~isempty(data_n.scans)
                    scans_n = data_n.scans;
                    poses_n = data_n.poses;
                    
                    scans = obj.local_scans;
                    poses = obj.poses_history;
                    
                    for k=1:size(scans_n, 2)
                        scans{end+k} = scans_n{1, k};
                    end
                    poses = [poses; poses_n];

                    obj.global_occupancy = buildMap(scans, poses, 10, obj.max_lidar_map_range);
                    
                end

                num_clouds = size(obj.global_3D_point_cloud, 2);
                
                if num_clouds > 1
                    obj.local_cloud = [cell2mat(obj.global_3D_point_cloud{end}); ... 
                                       obj.local_cloud];
                end

                obj.global_3D_point_cloud{num_clouds+1} = num2cell(obj.local_cloud);
            end
            
            % Update overviewer informations
            if ~isempty(obj.overviewer)
                obj.overviewer.registered_agents(obj.id) = obj;
            end
        end

        function obj = set_current_est_pose(obj, pose)
            validateattributes(pose, {'numeric'}, {'size', [1, 3]});

            obj.prev_est_pose = obj.current_est_pose;
            obj.current_est_pose = pose;
            if ~isempty(obj.overviewer)
                obj.overviewer.registered_agents(obj.id) = obj;
            end
            obj.poses_history = [obj.poses_history; pose];
        end

        function [pthObj, solnInfo] = compute_roadmap(obj)
            % Take OccupancyMap
%             [scans, poses] = scansAndPoses(obj.slam_builder);
%             occMap = buildMap(scans, obj.poses_history, 10, obj.max_lidar_map_range);
            occMap = obj.global_occupancy;
            
            viewer = pcplayer([-20, 20], [-20, 20], [-20, 20]);
            view(viewer, obj.local_cloud);

            inflate(occMap, 0.1);
            occMap.FreeThreshold = 0.48;
            
            % Generate search space and node validator
            now_pose = obj.current_est_pose;
            
            low_bound_x = now_pose(1); % - obj.slam_builder.MaxLidarRange;
            high_bound_x = now_pose(1) + obj.slam_builder.MaxLidarRange;
            low_bound_y = now_pose(2); % - obj.slam_builder.MaxLidarRange;
            high_bound_y = now_pose(2) + obj.slam_builder.MaxLidarRange;
            low_bound_rot = now_pose(3) - (pi/2);
            high_bound_rot = now_pose(3) + (pi/2);
            
            disp([ low_bound_x, high_bound_x, ...
                  low_bound_y, high_bound_y, low_bound_rot, high_bound_rot])

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
%                 disp(["Sampled next state", next_state])
                next_state = sampleUniform(space);
            end
            disp(["Next state", next_state])

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

            controller.DesiredLinearVelocity = 0.8;
            controller.MaxAngularVelocity = 10;
            controller.LookaheadDistance = 0.1;

            current_pose = obj.current_est_pose; % path.States(1, :);
            goal_pose = path.States(end, :);
            disp(["State start: ", current_pose])
            disp(["State end: ", goal_pose])

            goal_th = 0.5;

            dist = utility_functions.euclidean_2D(current_pose, goal_pose);

            % initial_pose = obj.get_current_pose("XYR");

            while dist > goal_th
                [new_v, new_a] = controller(obj.current_est_pose);
                obj = obj.set_velocity([new_v 0 0], [0, 0, new_a]);
                start_time = datetime('now');

                pause(0.1)

                current_pose = obj.get_current_pose("XYR");
                if ~isempty(obj.overviewer)
                    if obj.overviewer.is_gps_available(current_pose(1:2))
                        disp("GPS ON")

                        [wr, ws] = obj.get_factory_setup();

                        % disp(["Curr pose", current_pose])

                        now_time = datetime('now');
                        elapsed_time = seconds(now_time - start_time);
                        est_pose = obj.tracking_EKF.predict([new_v, new_a], ...
                                                            [wr, ws], ...
                                                            [0, elapsed_time]);

                        x_corr = obj.tracking_EKF.correct(current_pose, ...
                                                          [new_v, new_a], ...
                                                          [wr, ws], ...
                                                          [0, elapsed_time]);
                        obj.current_est_pose = current_pose;
                    else
                        disp("GPS OFF - KF in use");
                        now_time = datetime('now');
                        elapsed_time = seconds(now_time - start_time);
                        est_pose = obj.tracking_EKF.predict([new_v, new_a], ...
                                                            [wr, ws], ...
                                                            [0, elapsed_time]);
                        obj.current_est_pose = est_pose;
                    end
                else
                    obj.current_est_pose = current_pose;
                end

                dist = utility_functions.euclidean_2D(obj.current_est_pose, ...
                                                      goal_pose);

                disp([current_pose, "Dist: ", dist])
            end
            obj = obj.set_current_est_pose(current_pose);
        end

        function [] = do_slam(obj, iterations, mov_th)
            if ~exist("iterations", "var"); iterations = 100; end
            if ~exist("mov_th", "var"); mov_th = obj.lidar_range; end

            k = 1;
            while k <= iterations
                obj = obj.compute_map(mov_th);
                [path, roadmap] = obj.compute_roadmap();
                disp("Executing maneuvers")
                obj = obj.execute_maneuvers(path);
                k = k+1;
            end
            obj = obj.set_velocity();  % Stop agent after slamming
            obj.show_map();
        end

        function [] = show_map(obj)
            figure;
            title("Occupancy Map");
            show(obj.global_occupancy);
            
            my_cloud_ag = pointCloud(cell2mat(obj.global_3D_point_cloud{1, end}));
            player = pcplayer(my_cloud_ag.XLimits, my_cloud_ag.YLimits, my_cloud_ag.ZLimits);
          
            while isOpen(player) 
                view(player, my_cloud_ag);           
            end
        end
    end
end