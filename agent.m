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
        current_relative_pose = [0; 0; 0];  % x y rotation
        prev_relative_pose = [0; 0; 0];
        absolute_pose = [0, 0, 0];
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
        global_3D_point_cloud = {}; % Cloud 3D representation of map
        local_cloud = [];
        poses_graph = poseGraph3D;
        err_KF;
        no_scans = true;

        % Others
        prev_tf;
        LidarData;
        estimated_acceleration = 0.1741;
        slam_builder;
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

        function obj = initialize_err_KF(obj, meas_cov, proc_cov)
            if ~exist('meas_cov', 'var'); meas_cov=ones(1, 3); end
            if ~exist('proc_cov', 'var'); proc_cov=zeros(1, 3); end
            if size(meas_cov, 2) ~= 3 || size(proc_cov, 2) ~= 3
                error("Input argument must be 3D");
            end
            
            R = diag(meas_cov);
            Q = diag(proc_cov);

            obj.err_KF = KF(R, Q);
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
                             absolute_p, comm_range, ...
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
            if ~exist("lidar_orig_h", 'var')
                lidar_orig_h = 0.135;
            end
            if ~exist("max_lidar_map_range_", 'var')
                max_lidar_map_range_ = scan_range;
            end

            obj.id = id_;
            obj.lidar_range = scan_range;
            obj.communication_range = comm_range;
            obj.max_lidar_map_range = max_lidar_map_range_;
            obj = obj.set_current_relative_pose(current_pose);
            obj.absolute_pose = absolute_p;
            obj.lidar_origin_height = lidar_orig_h;
            obj.estimated_acceleration = est_acc;
            obj.tracking_EKF = trackingEKF(@tran_function, ...
                                           @measurement_function, ...
                                           current_pose, ...
                                           "MeasurementNoise", 0.1, ...
                                           'ProcessNoise', diag([1,1,1]));
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
            for k=1:size(obj.global_3D_point_cloud, 1)
                pcl_wogrd = obj.global_3D_point_cloud(k);
                % pcl_wogrd = pointCloud(local_cloud);
                % Data downsampling for speed
                pcl_wogrd_sampled = pcdownsample(pcl_wogrd, 'random', 0.25);

                % Registering point cloud
                scanAccepted = 1;
                count = size(obj.global_3D_point_cloud, 1);
                if count == 0
                    tform = [];
                else
                    if count == 1
                        moving = obj.global_3D_point_cloud(count);
                        tform = pcregisterndt(pcl_wogrd_sampled, moving, 2.5);
                    else
                        tform = pcregisterndt(pcl_wogrd_sampled, obj.global_3D_point_cloud(count), 2.5, ...
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
                data = receive(obj.scanned_data_sub, 3);
                
                data_in = [data.Points(:).X; data.Points(:).Y; data.Points(:).Z];
                
                % Correct noise KF
                [data_out, updated_kf] = obj.err_KF.train(data_in);
                obj.err_KF = updated_kf;
                
                data_out = data_out';
                
                data = (data_in' + data_out)/2;

                temp_cloud = utility_functions.pre_process_cloud3D(data, ...
                                                                   obj);
                
                % Clouds for nearby agents here (before the for cycle)!
                nearby_cloud = utility_functions.get_nearby_clouds(obj);
                obj.local_cloud = [temp_cloud; nearby_cloud];
                
                % Fix wrt other scans
                num_clouds = size(obj.global_3D_point_cloud, 2);
                if num_clouds >= 1
                    last_cloud = cell2mat(obj.global_3D_point_cloud{1, end});
                    fixed_cloud = pointCloud(last_cloud);
                    mov_cloud = pointCloud(obj.local_cloud);
                    fixed_cloud_down = pcdownsample(fixed_cloud, 'gridAverage', 0.5);
                    mov_cloud_down = pcdownsample(mov_cloud, 'gridAverage', 0.5);

                    tform = pcregistericp(mov_cloud_down, ...
                                          fixed_cloud_down, ...
                                          'Metric', ...
                                          'pointToPlane', ...
                                          'Extrapolate', true);
                    cloud_align = pctransform(mov_cloud, tform);
                    
                    cloud_trans = pcmerge(fixed_cloud, cloud_align, 0.015);
                    cloud_wrt_rel_origin = cloud_trans.Location;

                else
                    cloud_wrt_rel_origin = obj.local_cloud;
                end
                
                obj.global_3D_point_cloud{num_clouds+1} = num2cell(cloud_wrt_rel_origin);
            end
            
            if ~isempty(obj.overviewer)
                obj.overviewer.registered_agents(obj.id) = obj;
            end
        end

        function obj = set_current_relative_pose(obj, pose)
            validateattributes(pose, {'numeric'}, {'size', [1, 3]});

            obj.prev_relative_pose = obj.current_relative_pose;
            obj.current_relative_pose = pose;
            obj.absolute_pose = obj.absolute_pose + pose;
            if ~isempty(obj.overviewer)
                obj.overviewer.registered_agents(obj.id) = obj;
            end
            obj.poses_history = [obj.poses_history; pose];
        end

        function [pthObj, solnInfo] = compute_roadmap(obj)
            % Take the readings
            [scans, poses] = scansAndPoses(obj.slam_builder);
            
            % Build occupancy map
            occMap = buildMap(scans, poses, 10, obj.max_lidar_map_range);
            inflate(occMap, 0.2);
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
            disp(["State start: ", current_pose])
            disp(["State end: ", goal_pose])

            goal_th = 0.5;

            dist = utility_functions.euclidean_2D(current_pose, goal_pose);

            initial_pose = obj.get_current_pose("XYR");

            while dist > goal_th
                [v_l, v_a] = obj.get_current_vels();
                [new_v, new_a] = controller(current_pose);
                obj = obj.set_velocity([new_v 0 0], [0, 0, new_a]);
                start_time = datetime('now');

                if obj.overviewer.is_gps_available(new_pose(1:2))
                    disp("GPS ON")
                [new_x, new_y] = utility_functions.H_trans_2D(initial_pose(1:2), ...
                                                              new_pose(1:2), ...
                                                              obj.absolute_pose(3));
                [wr, ws] = obj.get_factory_setup();

                theta = new_pose(3) - initial_pose(3);
                current_pose = [new_x, new_y, theta];
                
                now_time = datetime('now');
                elapsed_time = seconds(now_time - start_time);
                est_pose = obj.tracking_EKF.predict([new_v, new_a], ...
                                                    [wr, ws], ...
                                                    [0, elapsed_time]);
                x_corr = obj.tracking_EKF.correct(current_pose, ...
                                                  [new_v, new_a], ...
                                                  [wr, ws], ...
                                                  [0, elapsed_time]);

                else
                    disp("GPS OFF - KF in use");
                    now_time = datetime('now');
                    elpsed_time = seconds(now_time - start_time);
                    est_pose = obj.tracking_EKF.predict([new_v, new_a], ...
                                                        [wr, ws], ...
                                                        [0, elapsed_time]);
                    current_pose = est_pose;
                end

                dist = utility_functions.euclidean_2D(current_pose, ...
                                                      goal_pose);

                % initial_pose = new_pose;
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
            
            player = pcplayer([-40, +40], [-40, +40], [-40, 40]);
            my_cloud_ag = [];
            nodesPositions = nodes(obj.poses_graph);
            for k=1:numel(obj.global_3D_point_cloud)
                t_cloud = obj.global_3D_point_cloud(k).Location;
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