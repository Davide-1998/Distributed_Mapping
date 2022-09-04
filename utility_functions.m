classdef utility_functions
    methods (Static)
        function [] = setup_environment(ip_addr, matlab_ip, master_port)
            if ~exist('master_port', 'var')
                master_port = 11311;
            end
            rosshutdown;
            setenv("ROS_MASTER_URI","http://" + ip_addr + ":" + master_port);
            setenv('ROS_IP', ip_addr);
            setenv('ROS_HOSTNAME',ip_addr);
            rosinit(ip_addr, master_port, ...
                    'NodeHost', matlab_ip, ...
                    'NodeName', '/Matlab_node')
        end

        function dist = euclidean_2D(pose_a, pose_b)
            dist = norm(pose_a(1:2) - pose_b(1:2));
        end
        function dist = euclidean_3D(pose_a, pose_b)
            dist = norm(pose_a(1:3) - pose_b(1:3));
        end

        function sub = subscriber_to_topic(topicname)
            sub = rossubscriber(topicname, "DataFormat", "struct");
        end

        function make_agent_model(agent_id, archetype_dir)
            if ~exist("archetype_dir", 'var')
                archetype_dir = pwd;
            end
            
            % Load archetype model and config
            model_file_path = archetype_dir + "/MyAgent/model.sdf";
            config_file_path = archetype_dir + "/MyAgent/model.config";
            model_lines = readlines(model_file_path);
            config_lines = readlines(config_file_path);

            % update model and config
            model_lines(3) = sprintf('\t<model name=\"' + agent_id + '\">');
            model_lines(238) = sprintf('\t\t<topicName> ' + agent_id + ...
                                       '/ScanResults </topicName>');
            model_lines(258) = sprintf('\t\t<commandTopic> /' + ...
                                       agent_id + '/vel </commandTopic>');
            model_lines(264) = sprintf('\t\t<odometryTopic> /' + ...
                                        agent_id + ...
                                        '/odometry </odometryTopic>');
            config_lines(3) = sprintf('\t<name> ' + agent_id + ' </name>');

            % Creating dir for the model in the general dir
            general_dir = pwd + "/generated_models";
            if ~exist("general_dir", "dir")
                mkdir(general_dir);
            end
            
            model_config_dir = general_dir + '/' + agent_id;
            if ~exist("model_config_dir", "dir")
                mkdir(model_config_dir);
            end
            
            % save model and config
            file_out = fopen(model_config_dir + "/" + "model.sdf", 'wt');
            fprintf(file_out, '%s\n', model_lines);
            fclose(file_out);
            
            file_out = fopen(model_config_dir + "/" + "model.config", 'wt');
            fprintf(file_out, '%s\n', config_lines);
            fclose(file_out);       
        end
        function copy_models_to_gazebo(gazebo_folder, vm_ip, user_vm)
            command = 'cp -r ';
            if ~exist("gazebo_folder", 'var')
                gazebo_folder = '~/.gazebo/models';
            end
            if exist("vm_ip", 'var')
                if ~exist("user_vm", 'var')
                    disp("User for the host machine has to be specified")
                    return
                end
                gazebo_folder = sprintf(user_vm + "@" + vm_ip + ":" + ... 
                                        gazebo_folder);
                command = 'scp -r ';
            end
                 
            command = command + "generated_models/* " + gazebo_folder;
            system(command);
        end
        
        function [new_x, new_y] = H_trans_2D_direct(dist_new_old, point_old, rot_old)
            % Transfer the coordinate of a point from an old coordinate
            % system to a new one. It is the reciprocal operation of
            % H_trans_2D
            rotation = [cos(rot_old) -sin(rot_old); ...
                        sin(rot_old) cos(rot_old)];
            res = dist_new_old + (rotation*point_old');
            new_x = res(1);
            new_y = res(2);
        end

        function [new_x, new_y] = H_trans_2D(d_oo, p_o, r_new)
            % For doubts see the homogeneous transformation theory.
            % d_oo: coordinates of new origin from reference one.
            % p_o: point coordinates wrt the reference origin.
            % r_new: rotation of the new origin wrt the reference one.
            validateattributes(d_oo, {'numeric'}, {'size', [1, 2]});
            validateattributes(p_o, {'numeric'}, {'size', [1, 2]});
            
            diff = p_o - d_oo;
            rotation = [cos(r_new) -sin(r_new); sin(r_new) cos(r_new)];
            
            res = inv(rotation) * diff';
            new_x = res(1);
            new_y = res(2);
        end

        function [xyz_cloud, ground, pits] = pre_process_cloud3D(LidarData, ...
                                                                 agent)
            xyz_cloud = [];
            pits = [];
            cloud_c = 1;
            rho_th = 0.95;
            if ~isa(LidarData, 'numeric')
                for k=1:size(LidarData.Points, 1)

                    x_c = LidarData.Points(k).X;
                    y_c = LidarData.Points(k).Y;
                    z_c = LidarData.Points(k).Z;

                    rho = norm([x_c, y_c, z_c]);

                    if rho < rho_th*agent.lidar_range % It is not the end of the sensor range
                        xyz_cloud(cloud_c, :) = [x_c, y_c, z_c];
                        cloud_c = cloud_c + 1;
                    end
                end
            else
                for k=1:size(LidarData, 1)
                    rho = norm(LidarData(k, :));
                    if rho < rho_th*agent.lidar_range
                        xyz_cloud(cloud_c, :) = LidarData(k, :);
                        cloud_c = cloud_c + 1;
                    end
                end
            end
            disp(size(xyz_cloud))
            t_cloud = pointCloud(xyz_cloud);
            [idxGround, t_cloud, ground] = segmentGroundSMRF(t_cloud, ...
                                                             "MaxWindowRadius", 5, ...
                                                             "SlopeThreshold", 0.07, ...
                                                             "ElevationThreshold", 0.07);

            condition_to_move = 0.5*agent.wheel_radius + agent.lidar_origin_height;
            pits_count = 1;
            for k=1:size(xyz_cloud)               
                if ground.Location(k, 3) < -1*condition_to_move
                    pits(pits_count, :) = ground.Location(k, :);
                    pits_count = pits_count +1;
                end
            end
            t_cloud = removeInvalidPoints(t_cloud);
            xyz_cloud = t_cloud.Location;
        end

        function [ranges, angles] = cartesian_to_polar_2D(matrix)
            ranges = zeros(size(matrix, 1), 1);
            angles = zeros(size(matrix, 1), 1);
            for k=1:size(matrix, 1)
                ranges(k) = norm([matrix(k, 1), matrix(k, 2)]);
                angles(k) = atan2(matrix(k, 2), matrix(k, 1));
            end
        end

        function [ranges, angles] = map_3D_to_2D_polar(agent)
            positions = nodes(agent.poses_graph);
            ranges = [];
            angles = [];

            for k=1:numel(agent.map_cloud)
                t_cloud = agent.map_cloud(k);
                t_cloud = t_cloud.Location;
                t_pose_angle = quat2eul(positions(k, 4:end));
                t_pose = [positions(k, 1:2), t_pose_angle(3)];
                t_cloud = t_cloud + t_pose;
                
                [new_r, new_a] = utility_functions.cartesian_to_polar_2D(t_cloud);
                ranges = [ranges; new_r];
                angles = [angles; new_a];
            end
        end

        function nearby_cloud = get_nearby_clouds(agent)
            nearby_cloud = [];

            if isempty(agent.overviewer)
                return;
            end
            nearby_agents = agent.overviewer.any_nearby(agent.id);
            
            if numel(nearby_agents) ~= 0
                keys = nearby_agents.keys;
                for id_ag=1:numel(keys)
                    selected_agent = agent.overviewer.registered_agents(keys{1, id_ag});
                    
                    % Last cloud is the most updated representation!
                    if ~isempty(selected_agent.global_3D_point_cloud)
                        agent_cloud = cell2mat(selected_agent.global_3D_point_cloud{1, end});
                    else
                        disp([selected_agent.id, " has no clouds"])
                        return;
                    end
                    
                    data = nearby_agents(keys{1, id_ag});
                    rho = data(1);
                    angle = data(2);
                    agent_coord = [rho*cos(angle), rho*sin(angle)];
                    orient = selected_agent.poses_history(end, 3) - ...
                             agent.current_relative_pose(3);
%                     orient = -3.14;

                    % Maybe account for a smart indexing?
                    local_cloud = [];
                    for k=1:size(agent_cloud, 1)
                        agent_points = agent_cloud(k, :);
                        % agent_points(1:2) = agent_points(1:2);
                        [nx, ny] = utility_functions.H_trans_2D(agent_coord, ...
                                                                agent_points(1:2), ...
                                                                orient);
                        % local_points = local_points + agent_coord;
                        local_cloud = [local_cloud; [nx, ny, agent_points(3)]];
                    end
                    nearby_cloud = [nearby_cloud; local_cloud];
                end
            else
                return;
            end
        end

        function [nearby_ranges, nearby_angles] = agent_data_to_local_system(ref_agent)
            nearby_ranges = [];
            nearby_angles = [];
            nearby_agents = [];
            cmap = [];

            overviewer = ref_agent.overviewer;
            if ~isempty(overviewer)
                nearby_agents = overviewer.any_nearby(ref_agent.id);
            end

            if numel(nearby_agents) ~= 0
                keys = nearby_agents.keys;

                for id_ag=1:numel(keys)
                    [scan_data, scan_poses, cmap] = overviewer.get_data(keys{1, id_ag});
                    if isempty(scan_data) == true
                        return;
                    end
                    
                    agent_ranges = []; % zeros(numel(scan_data)*scan_data{1, 1}.Count, 1);
                    agent_angles = []; % zeros(numel(scan_data)*scan_data{1, 1}.Count, 1);
                    
                    for k=1:numel(scan_data)
                        agent_ranges = [agent_ranges; scan_data{1, k}.Ranges];
                        agent_angles = [agent_angles; scan_data{1, k}.Angles];
                    end
                    
                    %Flip the data, only known position is the last one
                    agent_ranges = flipud(agent_ranges);
                    agent_angles = flipud(agent_angles);
                    scan_poses = flipud(scan_poses);

                    % Get current polar position of the agent
                    agent_data = nearby_agents(keys{1, id_ag});
                    range = agent_data(1);
                    incl = agent_data(2);
                    
                    % Position of nearby agent wrt actual agent
                    x_local = range*cos(incl);
                    y_local = range*sin(incl);

                    new_ranges = []; % zeros(1, numel(agent_ranges)) + inf;
                    new_angles = []; % zeros(1, numel(agent_ranges));
                    
                    for pos = 1:size(scan_poses, 1)
                        agent_theta = scan_poses(pos, 3);
                        for k=1:numel(agent_ranges)
                            if ~isinf(agent_ranges(k))
                            
                                theta = agent_angles(k); %- scan_poses(pos, 3); % remove rotation
                                x_range = agent_ranges(k)*cos(theta);
                                y_range = agent_ranges(k)*sin(theta);
                                
                                % Update location parameters
                                if pos < size(scan_poses, 1) && pos > 1
                                    diff = scan_poses(pos, 1:2) - ...
                                           scan_poses(pos-1, 1:2);
                                    x_local = x_local + diff(1);
                                    y_local = y_local + diff(2);
                                end
                                
                                % point location in reference coord
                                new_x = x_local + x_range;
                                new_y = y_local + y_range;
                                
                                new_theta = atan2(new_y, new_x);

                                new_ranges = [new_ranges; norm([new_x, new_y])];
                                new_angles = [new_angles; new_theta];
                            end
                        end
                    end
                    nearby_ranges = [nearby_ranges; new_ranges];
                    nearby_angles = [nearby_angles; new_angles];
                end
            end
        end
    end
end