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
            model_lines(220) = sprintf('\t\t<topicName> ' + agent_id + ...
                                       '/ScanResults </topicName>');
            model_lines(239) = sprintf('\t\t<commandTopic> /' + ...
                                       agent_id + '/vel </commandTopic>');

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

        function [nearby_ranges, nearby_angles] = agent_data_to_local_system(ref_agent)
            nearby_ranges = [];
            nearby_angles = [];
            overviewer = ref_agent.overviewer;
            nearby_agents = overviewer.any_nearby(ref_agent.id);
            
            if numel(nearby_agents) ~= 0
                keys = nearby_agents.keys;

                for id_ag=1:numel(keys)
                    [scan_data, scan_poses] = overviewer.get_data(keys{1, id_ag});
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