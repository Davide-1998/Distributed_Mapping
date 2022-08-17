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

        function dist = state_distance(pose_a, pose_b)
            dist = norm(pose_a(1:2) - pose_b(1:2));
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
            model_lines(210) = sprintf('\t\t<topicName> ' + agent_id + ...
                                       '/ScanResults </topicName>');
            model_lines(229) = sprintf('\t\t<commandTopic> /' + ...
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
    end
end