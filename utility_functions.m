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

        function sub = subscriber_to_topic(topicname)
            sub = rossubscriber(topicname, "DataFormat", "struct");
        end
    end
end