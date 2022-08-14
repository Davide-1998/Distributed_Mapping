%Agent Script
classdef agent
    properties
        id = "Agent_default";
        relative_pose = [0, 0, 0, 0, 0, 0];
        scanned_data_sub;
        vel_command;
        current_map = [];
        ros_conn = false;
    end
    
    methods
        function obj = set_id(obj, new_id)
            obj.id = new_id;
        end
        function obj = ros_connect(obj, agent_id)
            scan_topic = strcat('/', agent_id, '/ScanResults');
            scan_sub = utility_functions.subscriber_to_topic(scan_topic);
            vel_topic = strcat('/', agent_id, '/vel');

            obj.scanned_data_sub = scan_sub;
            obj.vel_command = rospublisher(vel_topic);
            obj.ros_conn = true;
        end
        function obj = compute_map(obj, rotation_adjust)
            %{
             Using as map the polar plot of the scanned area. This because
             The scan is 360 and already generates the map somehow.
             So, translating it into cartesian coordinates it is possible
             to have a chunk of the map.
             TODO: Map plot must grow linearly as new area are explored.
                   Growth is equal to scan range.
             Idea is: Scan Data -> cartesian transform -> occupancy matrix
            %}
            if obj.ros_conn == false
                disp("Not connected to ROS");
               
            else
                % if map already exist and agent is in one of the
                % corners update it
                % if map already exist -> update -> i.e. sum values

                LidarData = receive(obj.scanned_data_sub, 3);
                array_of_collisions = LidarData.Ranges;
                res_step = LidarData.AngleIncrement;
                max_range = obj.scanned_data_sub.LatestMessage.RangeMax;
                
                % Due to simulated sensor, data acquisition may vary
                % Need a correction parameter
                if ~exist('rotation_adjust', 'var')
                    rotation_adjust = 0;
                end

                % initialize map matrix -> double range because is radius
                % Use 255 as default value because new areas are supposed
                % empty.
                px_per_meter = 50;
                pixel_line = max_range*px_per_meter;
                new_map = zeros(pixel_line*2) + 255;

                % parse data
                infs = isinf(array_of_collisions);
                for k = 1:numel(array_of_collisions)
                    if infs(k) == 0  % Collision
                        theta = rotation_adjust + (k-1)*res_step;
                        rho = array_of_collisions(k) / max_range;
                        x = round(rho*cos(theta)*pixel_line);
                        y = round(rho*sin(theta)*pixel_line);

                        if x >= 0
                            x = x + pixel_line;
                        else
                            x = pixel_line - abs(x);
                        end

                        if y <= 0
                            y = abs(y) + pixel_line;
                        else
                            y = pixel_line - y;
                        end

                        new_map(y, x) = 0;
                    end
                end
                obj.current_map = new_map;
            end
        end
        function [] = show_map(obj, None)
            image(obj.current_map)
            colorbar
            colormap("gray")
        end
    end
end