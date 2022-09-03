classdef agentOverviewer
    properties
        registered_agents;
        map_properties;
    end

    methods
        function obj = agentOverviewer()
            obj.registered_agents = containers.Map;
            obj.map_properties = struct(['no_gps_limits'], [[]]);
        end
        
        function obj = set_no_gps_areas(obj, x_limits, y_limits)
            validateattributes(x_limits, {'numeric'}, {'size', [1, 2]});
            validateattributes(y_limits, {'numeric'}, {'size', [1, 2]});
            limits = [[min(x_limits); max(x_limits)], ...
                      [min(y_limits); max(y_limits)]];
            obj.map_properties.no_gps_limits = limits;
            obj.map_properties.no_gps_limits
        end

        function res = is_gps_available(obj, pos)
            validateattributes(pos, {'numeric'}, {'size', [1, 2]});
            if isempty(obj.map_properties.no_gps_limits)
                disp("No GPS limits setted, returning true");
                res = true;
                return;
            end
            
            if pos(1) < obj.map_properties.no_gps_limits(1, 2) && ...
               pos(1) > obj.map_properties.no_gps_limits(1, 1) && ...
               pos(2) < obj.map_properties.no_gps_limits(2, 2) && ...
               pos(2) > obj.map_properties.no_gps_limits(2, 1)
                res = false;
            else
                res = true;
            end
        end

        function obj = register_agent(obj, agent)
            if isKey(obj.registered_agents, agent.id)
                disp("Agent_id already registered")
                return
            end
            obj.registered_agents(agent.id) = agent;
        end
        
        function res = is_nearby(obj, id_ag_1, id_ag_2)
            if ~isKey(obj.registered_agents, id_ag_1)
                disp(["Agent ", id_ag_1, " is not registered"])
                res = [false, 0, 0];
                return;
            end
            if   ~isKey(obj.registered_agents, id_ag_2)
                disp(["Agent ", id_ag_2, " is not registered"])
                res = [false, 0, 0];
                return;
            end
            
            if id_ag_1 == id_ag_2
                res = [false, 0, 0];
                return;
            end

            agent_a = obj.registered_agents(id_ag_1);
            agent_a_pose = agent_a.absolute_pose;

            agent_b = obj.registered_agents(id_ag_2);
            agent_b_pose = agent_b.absolute_pose;

            dist = utility_functions.euclidean_2D(agent_a_pose, ...
                                                   agent_b_pose);
            % simulate proximity sensor output
            agent_b_to_a_x = agent_b_pose(1) - agent_a.current_relative_pose(1);
            agent_b_to_a_y = agent_b_pose(2) - agent_a.current_relative_pose(2);
            range = dist;
            angle = atan2(agent_b_to_a_y - agent_a_pose(2), ...
                          agent_b_to_a_x - agent_a_pose(1));

             if (dist < agent_a.communication_range)
                res = [true, range, angle, agent_b_pose(3)];
            else
                res = [false, 0, 0, 0];
            end
        end

        function res = any_nearby(obj, agent_id)
            res = containers.Map;
            available_agents = obj.registered_agents.keys;
            for k=1:numel(available_agents)
                key = available_agents{1, k};
                data = obj.is_nearby(agent_id, key);
                if data(1) == true
                    res(key) = data(2:3);
                end
            end
        end

        function [scans, poses, cmap] = get_data(obj, agent_id)
            if ~isKey(obj.registered_agents, agent_id)
                disp(["Agent ", agent_id, " is not registered"]);
            end
   
            selected_agent = obj.registered_agents(agent_id);
            if selected_agent.no_scans == false
                [scans, poses] = scansAndPoses(selected_agent.slam_builder);
                cmap = selected_agent.map_cloud;
            else
                cmap = [];
                scans = [];
                poses = [];
            end
        end

    end
end