classdef agentOverviewer
    properties
        registered_agents;
    end

    methods
        function obj = agentOverviewer()
            obj.registered_agents = containers.Map("char", "any");
        end

        function register_agent(obj, agent)
            if isKey(obj, agent.id)
                disp("Agent_id already registered")
                return
            end
            obj.registered_agents(agent.id) = agent;
        end
        
        function res = is_nearby(obj, id_ag_1, id_ag_2)
            if ~isKey(obj, id_ag_1) || ~isKey(obj, id_ag_2)
                disp("One of the agents is not registered")
                res = false;
                return;
            end
            
            agent_a = obj.registered_agents(id_ag_1);
            agent_a_pose = agent_a.current_relative_pose;
            agent_b = obj.registered_agents(id_ag_2);
            agent_b_pose = agent_b.current_relative_pose;
            dist = unitlity_functions.euclidean_3D(agent_a_pose, ...
                                                   agent_b_pose); 
            if (dist < agent_a.communication_range)
                res = true;
            else
                res = false;
            end
        end
        
    end
end