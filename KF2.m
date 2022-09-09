% KF2
% Translation
% A -> state_tr -> default to I
% B -> input_matrix -> default to 0
% H -> ob_matrix -> default to I
% R -> ob_cov -> Necessary
% Q -> process_noise_cov -> default to I
% u -> input vector
classdef KF2
    properties
        state_est;
        state_cov;

        problem_dim;
        state_tr;
        input_matrix;
        input_vector;
        ob_matrix;
        ob_cov;
        process_noise_cov;

        first_it = true;
    end
    methods
        % Constructor
        function obj = KF2(problem_dim, R_mod, Q_mod)
            if ~exist('problem_dim', 'var')
                error("A dimension of vectors is required");
            end
            if ~exist('Q_mod', 'var')
                Q_mod = 1;
            end
            if ~exist('R_mod', 'var')
                error("A value for the Observation Covariance R must be setted");
            end

            obj.problem_dim = problem_dim;
            obj.state_tr = eye(problem_dim);
            obj.input_matrix = zeros(problem_dim);
            obj.input_vector = zeros(problem_dim, 1);
            obj.ob_matrix = eye(problem_dim);
            obj.ob_cov = eye(problem_dim)*R_mod;
            obj.process_noise_cov = eye(problem_dim)*Q_mod;
        end
        
        function [next_state, obj] = estimate(obj, measurement)

            if obj.first_it
                
                obj.state_est = inv(obj.ob_matrix) * measurement;
                obj.state_cov = inv(obj.ob_matrix) * obj.ob_cov * inv(obj.ob_matrix');
                
                obj.first_it = false;
            else
                obj.state_est = (obj.state_tr * obj.state_est) + ...
                            (obj.input_matrix * obj.input_vector);
                obj.state_cov = obj.state_tr * obj.state_cov * obj.state_tr' + obj.process_noise_cov;

                k_gain = obj.state_cov * obj.ob_matrix' * ...
                         inv((obj.ob_matrix * obj.state_cov * obj.ob_matrix') + obj.ob_cov);

                obj.state_est = obj.state_est + ...
                                ( k_gain * ( measurement - (obj.ob_matrix * obj.state_est) ) );
                obj.state_cov = obj.state_cov - ( k_gain * obj.ob_matrix * obj.state_cov );
            end

            next_state = obj.state_est';  % [1 x problem_dim]
        end
    end
end