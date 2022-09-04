% Kalmann Filter class
% Ref to https://it.mathworks.com/matlabcentral/fileexchange/24486-kalman-filter-in-matlab-tutorial
classdef KF
    properties
        current_state_est; % X
        state_transition_matrix;  % A
        state_covariance; % P
        proc_noise_cov; % Q
        meas_noise_cov; % R
        ob_matrix; % H
        input_matrix; % B
    end

    methods
        function obj = KF(R, Q, X, A, P, H, B)
            if ~exist('X', 'var')
                disp("No initial state setted"); 
            else
                obj.current_state_est = X;
            end
            if ~exist('A', 'var'); A = eye(3); end
            if ~exist('P', 'var'); P = eye(3); end
            if ~exist('Q', 'var'); Q = zeros(3); end
            if ~exist('R', 'var'); R = eye(3); end
            if ~exist('H', 'var'); H = eye(3); end
            if ~exist('B', 'var'); B = zeros(3); end
            
            obj.state_transition_matrix = A;
            obj.state_covariance = P;
            obj.proc_noise_cov = Q;
            obj.meas_noise_cov = R;
            obj.ob_matrix = H;
            obj.input_matrix = B;
        end

        function [new_state, obj] = predict(obj, meas_in)
            if ~exist('meas_in', 'var'); meas_in = zeros(1, 3); end
            if ~isempty(obj.current_state_est)
                new_state = (obj.state_transition_matrix*obj.current_state_est) + ...
                             obj.input_matrix*meas_in;
                obj.current_state_est = new_state;

                new_sc = obj.state_transition_matrix*obj.state_covariance*(obj.state_transition_matrix') ...
                         + obj.proc_noise_cov;
                obj.state_covariance = new_sc;
            else
                obj.current_state_est = inv(obj.ob_matrix) * meas_in;
                new_state = obj.current_state_est;
                
                obj.state_covariance = inv(obj.ob_matrix) * obj.meas_noise_cov * inv(obj.ob_matrix)';
            end
        end

        function [curr_state, obj] = train(obj, new_meas)
            [new_est, obj] = obj.predict(new_meas);

            kalmann_gain = obj.state_covariance * (obj.ob_matrix') * ...
                           inv((obj.ob_matrix * obj.state_covariance * ...
                                (obj.ob_matrix')) + obj.meas_noise_cov);
            
            curr_state = new_est + kalmann_gain*(new_meas - (obj.ob_matrix*new_est));
            obj.current_state_est = curr_state;

            obj.state_covariance = obj.state_covariance - ...
                                   kalmann_gain*obj.ob_matrix*obj.state_covariance;
        end
    end
end