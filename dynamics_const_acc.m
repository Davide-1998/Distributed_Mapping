% Const accellerate body dynamics

function pos = dynamics_const_acc(params, axis)
    % params is a vector having [now_pos, now_vel, new_vel, theta, elapsed_time]
    now_pos = params(1);
    now_vel = params(2);
    new_vel = params(3);
    theta = params(4);
    el_time = params(end);
    
    mul = 1;
    if axis == "x"
        mul = cos(theta);
    elseif axis == "y"
        mul = sin(theta);
    end

    acc_space = (new_vel - now_vel)*el_time*0.5 * mul;
    vel_space = now_vel*el_time * mul;

    pos = now_pos + vel_space + acc_space;
end