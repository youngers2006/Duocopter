function [x_dot] = physics_model(x, u, config, params)
% Function containing all physics for the drone
% x = [pos; pos_dot]
    % Masses
    m_motor = config.motor_mass;
    m_prop = config.prop_mass;
    m_cable = config.cable_mass;
    m_frame = 2 * config.arm_mass; 
    m_total = 2 * m_motor + 2 * m_prop + m_cable + m_frame;

    % Throttle setting thrust
    motor = config.motor_choice;
    if motor == 1
        thrust_array = params.Large_Motor_Array; % get from table 
        throttle_array = params.throttle_array_large;
    else % motor == 2
        thrust_array = params.Small_Motor_Array; % get from other table
        throttle_array = params.throttle_array_small;
    end
    T = interp1(throttle_array, ...
        thrust_array, u, 'linear', 'extrap');

    % Forces
    W = m_total * params.g;

    if x(2) == 0 
        F = params.static_friction;
    else
        N = params.static_friction / params.mu_s; % add thrust from bending contribution
        F = params.mu_k * N;
    end

    % Force balance
    R = T - (W + F);
    a = R / m_total;

    % State derivative
    x_dot = zeros(2,1);
    x_dot(1) = x(2);
    x_dot(2) = a;
end