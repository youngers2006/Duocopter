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
        Thrust_curve = params.Large_Motor_Curve; % get from table 
    elseif motor == 2
        Thrust_curve = params.Small_Motor_Curve; % get from other table
    end
    T = polyval(Thrust_curve, u);

    % Forces
    W = m_total * params.g;
    F = x(2) * params.mu;

    % Force balance
    R = T - (W + F);
    a = R / m_total;

    % State derivative
    x_dot = zeros(2);
    x_dot(1) = x(2);
    x_dot(2) = a;
end