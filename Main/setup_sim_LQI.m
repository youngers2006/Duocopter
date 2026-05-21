clear
clc
% parameters
% motor
opts = detectImportOptions("Clean_Throttle_Power.csv");
opts.VariableNamingRule = 'preserve'; 
power_data = readtable("Clean_Throttle_Power.csv", opts);
opts = detectImportOptions("Clean_Throttle_Thrust.csv");
opts.VariableNamingRule = 'preserve'; 
thrust_data = readtable("Clean_Throttle_Thrust.csv", opts);

params.throttle_array = thrust_data.Throttle_Percent;
params.thrust_array = 2 * (thrust_data.Thrust_gf / 1000) * 9.81;
params.power_array = 2 * power_data.Power_W;

params.tau_motor = 0.05;
params.v_tolerance = 1e-3;
params.max_v = 40;
params.mu_k = (0.17 + 0.11) / 2;
params.Ts = 0.05;
params.alpha = 0.4;
params.Fs_max = 0.130 * 9.81;
params.max_height = 1.44;
params.min_height = 0;
params.lidar_noise_var = 0.00005;

% configurations
config.initial_height = 0.0;
config.cable_length = 1.0;
config.cart_length = 0.214;
config.drone_cg = 0.01176;
config.motor_moment_arm_y = 0.01176;
config.motor_moment_arm_x = 0.27;

config.cable_mass = 0.19;
config.m_motor = 0.076;
config.m_prop = 0.006;
config.m_esc = 0.04;
config.m_cart = 0.15;
config.m_structure = 0.033;

% hover thrust
m_cable_I = (config.cable_mass / config.cable_length) * ...
    (config.initial_height - (config.cart_length / 2) + config.cable_length);
m_total_I = 2 * (config.m_motor + config.m_prop + config.m_esc) + ...
m_cable_I + config.m_structure + config.m_cart;

params.hover_thrust_I = m_total_I * 9.81;
params.hover_throttle_I = interp1(params.thrust_array, ...
    params.throttle_array, params.hover_thrust_I, 'linear', 'extrap');

% LQI cost matrices
Q = [100, 0, 0;
     0,   4, 0;
     0,   0, 4];
R = 10;

dh = 0.1;
lin_density = 2 * 0.19 / 1;
[K_gains, z_gains] = gain_scheduling(Q, ...
    R, params.mu_k, config.drone_cg, config.cart_length, config.motor_moment_arm_y, params.min_height, params.max_height, ...
    dh, params.Ts, m_total_I, lin_density, params.thrust_array, params.throttle_array);

disp('Workspace loaded.');