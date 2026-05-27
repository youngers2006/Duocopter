clear;
clc;

profile = 4;
step_height = 1;

[params.power_array, params.thrust_array, params.throttle_array] = process_motor_data("Small_Motor_data.csv");

params.tau_motor = 0.006;
params.v_tolerance = 1e-3;
params.max_v = 40;
params.mu_k = (0.17 + 0.11) / 2;
params.Ts = 0.05;
params.Fs_max = 0.130 * 9.81;
params.max_height = 1.44;
params.min_height = 0;
params.lidar_noise_var = 4.9129e-05;

% configurations
config.initial_height = 0;
config.cable_length = 1.0;
config.cart_length = 0.214;
config.drone_cg = 0.01176;
config.motor_moment_arm_y = 0.0135;
config.motor_moment_arm_x = 0.27;

config.cable_mass = 0.19;
config.m_motor = 0.076;
config.m_prop = 0.006;
config.m_esc = 0.04;
config.m_cart = 0.15;
config.m_structure = 0.057;

% gains
Kp = 357.4457;
Ki = 124.9598;
Kd = 274.1128;
N = 20.7210;
phi = 16.3915;

% hover thrust at zero height
config.m_cable_I = (config.cable_mass / config.cable_length) * ...
    (config.cable_length - (config.cart_length / 2));
config.m_total_I = 2 * (config.m_motor + config.m_prop + config.m_esc) + ...
config.m_cable_I + config.m_structure + config.m_cart;

params.hover_thrust_I = config.m_total_I * 9.81;
params.hover_throttle_I = interp1(params.thrust_array, ...
    params.throttle_array, params.hover_thrust_I, 'linear', 'extrap');

ref_signal = timeseries([0, 0, 1.0, 0.2, 0.2, 1.1, 1.1, 0.25, 0.9, 0.5, 0, 0], ...
    [0, 5, 10, 20, 30, 30.001, 41, 46, 46.001, 65, 70, 80]);
ref_signal.Name = 'Reference_Height';

modelName = 'Model_PID';
load_system(modelName);
simIn = Simulink.SimulationInput(modelName);

for i = 0:10
    % sim for nominal configuration
    params.lidar_noise_var = 4.9129e-05 * ((i + 10)/10);
    profile = 4;
    simIn = setVariable(simIn, 'config', config);
    simIn = setVariable(simIn, 'params', params);
    simIn = setVariable(simIn, 'Kp', Kp);
    simIn = setVariable(simIn, 'Ki', Ki);
    simIn = setVariable(simIn, 'Kd', Kd);
    simIn = setVariable(simIn, 'N', N);
    simIn = setVariable(simIn, 'phi', phi);
    simIn = setVariable(simIn, 'profile', profile);
    simIn = setVariable(simIn, 'step_height', step_height);
    simIn = setVariable(simIn, 'ref_signal', ref_signal);
    out = sim(simIn);
    
    mse_signal = out.MSE;
    mse_array = mse_signal.Data;
    final_mse = mse_array(end);
    
    energy_signal = out.energy;
    energy_array = energy_signal.Data;
    final_energy = energy_array(end);
    
    disp("MSE:")
    disp(final_mse)

    disp("Energy:")
    disp(final_energy)
    disp("-----------------")
end