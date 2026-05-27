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
Kp = 451.0632;
Ki = 269.5600;
Kd = 216.8701;
N = 21.5158;
phi = 31.6358;

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
set_param(modelName, 'SignalLogging', 'off');
set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
set_param(modelName, 'FastRestart', 'on');

N_steps = 30;

mu_k_res = zeros(2, N_steps + 1);
Fs_res = zeros(2, N_steps + 1);
m_structure_res = zeros(2, N_steps + 1);
x_cg_res = zeros(2, N_steps + 1);
tau_res = zeros(2, N_steps + 1);

for i = 1:5 % mu_k, Fs, m_structure, x_cg, tau
    if i == 1
        max = 1.7;
        min = 1.1;
    elseif i == 2
        max = 0.2 * 9.81;
        min = 0.05 * 9.81;
    elseif i == 3
        max = 0.107;
        min = 0.007;
    elseif i == 4
        max = 0.01764;
        min = 0.00588;
    elseif i == 5
        max = 0.009;
        min = 0.003;
    end

    for j = 0:N_steps
        if i == 1
            params.mu_k = min + (max / N_steps) * j;
        elseif i == 2
            params.Fs_max = min + (max / N_steps) * j;
        elseif i == 3
            config.m_structure = min + (max / N_steps) * j;
        elseif i == 4
            config.drone_cg = min + (max / N_steps) * j;
        elseif i == 5
            params.tau_motor = min + (max / N_steps) * j;
        end

        simIn = Simulink.SimulationInput(modelName);
        
        % sim for nominal configuration
        profile_ramp = 4;
        simIn = setVariable(simIn, 'params', params);
        simIn = setVariable(simIn, 'config', config);
        simIn = setVariable(simIn, 'Kp', Kp);
        simIn = setVariable(simIn, 'Ki', Ki);
        simIn = setVariable(simIn, 'Kd', Kd);
        simIn = setVariable(simIn, 'N', N);
        simIn = setVariable(simIn, 'phi', phi);
        simIn = setVariable(simIn, 'profile', profile_ramp);
        simIn = setVariable(simIn, 'step_height', step_height);
        simIn = setVariable(simIn, 'ref_signal', ref_signal);
        out = sim(simIn);
        
        mse_signal = out.MSE;
        mse_array = mse_signal.Data;
        final_mse = mse_array(end);
        
        energy_signal = out.energy;
        energy_array = energy_signal.Data;
        final_energy = energy_array(end);

        if i == 1
            mu_k_res(1, j+1) = final_mse;
            mu_k_res(2, j+1) = final_energy;
        elseif i == 2
            Fs_res(1, j+1) = final_mse;
            Fs_res(2, j+1) = final_energy;
        elseif i == 3
            m_structure_res(1, j+1) = final_mse;
            m_structure_res(2, j+1) = final_energy;
        elseif i == 4
            x_cg_res(1, j+1) = final_mse;
            x_cg_res(2, j+1) = final_energy;
        elseif i == 5
            tau_res(1, j+1) = final_mse;
            tau_res(2, j+1) = final_energy;
        end

        params.mu_k = (0.17 + 0.11) / 2;
        params.Fs_max = 0.130 * 9.81;
        config.m_structure = 0.057;
        config.drone_cg = 0.01176;
        params.tau_motor = 0.06;
    end
end

set_param(modelName, 'FastRestart', 'off');