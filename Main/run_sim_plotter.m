clear;
clc;

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

% sim for nominal configuration
profile = 4;
simIn = setVariable(simIn, 'Kp', Kp);
simIn = setVariable(simIn, 'Ki', Ki);
simIn = setVariable(simIn, 'Kd', Kd);
simIn = setVariable(simIn, 'N', N);
simIn = setVariable(simIn, 'phi', phi);
simIn = setVariable(simIn, 'profile', profile);
simIn = setVariable(simIn, 'step_height', step_height);
out = sim(simIn);

mse_signal = out.MSE;
mse_array = mse_signal.Data;
final_mse4 = mse_array(end);

energy_signal = out.energy;
energy_array = energy_signal.Data;
final_energy4 = energy_array(end);

z_sim4 = out.height.Data;
t4 = out.tout;
z_target4 = out.target_signal.Data(1,:);

simIn2 = Simulink.SimulationInput(modelName);

% sim for nominal configuration
profile = 2;
simIn2 = setVariable(simIn2, 'Kp', Kp);
simIn2 = setVariable(simIn2, 'Ki', Ki);
simIn2 = setVariable(simIn2, 'Kd', Kd);
simIn2 = setVariable(simIn2, 'N', N);
simIn2 = setVariable(simIn2, 'phi', phi);
simIn2 = setVariable(simIn2, 'profile', profile);
simIn2 = setVariable(simIn2, 'step_height', step_height);
out = sim(simIn2);

mse_signal = out.MSE;
mse_array = mse_signal.Data;
final_mse2 = mse_array(end);

energy_signal = out.energy;
energy_array = energy_signal.Data;
final_energy2 = energy_array(end);

z_sim2 = out.height.Data;
t2 = out.tout;
z_target2 = out.target_signal.Data(1,:);

simIn3 = Simulink.SimulationInput(modelName);

% sim for nominal configuration
profile = 3;
simIn3 = setVariable(simIn3, 'Kp', Kp);
simIn3 = setVariable(simIn3, 'Ki', Ki);
simIn3 = setVariable(simIn3, 'Kd', Kd);
simIn3 = setVariable(simIn3, 'N', N);
simIn3 = setVariable(simIn3, 'phi', phi);
simIn3 = setVariable(simIn3, 'profile', profile);
simIn3 = setVariable(simIn3, 'step_height', step_height);
out = sim(simIn3);

mse_signal = out.MSE;
mse_array = mse_signal.Data;
final_mse3 = mse_array(end);

energy_signal = out.energy;
energy_array = energy_signal.Data;
final_energy3 = energy_array(end);

z_sim3 = out.height.Data;
t3 = out.tout;
z_target3 = out.target_signal.Data(1,:);

simIn1 = Simulink.SimulationInput(modelName);

% sim for nominal configuration
profile = 1;
simIn1 = setVariable(simIn1, 'Kp', Kp);
simIn1 = setVariable(simIn1, 'Ki', Ki);
simIn1 = setVariable(simIn1, 'Kd', Kd);
simIn1 = setVariable(simIn1, 'N', N);
simIn1 = setVariable(simIn1, 'phi', phi);
simIn1 = setVariable(simIn1, 'profile', profile);
simIn1 = setVariable(simIn1, 'step_height', step_height);
out = sim(simIn1);

mse_signal = out.MSE;
mse_array = mse_signal.Data;
final_mse1 = mse_array(end);

energy_signal = out.energy;
energy_array = energy_signal.Data;
final_energy1 = energy_array(end);

z_sim1 = out.height.Data;
t1 = out.tout;
z_target1 = out.target_signal.Data(1,:);

disp("plotting...")

figure()
hold on;
plot(t1, z_sim1, 'r-', 'LineWidth', 2)
plot(t1, z_target1, 'k--', 'LineWidth', 2)
xlabel("t (s)")
ylabel("h (m)")
legend("Tracked Trajectory", "Reference Trajectory")
grid on
hold off;

figure()
hold on;
plot(t2, z_sim2, 'r-', 'LineWidth', 2)
plot(t2, z_target2, 'k--', 'LineWidth', 2)
legend("Tracked Trajectory", "Reference Trajectory")
xlabel("t (s)")
ylabel("h (m)")
grid on
hold off;

figure()
hold on;
plot(t3, z_sim3, 'r-', 'LineWidth', 2)
plot(t3, z_target3, 'k--', 'LineWidth', 2)
legend("Tracked Trajectory", "Reference Trajectory")
xlabel("t (s)")
ylabel("h (m)")
grid on
hold off;

figure()
hold on;
plot(t4, z_sim4, 'r-', 'LineWidth', 2)
plot(t4, z_target4, 'k--', 'LineWidth', 2)
legend("Tracked Trajectory", "Reference Trajectory")
xlabel("t (s)")
ylabel("h (m)")
grid on
hold off;