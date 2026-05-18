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
params.thrust_array = (thrust_data.Thrust_gf / 1000) * 9.81;
params.power_array = power_data.Power_W;

% replace this when results are obtained
params.deflection_array = zeros(1, 100);
params.thrust_array_deflection = linspace(0, max(params.thrust_array), 100);

params.tau_motor = 0.2;
params.v_tolerance = 1e-3;
params.max_v = 40;
params.mu_k = (0.17 + 0.11) / 2;
params.Ts = 0.05;
params.alpha = 0.2;
params.Fs_max = 0.130 * 9.81;
params.max_height = 1.44;
params.min_height = 0;
params.lidar_noise_var = 0.05;

% configurations
config.initial_height = 0.0;
config.cable_length = 1.0;
config.cart_length = 0.214;
config.drone_cg = 0.005;
config.motor_moment_arm_y = 0.05;

config.cable_mass = 0.19;
config.m_motor = 0.076;
config.m_prop = 0.006;
config.m_esc = 0.04; 
config.m_cart = 0.15;
config.m_structure = 0.03;

% hover thrust
m_cable_I = (config.cable_mass / config.cable_length) * ...
    (config.initial_height - (config.cart_length / 2) + config.cable_length);
m_total_I = 2 * (config.m_motor + config.m_prop + config.m_esc) + ...
m_cable_I + config.m_structure + config.m_cart;

params.hover_thrust_I = (m_total_I * 9.81) / 2;
params.hover_throttle_I = round(interp1(params.thrust_array, ...
    params.throttle_array, params.hover_thrust_I, 'linear', 'extrap'));

% LQI cost matrices
Q = [100, 0, 0;
     0, 10, 0;
     0, 0, 500];
R = 20;

% Linearised system matrices
if config.motor_choice == 1
    dTdu_array = gradient(params.Large_Motor_Array, params.throttle_array_large);
    dTdu = interp1(params.throttle_array_large, ...
        dTdu_array, hover_throttle * 10, 'linear');
else % config.motor_choice == 2
    dTdu_array = gradient(params.Small_Motor_Array, params.throttle_array_small);
    dTdu = interp1(params.throttle_array_small, ...
        dTdu_array, hover_throttle * 10, 'linear');
end
dTdu = (dTdu * 2) * 0.00981;

% system matrices
Ac = [0, 1;
      0, 0];
Bc = [0;
      dTdu / m_total_I];
Cc = [1, 0];
Dc = 0;

sys_c = ss(Ac, Bc, Cc, Dc);
sys_d = c2d(sys_c, params.Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;

% Build augemnted system with integral state
A = [Ad, zeros(2,1);
     Cd * params.Ts, 1];
B = [Bd;
     0];

% solve for gain matrices
K = dlqr(A, B, Q, R);
K1 = K(1);
K2 = K(2);
K3 = K(3);

disp('Workspace loaded.');