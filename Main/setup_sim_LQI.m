% parameters
% Large motor
filename = 'Large_Motor_data.csv';
opts = detectImportOptions(filename);
opts.VariableNamingRule = 'preserve'; 
motor_data = readtable(filename, opts);
raw_pwm = motor_data.("ESC signal (µs)");
raw_thrust_gf = motor_data.("Thrust (gf)");
throttle_mapped = raw_pwm - 1000;
[u_throttle_large, ~, idx_large] = unique(throttle_mapped);
u_thrust_large = accumarray(idx_large, raw_thrust_gf, [], @mean);
valid_indices = u_throttle_large <= 1000;
params.throttle_array_large = u_throttle_large(valid_indices);
params.Large_Motor_Array = u_thrust_large(valid_indices);

% Small motor
filename = 'Small_Motor_data.csv';
opts = detectImportOptions(filename);
opts.VariableNamingRule = 'preserve'; 
motor_data = readtable(filename, opts);
raw_pwm = motor_data.("ESC signal (µs)");
raw_thrust_gf = motor_data.("Thrust (gf)");
throttle_mapped = raw_pwm - 1000;
[u_throttle_small, ~, idx_small] = unique(throttle_mapped);
u_thrust_small = accumarray(idx_small, raw_thrust_gf, [], @mean);
valid_indices = u_throttle_small <= 1000;
params.throttle_array_small = u_throttle_small(valid_indices);
params.Small_Motor_Array = u_thrust_small(valid_indices);

params.g = 9.81;
params.mu_k = 0.14;
params.Ts = 0.01;
params.alpha = 0.2;

% configurations
config.initial_height = 0.0;
config.motor_mass = 0.050;
config.prop_mass = 0.010; 
config.cable_mass = 0.020;
config.arm_mass = 0.030;
config.motor_choice = 1;

% hover thrust
m_total = 2 * config.motor_mass + 2 * config.prop_mass + config.cable_mass + 2 * config.arm_mass;
hover_thrust_gf = (m_total * 1000) / 2;
if config.motor_choice == 1
    exact_throttle = interp1(params.Large_Motor_Array, ...
        params.throttle_array_large, hover_thrust_gf, 'linear', 'extrap');
else % config.motor_choice == 2
    exact_throttle = interp1(params.Small_Motor_Array, ...
        params.throttle_array_small, hover_thrust_gf, 'linear', 'extrap');
end
hover_throttle = round(exact_throttle);

% LQI cost matrices
Q = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];
R = 0.5;

% Linearised system matrices
if config.motor_choice == 1
    dTdu_array = gradient(params.Large_Motor_Array, params.throttle_array_large);
    dTdu = interp1(params.throttle_array_large, ...
        dTdu_array, hover_throttle, 'linear');
else % config.motor_choice == 2
    dTdu_array = gradient(params.Small_Motor_Array, params.throttle_array_small);
    dTdu = interp1(params.throttle_array_small, ...
        dTdu_array, hover_throttle, 'linear');
end

% system matrices
Ad = [0, 1;
      0, 0];
Bd = [0;
      dTdu / m_total];
Cd = [1, 0];

% Build augemnted system with integral state
A = [Ad, zeros(2,1);
     Cd, 1];
B = [Bd;
     0];

% solve for gain matrices
K = dlqr(A, B, Q, R);
K1 = K(1);
K2 = K(2);
K3 = K(3);

disp('Workspace loaded.');