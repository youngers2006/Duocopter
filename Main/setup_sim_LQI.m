% parameters
params.g = 9.81;
params.mu_k = 0.14;
params.Large_Motor_Array = ;
params.Small_Motor_Array = ;
params.throttle_array = ;
params.Ts = 0.01;

% configurations
config.initial_height = 0.0;
config.motor_mass = 0.0;
config.prop_mass = 0.0;
config.cable_mass = 0.0;
config.arm_mass = 0.0;
config.motor_choice = 1;

% hover thrust
m_total = 2 * config.motor_mass + 2 * config.prop_mass + config.cable_mass + 2 * config.arm_mass;
hover_thrust_gf = (m_total * 1000) / 2;
exact_throttle = interp1(params.thrust_array, ...
   params.throttle_array, hover_thrust_gf, 'linear', 'extrapolate');
hover_throttle = round(exact_throttle);

% LQI cost matrices
Q = [1, 0, 0;
     0, 1, 0;
     0, 0, 1];
R = 0.5;

% Linearised system matrices
dTdu_array = gradient(params.thrust_array, params.throttle_array);
dTdu = interp1(dTdu_array, ...
    params.throttle_array, hover_throttle, 'linear');

% system matrices
Ad = [0, 0;
      0, 1];
Bd = [0;
      dTdu / m_total];
Cd = [1, 0];

A = [Ad, 0;
     Cd, 1];
B = [Bd;
     0];

K = dlqr(A, B, Q, R);
K1 = K(1);
K2 = K(2);
K3 = K(3);

disp('Workspace loaded.');