% parameters
params.g = 9.81;
params.mu_k = 0.14;
params.Large_Motor_Array = ;
params.Small_Motor_Array = ;
params.throttle_array = ;
params.Ts = 0.01;
params.alpha = 0.1;

% configurations
config.initial_height = 0.0;
config.motor_mass = 0.0;
config.prop_mass = 0.0;
config.cable_mass = 0.0;
config.arm_mass = 0.0;
config.motor_choice = 1;

% hover thrust
m_total = 2 * config.m_motor + 2 * config.m_prop + config.m_cable + config.m_frame;
hover_thrust_gf = (m_total * 1000) / 2; 
exact_throttle = interp1(params.thrust_array, ...
   params.throttle_array, hover_thrust_gf, 'linear', 'extrapolate');
hover_throttle = round(exact_throttle);

disp('Workspace loaded.');