params.g = 9.81;
params.m_cart = 0.150;
params.m_total = 0.650;
params.mu_k = 0.14;
params.L_thrust = 0.27;

config.velocity_tolerance = 1e-3;
config.initial_height = 0.0;
config.max_thrust_gf = 1300;
config.target_height = 1.0;

hover_thrust_gf = (params.m_total * 1000) / 2; 
exact_throttle = interp1(params.thrust_array, ...
   params.throttle_array, hover_thrust_gf, 'linear', 'extrapolate');
hover_throttle = round(exact_throttle);

disp('Workspace loaded.');