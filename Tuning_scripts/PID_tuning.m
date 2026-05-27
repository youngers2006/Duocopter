disp('Initialising Optimiser...');

% Load parameters and optimisation bounds
[config, params, ref_signal] = load_pid_params();
num_vars = 5;
lb = [0.0, 0.0, 0.0, 1, 0.0]; 
ub = [400.0, 400.0, 400.0, 30, 30];

% define cost function
cost_function = @(x) GA_optim_fn(x, config, params, ref_signal);

% solver setup
options = optimoptions('gamultiobj', ...
    'PopulationSize', 100, ...         
    'MaxGenerations', 60, ...          
    'CrossoverFraction', 0.8, ...      
    'ParetoFraction', 0.35, ...        
    'UseParallel', true, ...           
    'Display', 'iter', ...             
    'PlotFcn', @gaplotpareto);         

% Initialise parallel computing
if isempty(gcp('nocreate'))
    disp('Starting Parallel Pool...');
    parpool;
end

% Set simulink model to fast restart mode
modelName = 'Model_PID';
if ~bdIsLoaded(modelName)
    load_system(modelName);
end
set_param(modelName, 'SignalLogging', 'off');
set_param(modelName, 'ReturnWorkspaceOutputs', 'on');
set_param(modelName, 'FastRestart', 'on');

% optimisation to create pareto optimisation frontier
disp('Optimisation Started');
tic;
[x_pareto, fval_pareto] = gamultiobj(cost_function, num_vars, ...
    [], [], [], [], lb, ub, [], options);
time_taken = toc;

% unlock model
set_param(modelName, 'FastRestart', 'off');
fprintf('Optimisation Complete in %.2f seconds.\n', time_taken);

% define weightings
w_MSE = 4/7;
w_Energy = 3/7;

% Normalise cost objectives to allow relative weigthing
raw_MSE = fval_pareto(:, 1);
raw_Energy = fval_pareto(:, 2);
norm_MSE = (raw_MSE - min(raw_MSE)) ./ (max(raw_MSE) - min(raw_MSE));
norm_Energy = (raw_Energy - min(raw_Energy)) ./ (max(raw_Energy) - min(raw_Energy));

% Calculate weighted cost for all points on the optimisation front
weighted_costs = (w_MSE .* norm_MSE) + (w_Energy .* norm_Energy);

% Select parameters with lowest cost
[min_cost, best_index] = min(weighted_costs);
optimal_gains = x_pareto(best_index, :);
optimal_costs = fval_pareto(best_index, :);

% print parameters
fprintf('\n--- Optimal Controller ---\n');
fprintf('Pareto Row Index: %d\n', best_index);
fprintf('Kp:      %.4f\n', optimal_gains(1));
fprintf('Ki:      %.4f\n', optimal_gains(2));
fprintf('Kd:      %.4f\n', optimal_gains(3));
fprintf('N:   %.4f\n', optimal_gains(4));
fprintf('psi:     %.4f\n', optimal_gains(5));
fprintf('-----------------------------------------\n');
fprintf('Expected MSE:   %.4f\n', optimal_costs(1));
fprintf('Expected Energy: %.4f\n', optimal_costs(2));