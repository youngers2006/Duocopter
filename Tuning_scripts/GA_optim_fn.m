function [J] = GA_optim_fn(x, config, params, ref_signal)
% x = [KP_o, KI_o, KD_o, N_o, KP_i, KI_i, KD_i, N_i]
    % unpack variables
    Kp = x(1);
    Ki = x(2);
    Kd = x(3);
    N = x(4);
    phi = x(5);
    modelName = 'Model_PID';
    load_system(modelName);
    assignin('base', 'config', config);
    assignin('base', 'params', params);

    simIn_nom = Simulink.SimulationInput(modelName);

    profile = 1;
    step_height_nom = 0.7;
    step_height_wcl = 0.3;
    step_height_wch = 0.114;

    % sim for nominal configuration
    profile_ramp = 3;
    simIn_nom = setVariable(simIn_nom, 'Kp', Kp);
    simIn_nom = setVariable(simIn_nom, 'Ki', Ki);
    simIn_nom = setVariable(simIn_nom, 'Kd', Kd);
    simIn_nom = setVariable(simIn_nom, 'N', N);
    simIn_nom = setVariable(simIn_nom, 'phi', phi);
    simIn_nom = setVariable(simIn_nom, 'profile', profile_ramp);
    simIn_nom = setVariable(simIn_nom, 'step_height', step_height_nom);
    simIn_nom = setVariable(simIn_nom, 'ref_signal', ref_signal);
    simIn_nom = setModelParameter(simIn_nom, 'TimeOut', 30);
    
    try
        out_nom = sim(simIn_nom);
        mse_signal = out_nom.MSE;
        mse_array = mse_signal.Data;
        final_mse = mse_array(end);
        energy_signal = out_nom.energy;
        energy_array = energy_signal.Data;
        final_energy = energy_array(end);
        J1_mse = final_mse;
        J1_en = final_energy;
    catch
        J1_mse = 1e6;
        J1_en = 1e6;
    end

    simIn_wcl = Simulink.SimulationInput(modelName);

    % sim for light rise
    simIn_wcl = setVariable(simIn_wcl, 'Kp', Kp);
    simIn_wcl = setVariable(simIn_wcl, 'Ki', Ki);
    simIn_wcl = setVariable(simIn_wcl, 'Kd', Kd);
    simIn_wcl = setVariable(simIn_wcl, 'N', N);
    simIn_wcl = setVariable(simIn_wcl, 'phi', phi);
    simIn_wcl = setVariable(simIn_wcl, 'profile', profile);
    simIn_wcl = setVariable(simIn_wcl, 'step_height', step_height_wcl);
    simIn_wcl = setVariable(simIn_wcl, 'ref_signal', ref_signal);
    simIn_wcl = setModelParameter(simIn_wcl, 'TimeOut', 30);
    
    try
        out_wcl = sim(simIn_wcl);
        mse_signal = out_wcl.MSE;
        mse_array = mse_signal.Data;
        final_mse = mse_array(end);
        energy_signal = out_wcl.energy;
        energy_array = energy_signal.Data;
        final_energy = energy_array(end);
        J2_mse = final_mse;
        J2_en = final_energy;
    catch
        J2_mse = 1e6;
        J2_en = 1e6;
    end

    simIn_wch = Simulink.SimulationInput(modelName);

    % sim for heavy rise
    config_h = config;
    config_h.initial_height = 0.0;
    simIn_wch = setVariable(simIn_wch, 'config', config_h);
    simIn_wch = setVariable(simIn_wch, 'Kp', Kp);
    simIn_wch = setVariable(simIn_wch, 'Ki', Ki);
    simIn_wch = setVariable(simIn_wch, 'Kd', Kd);
    simIn_wch = setVariable(simIn_wch, 'N', N);
    simIn_wch = setVariable(simIn_wch, 'phi', phi);
    simIn_wch = setVariable(simIn_wch, 'profile', profile);
    simIn_wch = setVariable(simIn_wch, 'step_height', step_height_wch);
    simIn_wch = setVariable(simIn_wch, 'ref_signal', ref_signal);
    simIn_wch = setModelParameter(simIn_wch, 'TimeOut', 30);
    
    try
        out_wch = sim(simIn_wch);
        mse_signal = out_wch.MSE;
        mse_array = mse_signal.Data;
        final_mse = mse_array(end);
        energy_signal = out_wch.energy;
        energy_array = energy_signal.Data;
        final_energy = energy_array(end);
        J3_mse = final_mse;
        J3_en = final_energy;
    catch
        J3_mse = 1e6;
        J3_en = 1e6;
    end
        
    % Total cost
    Total_MSE = J1_mse + J2_mse + J3_mse;
    Total_Energy = J1_en + J2_en + J3_en;
    J = [Total_MSE, Total_Energy];
end