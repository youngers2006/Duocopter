clear
clc

tau = process_motor_data("Small_Motor_data.csv");

function process_motor_data(filename)
    % process_motor_data: Cleans and maps motor CSV data to a 0-100 scale.
    % Usage: tau = process_motor_data('Small_Motor_data.csv')
    
    disp(['Processing data from: ', filename]);

    % 1. Read the data safely keeping column headers intact
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve';
    data = readtable(filename, opts);
    
    % Extract raw variables
    raw_pwm = data.("ESC signal (µs)");
    raw_thrust = data.("Thrust (gf)");
    raw_power = data.("Electrical Power (W)");
    
    % 2. Filter valid operational range (1000 µs to 2000 µs)
    valid_idx = (raw_pwm >= 1000) & (raw_pwm <= 2000);
    pwm_valid = raw_pwm(valid_idx);
    thrust_valid = raw_thrust(valid_idx);
    power_valid = raw_power(valid_idx);
    
    % 3. Map PWM to a mathematical 0-100% scale
    throttle_raw = (pwm_valid - 1000) / 10;
    
    % 4. Handle sensor noise and duplicates using accumarray (mean average)
    [u_throttle, ~, idx] = unique(throttle_raw);
    u_thrust = accumarray(idx, thrust_valid, [], @mean);
    u_power = accumarray(idx, power_valid, [], @mean);
    
    % 5. Create a clean, strict 0 to 100 integer domain
    clean_throttle = (0:100)';
    
    % 6. Interpolate the raw data onto the clean integer domain
    clean_thrust = interp1(u_throttle, u_thrust, clean_throttle, 'linear', 'extrap');
    clean_power = interp1(u_throttle, u_power, clean_throttle, 'linear', 'extrap');
    
    % Clean up extrapolation artifacts (prevent negative thrust/power at 0% throttle)
    clean_thrust(clean_thrust < 0) = 0;
    clean_power(clean_power < 0) = 0;
    
    % 7. Package into tables and export to CSV
    thrust_table = table(clean_throttle, clean_thrust, ...
        'VariableNames', {'Throttle_Percent', 'Thrust_gf'});
    power_table = table(clean_throttle, clean_power, ...
        'VariableNames', {'Throttle_Percent', 'Power_W'});
    
    writetable(thrust_table, 'Clean_Throttle_Thrust.csv');
    writetable(power_table, 'Clean_Throttle_Power.csv');
    disp('Successfully generated: Clean_Throttle_Thrust.csv');
    disp('Successfully generated: Clean_Throttle_Power.csv');
end