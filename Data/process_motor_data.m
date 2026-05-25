function [power, thrust, throttle] = process_motor_data(filename)
    % read the data
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve';
    data = readtable(filename, opts);
    
    esc_signal = data.("ESC signal (µs)");
    thrust = data.("Thrust (gf)");
    power = data.("Electrical Power (W)");
    throttle = (esc_signal - 1000) / 10;
    
    power = [0; power(10:(end-12)); 224.6];
    power = 2 * power;
    throttle = [0; throttle(10:(end-12)); 100];
    thrust = [0; thrust(10:(end-12)); 781.5];
    thrust = 2 * thrust * 9.81 / 1000;

    [throttle, unique_idx] = unique(throttle);
    thrust = thrust(unique_idx);
    power = power(unique_idx);
  
    valid_idx = true(size(thrust));
    current_max = thrust(1);
    
    for i = 2:length(thrust)
        if thrust(i) > current_max
            current_max = thrust(i); % Data went up, keep it.
        else
            valid_idx(i) = false;    % Sensor twitched down/flat, delete it.
        end
    end
    
    throttle = throttle(valid_idx);
    thrust = thrust(valid_idx);
    power = power(valid_idx);
end