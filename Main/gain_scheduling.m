function [K, z_arr] = gain_scheduling(Q, R, mu_k, L_cg, L_c, L_m, h_min, h_max, dh, Ts, M0, lin_density, thrust_array, throttle_array)
% Calculate a gain schedule for the drone given the cost matrices and spacing
    N = floor(h_max / dh);
    Cc = [1, 0];
    Dc = 0;
    K = zeros(N+1, 3);
    z_arr = zeros(N+1, 1);

    P_thrust = polyfit(throttle_array, thrust_array, 2);
    P_slope = polyder(P_thrust);

    for i = 0:N
        z = h_min + i * dh;
        z_arr(i+1) = z;
        m = (M0 + lin_density * z);
        A21 = - ((lin_density * 9.81) / m) * (1 - mu_k * (L_cg / L_c));
        thrust = m * 9.81;

        hover_throttle = round(interp1(thrust_array, ...
            throttle_array, thrust, 'linear', 'extrap'));
        dT_du_hover = polyval(P_slope, hover_throttle);
        B2 = (1 - mu_k * (L_m) / (L_c)) * (dT_du_hover / m); 

        Ac = [0  ,   1;
              A21,   0];
        Bc = [0; B2];

        sys_c = ss(Ac, Bc, Cc, Dc);
        sys_d = c2d(sys_c, Ts, 'zoh');

        Ad = sys_d.A;
        Bd = sys_d.B;

        A_aug = [Ad,      zeros(2,1);
                 -Cc * Ts, 1];
        B_aug = [Bd; 0];

        K(i+1, :) = dlqr(A_aug, B_aug, Q, R);
    end
end