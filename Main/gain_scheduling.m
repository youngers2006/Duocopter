function K = gain_scheduling(Q, R, h_min, h_max, dh, Ts, M0, lin_density)
% Calculate a gain schedule for the drone given the cost matrices and spacing
    N = h_max / dh;
    Cc = [1, 0];
    Dc = 0;
    K = zeros(N+1, 1, 3);

    for i = 0:N
        z = h_min + i * dh;
        Ac = 
        Bc = 

        sys_c = ss(Ac, Bc, Cc, Dc);
        sys_d = c2d(ss, Ts, 'zoh');

        Ad = sys_d.A;
        Bd = sys_d.B;

        A_aug = [Ad,      0;
                 Cd * Ts, 1];
        B_aug = [Bd; 0];

        K(i+1, :, :) = dlqr(A_aug, B_aug, Q, R);
end