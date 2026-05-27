mu_k_array = linspace(0.11, 0.17, N_steps+1);
FS_array = linspace(0.05, 0.2, N_steps+1);
m_struct_array = linspace(0.007, 0.107, N_steps+1);
x_cg_array = linspace(0.00588, 0.01764, N_steps+1);
tau_array = linspace(0.003, 0.009, N_steps+1);

% line of best fits
% mu_k 0.1 -> 0.18
p1 = polyfit(mu_k_array, mu_k_res(1,:), 1);
p2 = polyfit(mu_k_array, mu_k_res(2,:), 1);

disp(p1)
disp("---------")
disp(p2)
disp("---------")

x_plot_mu = linspace(0.1, 0.18);
f1_plot_mu = polyval(p1, x_plot_mu);
f2_plot_mu = polyval(p2, x_plot_mu);

% FS_max 0.05 -> 0.2
p1 = polyfit(FS_array, Fs_res(1,:), 1);
p2 = polyfit(FS_array, Fs_res(2,:), 1);

disp(p1)
disp("---------")
disp(p2)
disp("---------")

x_plot_fs = linspace(0.05, 0.2);
f1_plot_fs = polyval(p1, x_plot_fs);
f2_plot_fs = polyval(p2, x_plot_fs);

% m_struct 0 -> 0.12
p1 = polyfit(m_struct_array, m_structure_res(1,:), 1);
p2 = polyfit(m_struct_array, m_structure_res(2,:), 1);

disp(p1)
disp("---------")
disp(p2)
disp("---------")

x_plot_ms = linspace(0, 0.12);
f1_plot_ms = polyval(p1, x_plot_ms);
f2_plot_ms = polyval(p2, x_plot_ms);

% x_cg 0.004 -> 0.018
p1 = polyfit(x_cg_array, x_cg_res(1,:), 1);
p2 = polyfit(x_cg_array, x_cg_res(2,:), 1);

disp(p1)
disp("---------")
disp(p2)
disp("---------")

x_plot_cg = linspace(0.004, 0.018);
f1_plot_cg = polyval(p1, x_plot_cg);
f2_plot_cg = polyval(p2, x_plot_cg);

% tau_m 0.003 -> 0.009
p1 = polyfit(tau_array, tau_res(1,:), 1);
p2 = polyfit(tau_array, tau_res(2,:), 1);

disp(p1)
disp("---------")
disp(p2)
disp("---------")

x_plot_tau = linspace(0.003, 0.009);
f1_plot_tau = polyval(p1, x_plot_tau);
f2_plot_tau = polyval(p2, x_plot_tau);

figure()
subplot(1,2,1)
hold on
scatter(mu_k_array, mu_k_res(1, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_mu, f1_plot_mu, 'k', 'LineWidth', 2)
hold off
xlabel("\mu_k")
ylabel("MSE (m^2)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on

subplot(1,2,2)
hold on
scatter(mu_k_array, mu_k_res(2, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_mu, f2_plot_mu, 'k', 'LineWidth', 2)
xlabel("\mu_k")
ylabel("Energy (kJ)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

figure()
subplot(1,2,1)
hold on
scatter(FS_array, Fs_res(1, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_fs, f1_plot_fs, 'k', 'LineWidth', 2)
xlabel("F_{smax}")
ylabel("MSE (m^2)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

subplot(1,2,2)
hold on
scatter(FS_array, Fs_res(2, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_fs, f2_plot_fs, 'k', 'LineWidth', 2)
xlabel("F_{smax}")
ylabel("Energy (kJ)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

figure()
subplot(1,2,1)
hold on
scatter(m_struct_array, m_structure_res(1, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_ms, f1_plot_ms, 'k', 'LineWidth', 2)
xlabel("m_{structure}")
ylabel("MSE (m^2)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

subplot(1,2,2)
hold on
scatter(m_struct_array, m_structure_res(2, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_ms, f2_plot_ms, 'k', 'LineWidth', 2)
xlabel("m_{structure}")
ylabel("Energy (kJ)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

figure()
subplot(1,2,1)
hold on
scatter(x_cg_array, x_cg_res(1, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_cg, f1_plot_cg, 'k', 'LineWidth', 2)
xlabel("x_{cg}")
ylabel("MSE (m^2)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

subplot(1,2,2)
hold on
scatter(x_cg_array, x_cg_res(2, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_cg, f2_plot_cg, 'k', 'LineWidth', 2)
xlabel("x_{cg}")
ylabel("Energy (kJ)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

figure()
subplot(1,2,1)
hold on
scatter(tau_array, tau_res(1, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_tau, f1_plot_tau, 'k', 'LineWidth', 2)
xlabel("\tau_m")
ylabel("MSE (m^2)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off

subplot(1,2,2)
hold on
scatter(tau_array, tau_res(2, :), 100, 'kx', 'LineWidth', 2)
plot(x_plot_tau, f2_plot_tau, 'k', 'LineWidth', 2)
xlabel("\tau_m")
ylabel("Energy (kJ)")
legend("Sampled Value", "Trend Caused by Variation", 'Location', 'best')
grid on
hold off