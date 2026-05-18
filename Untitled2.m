% =========================================================================
% DEMO INTERAKTIF REAL-TIME: PID CONTROLLER PITCH QUADCOPTER
% =========================================================================
clear; clc; close all;

% 1. Persiapan Jendela Simulasi (GUI)
fig = figure('Name', 'Dashboard Simulasi Quadcopter', 'NumberTitle', 'off', ...
             'Position', [100, 100, 800, 600], 'Color', 'w');

% Membuat area grafik (Axis)
ax = axes('Position', [0.1, 0.45, 0.8, 0.45]);
axis([-2 2 -2 2]); hold on; grid on;
title('Animasi Real-Time Sudut Pitch Quadcopter', 'FontSize', 14);
xlabel('Sumbu X'); ylabel('Sumbu Z (Ketinggian)');

% Menggambar Drone dan Garis Target
garisTarget = plot([-1.5 1.5], [0 0], 'r--', 'LineWidth', 1.5); % Garis referensi
lenganDrone = plot([-1 1], [0 0], 'b', 'LineWidth', 6);         % Lengan drone
bodiDrone   = plot(0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'k'); % Tengah drone
txtAktual   = text(-1.8, 1.5, 'Sudut Aktual: 0°', 'FontSize', 12, 'FontWeight', 'bold');

% 2. Membuat Tombol Slider Interaktif
% Slider Target Sudut (-45 s/d 45 derajat)
uicontrol('Style', 'text', 'Position', [50, 200, 120, 20], 'String', 'Target Pitch (Derajat):', 'BackgroundColor', 'w');
sl_target = uicontrol('Style', 'slider', 'Min', -45, 'Max', 45, 'Value', 0, 'Position', [180, 200, 400, 20]);
txt_target = uicontrol('Style', 'text', 'Position', [590, 200, 50, 20], 'String', '0', 'BackgroundColor', 'w');

% Slider Proportional (Kp)
uicontrol('Style', 'text', 'Position', [50, 150, 120, 20], 'String', 'Kp (Proportional):', 'BackgroundColor', 'w');
sl_Kp = uicontrol('Style', 'slider', 'Min', 0, 'Max', 5, 'Value', 1.5, 'Position', [180, 150, 400, 20]);
txt_Kp = uicontrol('Style', 'text', 'Position', [590, 150, 50, 20], 'String', '1.5', 'BackgroundColor', 'w');

% Slider Integral (Ki)
uicontrol('Style', 'text', 'Position', [50, 100, 120, 20], 'String', 'Ki (Integral):', 'BackgroundColor', 'w');
sl_Ki = uicontrol('Style', 'slider', 'Min', 0, 'Max', 2, 'Value', 0, 'Position', [180, 100, 400, 20]);
txt_Ki = uicontrol('Style', 'text', 'Position', [590, 100, 50, 20], 'String', '0', 'BackgroundColor', 'w');

% Slider Derivative (Kd)
uicontrol('Style', 'text', 'Position', [50, 50, 120, 20], 'String', 'Kd (Derivative):', 'BackgroundColor', 'w');
sl_Kd = uicontrol('Style', 'slider', 'Min', 0, 'Max', 5, 'Value', 0.5, 'Position', [180, 50, 400, 20]);
txt_Kd = uicontrol('Style', 'text', 'Position', [590, 50, 50, 20], 'String', '0.5', 'BackgroundColor', 'w');

% 3. Inisialisasi Fisika dan Matematika Plant
Iy = 0.0082;        % Momen Inersia
b = 0.05;           % Redaman
dt = 0.05;          % Resolusi waktu (detik)

theta = 0;          % Posisi sudut saat ini (Radian)
theta_dot = 0;      % Kecepatan sudut saat ini
error_integral = 0; % Akumulasi error untuk I
error_sebelumnya = 0;

% 4. Loop Simulasi Real-Time (Berjalan selama jendela terbuka)
while ishandle(fig)
    % A. Membaca input dari Slider pengguna
    target_deg = get(sl_target, 'Value');
    Kp = get(sl_Kp, 'Value');
    Ki = get(sl_Ki, 'Value');
    Kd = get(sl_Kd, 'Value');
    
    % Update teks di layar agar pengguna tahu nilainya
    set(txt_target, 'String', sprintf('%.1f°', target_deg));
    set(txt_Kp, 'String', sprintf('%.2f', Kp));
    set(txt_Ki, 'String', sprintf('%.2f', Ki));
    set(txt_Kd, 'String', sprintf('%.2f', Kd));
    
    target_rad = deg2rad(target_deg);
    
    % B. Algoritma PID Controller
    error = target_rad - theta;
    error_integral = error_integral + (error * dt);
    
    % Anti-Windup (Mencegah integral error menumpuk terlalu ekstrem)
    error_integral = max(min(error_integral, 5), -5); 
    
    error_derivatif = (error - error_sebelumnya) / dt;
    error_sebelumnya = error;
    
    % Output Kontroler (Sinyal/Torsi yang dikirim ke motor)
    u = (Kp * error) + (Ki * error_integral) + (Kd * error_derivatif);
    
    % Limitasi Torsi Motor (Drone di dunia nyata memiliki batas tenaga)
    u = max(min(u, 0.5), -0.5); 
    
    % C. Menyelesaikan Persamaan Diferensial Fisika Plant (Metode Euler)
    % Rumus: Iy * PercepatanSudut + b * KecepatanSudut = u
    theta_ddot = (u - (b * theta_dot)) / Iy;
    
    theta_dot = theta_dot + (theta_ddot * dt); % Update kecepatan
    theta = theta + (theta_dot * dt);          % Update posisi
    
    % D. Memperbarui Animasi Grafik
    L = 1; % Panjang lengan ilustrasi drone
    
    % Update kordinat kemiringan Drone
    x_drone = [-L*cos(theta), L*cos(theta)];
    y_drone = [-L*sin(theta), L*sin(theta)];
    set(lenganDrone, 'XData', x_drone, 'YData', y_drone);
    
    % Update kordinat garis Target (Referensi)
    x_target = [-1.5*cos(target_rad), 1.5*cos(target_rad)];
    y_target = [-1.5*sin(target_rad), 1.5*sin(target_rad)];
    set(garisTarget, 'XData', x_target, 'YData', y_target);
    
    % Update teks status aktual
    set(txtAktual, 'String', sprintf('Sudut Aktual: %.1f°', rad2deg(theta)));
    
    % Jeda sejenak untuk mensimulasikan waktu nyata (Real-time)
    pause(dt);
end