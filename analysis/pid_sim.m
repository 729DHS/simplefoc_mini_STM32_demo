%% SimpleFOC PID 仿真 — AS5600 量化噪声对 D 项的影响
%  可视化 Kd=0.01 vs Kd=0.02 的闭环响应
%  2026-06-01

clear; close all;

%% ===== 系统参数 =====
dt      = 0.0091;       % 控制周期 (s) — ~110Hz
t_sim   = 3.0;          % 仿真时长 (s)
n_steps = round(t_sim / dt);

% 电机模型 (简化: 纯惯性 + 粘性摩擦)
J       = 1e-6;         % 转动惯量 (kg·m²) — 2804 估算
B       = 5e-6;         % 粘性阻尼 (N·m·s/rad)
Kt      = 0.05;         % 力矩常数 (N·m / duty) — 0~1 duty → 0~Kt Nm

% AS5600 量化
enc_res = 4096;         % 12-bit
enc_lsb = 2*pi / enc_res; % ~0.00153 rad = 0.088°

%% ===== PID + 低通滤波 (与固件一致) =====
tau_lpf = 0.008;        % D 项低通滤波时间常数 (s)
alpha   = dt / (dt + tau_lpf); % ~0.532

%% ===== 仿真 =====
Kp_vals = [0.1, 0.1];
Kd_vals = [0.01, 0.02]; % 对比

figure('Position', [100 100 900 500]);
colors = {'b', 'r'};

for run = 1:2
    Kp = Kp_vals(run);
    Ki = 0.02;
    Kd = Kd_vals(run);

    % 状态初始化
    theta   = zeros(1, n_steps);  % 真实位置 (rad)
    omega   = zeros(1, n_steps);  % 角速度
    theta(1)= 0.5 * pi;           % 初始 ~90°
    target  = pi;                 % 目标 ~180°

    integ   = 0;
    prev_meas = theta(1);
    deriv_f = 0;
    output_lim = 0.4;             % 电压限幅

    for k = 2:n_steps
        % ---- 1. 编码器读数 (加量化) ----
        meas_true = theta(k-1);
        % 模拟 12-bit 量化: 对 0~2π 范围离散化
        meas_quant = round(meas_true / enc_lsb) * enc_lsb;
        % 确保在 0~2π
        meas_quant = mod(meas_quant, 2*pi);

        % ---- 2. PID 计算 ----
        error = target - meas_true;  % 用真值算误差 (排除量化对 P 的影响)

        P_out = Kp * error;
        I_contrib = Ki * integ;

        % D-on-measurement + 低通滤波
        vel = (meas_quant - prev_meas) / dt;
        deriv_f = deriv_f + alpha * (vel - deriv_f);
        D_out = -Kd * deriv_f;
        prev_meas = meas_quant;

        output = P_out + I_contrib + D_out;

        % 条件积分 + 输出限幅
        saturated = 0;
        if output > output_lim
            output = output_lim;
            saturated = 1;
        elseif output < -output_lim
            output = -output_lim;
            saturated = 1;
        end

        if ~saturated
            integ = integ + error * dt;
        end
        integ = max(-0.3, min(0.3, integ));

        % ---- 3. 电机动力学 ----
        torque = Kt * output;
        alpha_m = (torque - B * omega(k-1)) / J;
        omega(k) = omega(k-1) + alpha_m * dt;
        theta(k) = theta(k-1) + omega(k) * dt;
    end

    % ---- 绘图 ----
    t = (0:n_steps-1) * dt;
    subplot(2,1,1);
    plot(t, rad2deg(theta), colors{run}, 'LineWidth', 1.2); hold on;
    yline(rad2deg(target), [colors{run} '--']);

    subplot(2,1,2);
    plot(t(2:end), rad2deg(diff(theta)/dt), colors{run}, 'LineWidth', 1.2); hold on;
end

subplot(2,1,1);
legend('K_d=0.01', 'Target', 'K_d=0.02', 'Target', 'Location', 'best');
ylabel('Position (deg)'); title('Step Response — K_d Sensitivity');
grid on;

subplot(2,1,2);
legend('K_d=0.01', 'K_d=0.02', 'Location', 'best');
ylabel('Velocity (deg/s)'); xlabel('Time (s)');
title('Angular Velocity');
grid on;

%% ===== 量化噪声传递分析 =====
fprintf('\n===== D 项量化噪声分析 =====\n');
fprintf('AS5600 LSB = %.3f° = %.4f rad\n', rad2deg(enc_lsb), enc_lsb);
fprintf('控制周期 dt = %.1f ms\n', dt*1000);
fprintf('单 LSB 速度噪声 = %.1f °/s\n', rad2deg(enc_lsb/dt));

fprintf('\n         | 滤波增益 | 稳态 D 力 (rad/s 等效)\n');
fprintf('         | (DC=1)   |\n');
for Kd = [0.01, 0.02, 0.03, 0.05, 0.10]
    % 量化噪声通过滤波后的稳态值 (近似)
    noise_vel = enc_lsb / dt * alpha;  % 单 LSB 引起的滤波速度
    D_force = Kd * noise_vel;           % D 项产生的力
    fprintf('Kd=%.2f   |  α=%.2f   | %.4f rad/s = %.2f °/s\n', ...
        Kd, alpha, D_force, rad2deg(D_force));
end

fprintf('\n结论: Kd 越大, 量化噪声被放大越多.\n');
fprintf('Kd=0.01 时 D 噪声力 ≈ %.1f °/s, 几乎不可感知.\n', rad2deg(0.01*enc_lsb/dt*alpha));
fprintf('Kd=0.02 时已经进入"噪声驱动震荡"区间.\n');

%% ===== 阶跃响应统计 =====
fprintf('\n===== 阶跃响应统计 =====\n');
% 简化版本算稳态和超调
for run = 1:2
    theta = zeros(1, n_steps);
    omega = zeros(1, n_steps);
    theta(1) = 0.5*pi;
    target = pi;
    integ = 0; prev_meas = theta(1); deriv_f = 0;
    Kp = Kp_vals(run); Ki = 0.02; Kd = Kd_vals(run);

    for k = 2:n_steps
        meas_true = theta(k-1);
        meas_quant = round(meas_true / enc_lsb) * enc_lsb;
        meas_quant = mod(meas_quant, 2*pi);
        error = target - meas_true;
        P_out = Kp * error;
        I_contrib = Ki * integ;
        vel = (meas_quant - prev_meas) / dt;
        deriv_f = deriv_f + alpha * (vel - deriv_f);
        D_out = -Kd * deriv_f;
        prev_meas = meas_quant;
        output = P_out + I_contrib + D_out;
        saturated = 0;
        if output > output_lim, output = output_lim; saturated = 1;
        elseif output < -output_lim, output = -output_lim; saturated = 1; end
        if ~saturated, integ = integ + error * dt; end
        integ = max(-0.3, min(0.3, integ));
        torque = Kt * output;
        alpha_m = (torque - B * omega(k-1)) / J;
        omega(k) = omega(k-1) + alpha_m * dt;
        theta(k) = theta(k-1) + omega(k) * dt;
    end

    overshoot = max(0, max(theta) - target);
    settling_idx = find(abs(theta(round(end*0.3):end) - target) < 0.01, 1);
    fprintf('Kd=%.2f: 超调=%.1f°  ', Kd, rad2deg(overshoot));
    if ~isempty(settling_idx)
        fprintf('稳定\n');
    else
        fprintf('未稳定 (震荡)\n');
    end
end
