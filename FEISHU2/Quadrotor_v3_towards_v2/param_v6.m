%% 传递函数分析程序
% 分析系统: G(s) = (0.629s + 0.7108) / (1.13s + 0.7108)
close all
clear all
clc

%% 1. 定义传递函数
num = [0.629, 0.7108];      % 分子系数 [0.629, 0.7108]
den = [1.13, 0.7108];       % 分母系数 [1.13, 0.7108]

G = tf(num, den);

disp('========== 原始传递函数 ==========');
disp(G);

%% 2. 转换为零极点形式 (ZPK)
[z, p, k] = tf2zp(num, den);

disp('========== 零极点形式 ==========');
fprintf('零点 z = %.4f\n', z);
fprintf('极点 p = %.4f\n', p);
fprintf('增益 k = %.4f\n', k);

% 创建 ZPK 对象
G_zpk = zpk(z, p, k);
disp('ZPK 模型:');
disp(G_zpk);

%% 3. 计算转折频率
omega_z = abs(z);           % 零点转折频率 (rad/s)
omega_p = abs(p);           % 极点转折频率 (rad/s)
f_z = omega_z / (2*pi);     % 零点转折频率 (Hz)
f_p = omega_p / (2*pi);     % 极点转折频率 (Hz)

disp('========== 转折频率 ==========');
fprintf('零点转折频率: %.4f rad/s (%.4f Hz)\n', omega_z, f_z);
fprintf('极点转折频率: %.4f rad/s (%.4f Hz)\n', omega_p, f_p);

%% 4. 绘制波特图
figure('Name', 'Bode Plot', 'Position', [100 100 800 600]);
bode(G);
grid on;
title('Bode Diagram of G(s) = (0.629s + 0.7108)/(1.13s + 0.7108)');

% 在图上标注转折频率
hold on;
[G_mag, G_phase, w] = bode(G);
G_mag = squeeze(G_mag);
G_phase = squeeze(G_phase);

% 找到最接近转折频率的点
[~, idx_z] = min(abs(w - omega_z));
[~, idx_p] = min(abs(w - omega_p));

%% 5. 绘制阶跃响应
figure('Name', 'Step Response', 'Position', [100 100 600 400]);
step(G);
grid on;
title('Step Response');
xlabel('Time (s)');
ylabel('Amplitude');

%% 6. 绘制零极点图
figure('Name', 'Pole-Zero Map', 'Position', [100 100 500 400]);
pzmap(G);
grid on;
title('Pole-Zero Map');

%% 7. 计算稳态增益和关键指标
disp('========== 系统指标 ==========');
dc_gain = dcgain(G);
fprintf('直流增益 (DC Gain): %.4f (%.2f dB)\n', dc_gain, 20*log10(dc_gain));

high_freq_gain = num(1)/den(1);
fprintf('高频增益: %.4f (%.2f dB)\n', high_freq_gain, 20*log10(high_freq_gain));

% 时间常数
tau_z = 1/omega_z;
tau_p = 1/omega_p;
fprintf('零点时间常数: %.4f s\n', tau_z);
fprintf('极点时间常数: %.4f s\n', tau_p);

%% 8. 奈奎斯特图
figure('Name', 'Nyquist Plot', 'Position', [100 100 500 400]);
nyquist(G);
grid on;
title('Nyquist Diagram');

disp('========== 分析完成 ==========');

%% 阶跃响应对比
num = [0.629, 0.7108];
den = [1.13, 0.7108];
G = tf(num, den)

num2 = [3.384, 3.822];
den2 = [1, 4.585, 8.562,3.822];
G2 = tf(num2, den2)

figure('Name', 'Step Response');
hold on
step(G);
step(G2);
grid on;
hold off
title('Step Response');
xlabel('Time (s)');
ylabel('Amplitude');


%%
