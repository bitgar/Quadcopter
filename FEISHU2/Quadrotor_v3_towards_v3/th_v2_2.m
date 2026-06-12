close all
clear all
clc

%% 符号引擎定义
s = tf('s');
G_s = 1/s;

%% dp/dp_c 通道
% 参数，含模型参数与PID参数
disp('-----------------------------')
disp('滚转通道')

K = 0.4785;

% 角速度
kp = 220.0613;
ki = 8637.4096;
kd = 0.1;

% dp/dp_c 传递函数
num1 = kp*K/(1+kd*K);
num2 = kp*K/(1+kd*K)*ki/kp;

den1 = kp*K/(1+kd*K);
den2 = ki*K/(1+kd*K);

disp('dp/dp_c传递函数')
G_p_pc = tf([num1 num2],[1 den1 den2])
% G_p_pc = tf([den2],[1 den1 den2])

dc_gain_G_p_pc = dcgain(G_p_pc);
fprintf('dp/dp_c DC Gain: %.4f\n', dc_gain_G_p_pc);

bw_G_p_pc = bandwidth(G_p_pc);
fprintf('G_p_pc Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G_p_pc, bw_G_p_pc/(2*pi));

figure('Name', 'Bode G_p_pc');
bode(G_p_pc, {0.1, 1000}); 
grid on;
title('Bode Plot for Gppc');

% 自动查找并标注峰值
[mag, phase, wout] = bode(G_p_pc, {0.1, 1000});
mag = squeeze(mag);
mag_db = 20*log10(mag);
[peak_val, peak_idx] = max(mag_db);
fprintf('Peak Gain: %.2f dB at %.2f rad/s\n', peak_val, wout(peak_idx));

disp('-----------------------------')

% 角度dphi/dphi_c闭环传递函数
kt = 40;

num1 = kt*kp*K/(1+kd*K);
num2 = kt*kp*K/(1+kd*K)*ki/kp;
den1 = kp*K/(1+kd*K);
den2 = (kt*kp+ki)*K/(1+kd*K);
den3 = kt*ki*K/(1+kd*K);
disp('dphi/dphi_c传递函数')
G_dphi_dphic = tf([num1 num2],[1 den1 den2 den3])

% 转换为零极点形式
[z, p, k] = tf2zp([num1 num2],[1 den1 den2 den3]);
% 显示结果
disp('零点 (Zeros):'); disp(z);
disp('极点 (Poles):'); disp(p);
disp('增益 (Gain):'); disp(k);
disp('-----------------------------')


%%

