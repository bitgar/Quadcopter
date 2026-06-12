close all
clear all
clc

%% 符号引擎定义
s = tf('s');
G_s = 1/s;

%% 滚转通道
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

% 角度dphi/dphi_c闭环传递函数
kt = 40;

num1 = kt*kp*K/(1+kd*K);
num2 = kt*kp*K/(1+kd*K)*ki/kp;
den1 = kp*K/(1+kd*K);
den2 = (kt*kp+ki)*K/(1+kd*K);
den3 = kt*ki*K/(1+kd*K);
disp('dphi/dphi_c传递函数')
G_dphi_dphic = tf([num1 num2],[1 den1 den2 den3])
[num_G_dphi_dphic, den_G_dphi_dphic] = tfdata(G_dphi_dphic, 'v')

%% 横侧向动力学模型
V0 = 30;
theta0 = -0.4762;
K_psi_phi = -0.027;
K_beta_phi = -0.43121;
b_beta = -0.67395;
a_beta = -0.14107;
sin_theta0 = sin(theta0);

K_lambda_phi = K_psi_phi + K_beta_phi - sin_theta0
b_lambda = (K_beta_phi*b_beta - a_beta*(K_psi_phi-sin_theta0))/ ...
           (K_psi_phi + K_beta_phi - sin_theta0)

num = K_lambda_phi.*[1 b_lambda];
den = [1 -a_beta];

disp('dchi/dphi传递函数')
G_dchi_dphi = tf(num,den)

dc_gain_G_dchi_dphi = dcgain(G_dchi_dphi);
fprintf('dchi/dphi DC Gain: %.4f\n', dc_gain_G_dchi_dphi);

bw_G_dchi_dphi = bandwidth(G_dchi_dphi);
fprintf('G_dchi_dphi Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G_dchi_dphi, bw_G_dchi_dphi/(2*pi));

figure('Name', 'Bode G_dchi_dphi');
bode(G_dchi_dphi, {0.1, 1000}); 
grid on;
title('Bode Plot for G\_dchi\_dphi');

% 自动查找并标注峰值
[mag, phase, wout] = bode(G_dchi_dphi, {0.1, 1000});
mag = squeeze(mag);
mag_db = 20*log10(mag);
[peak_val, peak_idx] = max(mag_db);
fprintf('Peak Gain: %.2f dB at %.2f rad/s\n', peak_val, wout(peak_idx));

% -0.027   -0.43121+0.4584
% -0.027215-0.43121+0.4584
%% dyddot/dyddot_c
kyd = 1.8079;
beishu = V0*K_lambda_phi/(V0*K_lambda_phi*kyd+1);
% num = beishu .* [1 b_lambda];
% den = [1 (V0*K_lambda_phi*b_lambda*kyd-a_beta)/(V0*K_lambda_phi*kyd+1)];

z1 = -sin_theta0*4020;
z2 = -sin_theta0*157800;

p1 = 100.5;
p2 = 7964;
p3 = 157800;

% z1 = -sin_theta0*4353;
% z2 = -sin_theta0*134800;
% 
% p1 = 100.3;
% p2 = 7755;
% p3 = 134900;

num = [V0*z1 V0*z2];
den = [1 p1 (p2+V0*kyd*z1) p3+V0*kyd*z2];

% num = V0.*[num_G_dphi_dphic(1) num_G_dphi_dphic(2)];
% den = [1 den_G_dphi_dphic(1) (den_G_dphi_dphic(2)+V0*kyd) (den_G_dphi_dphic(3)+V0*kyd*num_G_dphi_dphic(2))];

disp('dyddot/dyddot_c传递函数')
G_dyddot_dyddot_c = tf(num,den)
[num_G_dyddot_dyddot_c, den_G_dyddot_dyddot_c] = tfdata(G_dyddot_dyddot_c, 'v');

dc_gain_G_dyddot_dyddot_c = dcgain(G_dyddot_dyddot_c);
fprintf('dyddot/dyddot_c DC Gain: %.4f\n', dc_gain_G_dyddot_dyddot_c);

bw_G_dyddot_dyddot_c = bandwidth(G_dyddot_dyddot_c);
fprintf('G_dyddot_dyddot_c Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G_dyddot_dyddot_c, bw_G_dyddot_dyddot_c/(2*pi));

figure('Name', 'Bode G_dchi_dphi');
bode(G_dyddot_dyddot_c, {0.1, 1000}); 
grid on;
title('Bode Plot for G\_dyddot\_dyddotc');

% 自动查找并标注峰值
[mag, phase, wout] = bode(G_dyddot_dyddot_c, {0.1, 1000});
mag = squeeze(mag);
mag_db = 20*log10(mag);
[peak_val, peak_idx] = max(mag_db);
fprintf('Peak Gain: %.2f dB at %.2f rad/s\n', peak_val, wout(peak_idx));


%% dyd/dyd_c
ky = 15;
beishu = (V0*K_lambda_phi*ky)/(V0*K_lambda_phi*kyd+1);
% num = beishu .* [1 b_lambda];
% den = [1 (V0*K_lambda_phi*b_lambda*kyd-a_beta+V0*K_lambda_phi*ky)/(V0*K_lambda_phi*kyd+1) (V0*K_lambda_phi*ky*b_lambda)/(V0*K_lambda_phi*kyd+1)];

num = [ky*num_G_dyddot_dyddot_c(3) ky*num_G_dyddot_dyddot_c(4)]
den = [1 den_G_dyddot_dyddot_c(2) den_G_dyddot_dyddot_c(3) (den_G_dyddot_dyddot_c(4)+ky*num_G_dyddot_dyddot_c(3)) ky*num_G_dyddot_dyddot_c(4)]
G_dyd_dydc = tf(num,den)

% (V0*K_lambda_phi*b_lambda*kyd-a_beta)/(V0*K_lambda_phi*kyd+1)
% (V0*K_lambda_phi)/(V0*K_lambda_phi*kyd+1)

disp('dyd/dyd_c传递函数')
G_dyd_dyd_c = tf(num,den)

dc_gain_G_dyd_dyd_c = dcgain(G_dyd_dyd_c);
fprintf('dyd/dyd_c DC Gain: %.4f\n', dc_gain_G_dyd_dyd_c);

bw_G_dyd_dyd_c = bandwidth(G_dyd_dyd_c);
fprintf('G_dyd_dyd_c Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G_dyd_dyd_c, bw_G_dyd_dyd_c/(2*pi));

% figure('Name', 'Bode G_dyd_dyd_c');
% bode(G_dyd_dyd_c, {0.1, 1000}); 
% grid on;
% title('Bode Plot for G\_dyd\_dydc');
% 
% % 自动查找并标注峰值
% [mag, phase, wout] = bode(G_dyd_dyd_c, {0.1, 1000});
% mag = squeeze(mag);
% mag_db = 20*log10(mag);
% [peak_val, peak_idx] = max(mag_db);
% fprintf('Peak Gain: %.2f dB at %.2f rad/s\n', peak_val, wout(peak_idx));
%%

