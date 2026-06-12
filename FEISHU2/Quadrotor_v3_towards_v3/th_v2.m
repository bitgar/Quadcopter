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

% 分析dphi/dphi_c开环传递函数
G_ol_dphi_dphic = G_p_pc*G_s;
zeta = 0.8;
figure;
rlocus(G_ol_dphi_dphic);
grid on
sgrid(zeta, [])
title('dphi/dphi_c传递函数极点')

figure
pzmap(G_dphi_dphic)
grid on
title('dphi/dphi_c闭环传递函数零极点')

% -------------------------------------------------------------------
% 新增：分析 G_dphi_dphic (滚转闭环)
% -------------------------------------------------------------------
disp('==================================================');
disp('Analyzing Final Closed Loop G_dphi_dphic...');

% 1. 基本指标
fprintf('DC Gain: %.4f\n', dcgain(G_dphi_dphic));
bw_dphi_dphic = bandwidth(G_dphi_dphic);
fprintf('Bandwidth: %.4f rad/s (%.2f Hz)\n', bw_dphi_dphic, bw_dphi_dphic/(2*pi));

% 2. 极点主导性分析
poles_dphi_dphic = pole(G_dphi_dphic);
zeros_dphi_dphic = zero(G_dphi_dphic);

disp('Poles:');
disp(poles_dphi_dphic);
disp('Zeros:');
disp(zeros_dphi_dphic);

% 计算主导极点比率
real_parts_dphi_dphic = abs(real(poles_dphi_dphic));
min_real_dphi_dphic = min(real_parts_dphi_dphic); % 主极点 (最慢)
other_real = max(real_parts_dphi_dphic); % 远极点 (最快)
ratio = other_real / min_real_dphi_dphic;

fprintf('Dominant Pole Real Part: %.2f\n', min_real_dphi_dphic);
fprintf('Far Pole Real Part     : %.2f\n', other_real);
fprintf('Separation Ratio       : %.2f (Rule of thumb: >3-5 implies strong dominance)\n', ratio);

if ratio > 2.0
    disp('Conclusion: System is strongly dominated by the real pole at -16.76.');
    disp('            It will behave like a 1st order system with minimal overshoot.');
else
    disp('Conclusion: Coupled dynamics, complex poles have significant effect.');
end

% 3. 绘图验证
figure('Name', 'G_dphi_dphic Analysis');
subplot(2,1,1);
bode(G_dphi_dphic);
grid on;
title('Bode Plot: G\_dphi\_dphic');

subplot(2,1,2);
step(G_dphi_dphic);
grid on;
title('Step Response: G\_dphi\_dphic');
legend('G\_final');

% 标注阶跃响应特性
info = stepinfo(G_dphi_dphic);
fprintf('Step Response Info:\n');
fprintf('  Rise Time: %.4f s\n', info.RiseTime);
fprintf('  Overshoot: %.2f %%\n', info.Overshoot);
fprintf('  Settling Time (2%%): %.4f s\n', info.SettlingTime);
disp('-----------------------------')

%% dq/dq_c 通道
disp('俯仰通道')

K = 0.3415

kp = 304.5454
ki = 11951.5151
kd = 0.1

% dq/dq_c 传递函数
num1 = kp*K/(1+kd*K)
num2 = kp*K/(1+kd*K)*ki/kp

den1 = kp*K/(1+kd*K)
den2 = ki*K/(1+kd*K)

disp('dq/dq_c传递函数')
G_q_qc = tf([num1 num2],[1 den1 den2])

% 角度
kt = 40;

num1 = kt*kp*K/(1+kd*K);
num2 = kt*kp*K/(1+kd*K)*ki/kp;
den1 = kp*K/(1+kd*K);
den2 = (kt*kp+ki)*K/(1+kd*K);
den3 = kt*ki*K/(1+kd*K);
disp('dtheta/dtheta_c传递函数')
G_dtheta_dthetac = tf([num1 num2],[1 den1 den2 den3])

disp('-----------------------------')

%% dr/dr_c 通道
% 参数，含模型参数与PID参数
disp('-----------------------------')
disp('偏航通道')

K = 0.007555;

% 角速度
kp = 6718.6969;
ki = 132072.3649;
kd = 0.1;

% dr/dr_c 传递函数
num1 = kp*K/(1+kd*K)
num2 = kp*K/(1+kd*K)*ki/kp

den1 = kp*K/(1+kd*K)
den2 = ki*K/(1+kd*K)

disp('dr/dr_c传递函数')
G_r_rc = tf([num1 num2],[1 den1 den2])

dc_gain_G_r_rc = dcgain(G_r_rc);
fprintf('dr/dr_c DC Gain: %.4f\n', dc_gain_G_r_rc);

bw_G_r_rc = bandwidth(G_r_rc);
fprintf('G_r_rc Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G_r_rc, bw_G_r_rc/(2*pi));

figure('Name', 'Bode G_r_rc');
bode(G_r_rc, {0.1, 1000}); 
grid on;
title('Bode Plot for Grrc');

% 自动查找并标注峰值
[mag, phase, wout] = bode(G_r_rc, {0.1, 1000});
mag = squeeze(mag);
mag_db = 20*log10(mag);
[peak_val, peak_idx] = max(mag_db);
fprintf('Peak Gain: %.2f dB at %.2f rad/s\n', peak_val, wout(peak_idx));

% 角度
kt = 20;

num1 = kt*kp*K/(1+kd*K);
num2 = kt*kp*K/(1+kd*K)*ki/kp;
den1 = kp*K/(1+kd*K);
den2 = (kt*kp+ki)*K/(1+kd*K);
% den3 = kt*ki/kd;
den3 = kt*ki*K/(1+kd*K);
disp('dpsi/dpsi_c传递函数')
G_dpsi_dpsic = tf([num1 num2],[1 den1 den2 den3])

disp('-----------------------------')

% 绘制根轨迹，计算阻尼比为0.8时的参数
Gol_dpsi_dpsic = G_r_rc*G_s;

zeta = 0.8;
figure;
rlocus(Gol_dpsi_dpsic);
grid on
sgrid(zeta, [])
title('dpsi/dpsi_c传递函数极点')

% -------------------------------------------------------------------
% 分析 G_dpsi_dpsic (偏航闭环)
% -------------------------------------------------------------------
disp('==================================================');
disp('Analyzing Final Closed Loop G_dpsi_dpsic...');

% 1. 基本指标
fprintf('DC Gain: %.4f\n', dcgain(G_dpsi_dpsic));
bw_dpsi_dpsic = bandwidth(G_dpsi_dpsic);
fprintf('Bandwidth: %.4f rad/s (%.2f Hz)\n', bw_dpsi_dpsic, bw_dpsi_dpsic/(2*pi));

% 2. 极点主导性分析
poles_dpsi_dpsic = pole(G_dpsi_dpsic);
zeros_dpsi_dpsic = zero(G_dpsi_dpsic);

disp('Poles:');
disp(poles_dpsi_dpsic);
disp('Zeros:');
disp(zeros_dpsi_dpsic);

% 计算主导极点比率
real_parts_dpsi_dpsic = abs(real(poles_dpsi_dpsic));
min_real_dpsi_dpsic = min(real_parts_dpsi_dpsic); % 主极点 (最慢)
other_real = max(real_parts_dpsi_dpsic); % 远极点 (最快)
ratio = other_real / min_real_dpsi_dpsic;

fprintf('Dominant Pole Real Part: %.2f\n', min_real_dpsi_dpsic);
fprintf('Far Pole Real Part     : %.2f\n', other_real);
fprintf('Separation Ratio       : %.2f (Rule of thumb: >3-5 implies strong dominance)\n', ratio);

if ratio > 2.0
    disp('Conclusion: System is strongly dominated by the real pole at -16.76.');
    disp('            It will behave like a 1st order system with minimal overshoot.');
else
    disp('Conclusion: Coupled dynamics, complex poles have significant effect.');
end

% 3. 绘图验证
figure('Name', 'G_dpsi_dpsic Analysis');
subplot(2,1,1);
bode(G_dpsi_dpsic);
grid on;
title('Bode Plot: G\_dpsi\_dpsic');

subplot(2,1,2);
step(G_dpsi_dpsic);
grid on;
title('Step Response: G\_dpsi\_dpsic');
legend('G\_final');

% 标注阶跃响应特性
info = stepinfo(G_dpsi_dpsic);
fprintf('Step Response Info:\n');
fprintf('  Rise Time: %.4f s\n', info.RiseTime);
fprintf('  Overshoot: %.2f %%\n', info.Overshoot);
fprintf('  Settling Time (2%%): %.4f s\n', info.SettlingTime);
disp('-----------------------------')

%% 横侧向动力学模型
V0 = 30;
theta0 = -0.4762;
% K_psi_phi = -0.02721
K_psi_phi = -0.027;
K_beta_phi = -0.43121;
b_beta = -0.67395;
a_beta = -0.14107;
% sin_theta0 = -0.45849;
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
num = beishu .* [1 b_lambda];
den = [1 (V0*K_lambda_phi*b_lambda*kyd-a_beta)/(V0*K_lambda_phi*kyd+1)];

disp('dyddot/dyddot_c传递函数')
G_dyddot_dyddot_c = tf(num,den)

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
num = beishu .* [1 b_lambda];
den = [1 (V0*K_lambda_phi*b_lambda*kyd-a_beta+V0*K_lambda_phi*ky)/(V0*K_lambda_phi*kyd+1) (V0*K_lambda_phi*ky*b_lambda)/(V0*K_lambda_phi*kyd+1)];

% (V0*K_lambda_phi*b_lambda*kyd-a_beta)/(V0*K_lambda_phi*kyd+1)
% (V0*K_lambda_phi)/(V0*K_lambda_phi*kyd+1)

disp('dyd/dyd_c传递函数')
G_dyd_dyd_c = tf(num,den)

dc_gain_G_dyd_dyd_c = dcgain(G_dyd_dyd_c);
fprintf('dyd/dyd_c DC Gain: %.4f\n', dc_gain_G_dyd_dyd_c);

bw_G_dyd_dyd_c = bandwidth(G_dyd_dyd_c);
fprintf('G_dyd_dyd_c Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G_dyd_dyd_c, bw_G_dyd_dyd_c/(2*pi));

figure('Name', 'Bode G_dyd_dyd_c');
bode(G_dyd_dyd_c, {0.1, 1000}); 
grid on;
title('Bode Plot for G\_dyd\_dydc');

% 自动查找并标注峰值
[mag, phase, wout] = bode(G_dyd_dyd_c, {0.1, 1000});
mag = squeeze(mag);
mag_db = 20*log10(mag);
[peak_val, peak_idx] = max(mag_db);
fprintf('Peak Gain: %.2f dB at %.2f rad/s\n', peak_val, wout(peak_idx));
%%

