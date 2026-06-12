close all
clear all
clc

%% dp/dp_c 通道
% 参数，含模型参数与PID参数
disp('-----------------------------')
disp('滚转通道')

K = 0.4785

% 角速度
kp = 10.04
ki = 11.34
kd = 0.1

% dr/dr_c 传递函数
num1 = kp*K/(1+kd*K)
num2 = kp*K/(1+kd*K)*ki/kp

den1 = kp*K/(1+kd*K)
den2 = ki*K/(1+kd*K)

disp('dp/dp_c传递函数')
G_p_pc = tf([num1 num2],[1 den1 den2])

% 角度
% page145书上是2.3，我觉得不对。阻尼0.8，应该对应增益0.0137653
kt = 0.80025;

num1 = kt*kp*K/(1+kd*K);
num2 = kt*kp*K/(1+kd*K)*ki/kp;
den1 = kp*K/(1+kd*K);
den2 = (kt*kp+ki)*K/(1+kd*K);
den3 = kt*ki*K/(1+kd*K);
disp('dphi/dphi_c传递函数')
G_dphi_dphic = tf([num1 num2],[1 den1 den2 den3])

disp('-----------------------------')

%% dq/dq_c 通道
% 参数，含模型参数与PID参数
disp('-----------------------------')
disp('俯仰通道')

K = 0.3415

% 角速度
kp = 14.53
ki = 15.74
kd = 0.1

% dr/dr_c 传递函数
num1 = kp*K/(1+kd*K)
num2 = kp*K/(1+kd*K)*ki/kp

den1 = kp*K/(1+kd*K)
den2 = ki*K/(1+kd*K)

disp('dq/dq_c传递函数')
G_q_qc = tf([num1 num2],[1 den1 den2])

% 角度
kt = 0.883;

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

K = 0.007555

% 角速度
kp = 148.3582
ki = 64.9067
kd = 0.1

% dr/dr_c 传递函数
num1 = kp*K/(1+kd*K)
num2 = kp*K/(1+kd*K)*ki/kp

den1 = kp*K/(1+kd*K)
den2 = ki*K/(1+kd*K)

disp('dr/dr_c传递函数')
G_r_rc = tf([num1 num2],[1 den1 den2])

% 角度
% page145书上是2.3，我觉得不对。阻尼0.8，应该对应增益0.0137653
% kt = 2.3;
kt = 0.0137653;

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
s = tf('s');
G_s = 1/s;

Gol_dpsi_dpsic = G_r_rc*G_s;

zeta = 0.8;
figure;
rlocus(Gol_dpsi_dpsic);
grid on
sgrid(zeta, [])
title('dpsi/dpsi_c传递函数极点')



%%


