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

K = 0.4785

% 角速度
kp = 10.04
ki = 11.34
kd = 0.1
% kp = 40.13
% ki = 372.143
% kd = 0.1

% dp/dp_c 传递函数
num1 = kp*K/(1+kd*K)
num2 = kp*K/(1+kd*K)*ki/kp

den1 = kp*K/(1+kd*K)
den2 = ki*K/(1+kd*K)

disp('dp/dp_c传递函数')
G_p_pc = tf([num1 num2],[1 den1 den2])

% 角度dphi/dphi_c闭环传递函数
% kt = 0.80025;
kt = 0.738;

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
Gol_dpsi_dpsic = G_r_rc*G_s;

zeta = 0.8;
figure;
rlocus(Gol_dpsi_dpsic);
grid on
sgrid(zeta, [])
title('dpsi/dpsi_c传递函数极点')



%%


