close all
clear all
clc

%================= 参数设定 =================
J      = 0.05;            % 惯量 (示例)
wc_des = 20;              % 目标截止频率 [rad/s]
zeta   = 0.5;             % 期望阻尼比，可自行修改

%================= 优化搜索 =================
% 单变量搜索: ki
optfun = @(ki) ...
    20*log10(abs(freqresp(tf([2*sqrt(J*ki) ki],[J 2*sqrt(J*ki) ki]), wc_des))) + 3;
% 让幅值在 wc_des 处距离 -3 dB 为零

% ki0 = (wc_des)^2 * J;      % 初值
ki0 = 0.1;      % 初值
ki  = fzero(optfun, ki0);  % 求解 ki
kp  = 2*sqrt(J*ki);   % 对应 kp

fprintf('调节结果: kp = %.4f , ki = %.4f\n', kp, ki);

%================= 验证与绘图 =================
T = tf([kp ki],[J kp ki]);
figure;
margin(T);            % 画Bode图并显示增益交叉、相位裕度等
grid on;
