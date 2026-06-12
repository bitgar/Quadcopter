close all
clear all
clc

%% 滚转角通道
s = tf('s');

G1  = (s + 1.083) / (s^2 + 4.104*s + 6.557);
G2  = 4.2144 / (s + 0.6963);
disp('dphi/dphi_c传递函数为')
L = G1*G2
%%

theta0 = -27.29/180*pi;
v0 =30;
kyd= 0.05;
z1 = 0.6963;
z2 = 0.7541;
p1 = 1.083;
p2 = 0.7541;

num1 = -sin(theta0)*v0*z1;
num2 = -sin(theta0)*v0*z2;
den1 = p1-kyd*sin(theta0)*v0*z1;
den2 = p2-kyd*sin(theta0)*v0*z2;

num = [num1 num2];
den = [den1 den2];

disp('dy_d_dot/dy_dc_dot传递函数为')
T = tf(num,den)
 
%% ================= 已知参数 =================
a = 9.577;
b = 10.37;
c = 1.562;
d = 1.273;

target_wn = 0.305;
ky = target_wn^2*c/b;

%% 闭环传递函数
num = ky/c * [a b];                % 分子
den = [1, (d+ky*a)/c, ky*b/c];     % 分母
sys = tf(num, den);

disp('闭环传递函数：');
sys

%% ================= 零点与极点 =================
z = zero(sys);
p = pole(sys);

fprintf('\n系统零点：\n');
disp(z.');

fprintf('系统极点：\n');
disp(p.');

%% ================= 无阻尼自然角频率 & 阻尼比 =================
% 对二阶系统，每个极点对都有对应的 wn, zeta
[wn, zeta, p_check] = damp(sys);

fprintf('\n极点对应的无阻尼自然角频率 wn (rad/s)：\n');
disp(wn.');

fprintf('极点对应的阻尼比 zeta：\n');
disp(zeta.');

%% ================= 根轨迹 =================
% 根轨迹应基于开环系统
G_ol = tf([a b], [c d 0]);   % G(s) = (a s + b)/(c s^2 + d s)

figure;
rlocus(G_ol);
grid on;
title('Root Locus with respect to k_y');
xlabel('Real Axis');
ylabel('Imaginary Axis');
 %%
 
 %%
 
 
 
