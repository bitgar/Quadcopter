close all
clear all
clc

%% 参数
z1 = 51.23;
z2 = 57.36;

p1 = 4.59;
p2 = 10.38;
p3 = 7.041;

ky = 0.014;

%% dyddot_dyddotc
num = [z1 z2];
den = [1 p1 p2 p3];

G_dyddot_dyddotc = tf(num,den)

[num2, den2] = tfdata(G_dyddot_dyddotc, 'v')
z1_2 = num2(3);
z2_2 = num2(4);
p1_2 = den2(2);
p2_2 = den2(3);
p3_2 = den2(4);

%% dyd_dydc
num = [ky*z1_2 ky*z2_2]
den = [1 p1_2 p2_2 (p3_2+ky*z1_2) ky*z2_2]
G_dyd_dydc = tf(num,den)

% num = [ky*z1 ky*z2]
% den = [1 p1 p2 (p3+ky*z1) ky*z2]
% G_dyd_dydc2 = tf(num,den)

disp('---------------------------------------------')

%% 阶跃响应
figure;
step(G_dyd_dydc);
grid on;
title('G_{dyd\_dydc} 阶跃响应');

%%

