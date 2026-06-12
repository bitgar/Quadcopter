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

%% dphi_dphic
num = [z1 z2];
den = [1 p1 p2 p3];

G_dyddot_dyddotc = tf(num,den)

%% dyd_dydc
num = [ky*z1 ky*z2];
den = [1 p1 p2 (p3+ky*z1) ky*z2];

G_dyd_dydc = tf(num,den)

disp('---------------------------------------------')

%% 阶跃响应
figure;
step(G_dyd_dydc);
grid on;
title('G_{dyd\_dydc} 阶跃响应');

%%

