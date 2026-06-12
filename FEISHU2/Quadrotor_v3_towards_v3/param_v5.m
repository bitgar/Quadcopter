close all
clear all
clc

%% ²ÎÊý
z1 = 1.7077;
z2 = 1.9121;
% z1 = 3.725*sind(27.29);
% z2 = 4.171*sind(27.29);

p1 = 4.59;
p2 = 8.878;
p3 = 4.173;

v0 = 30;
kyd = 0.05;

%% dphi_dphic
num = [z1 z2];
den = [1 p1 p2 p3];

G_dphi_dphic = tf(num,den)

%% dyddot_dyddotc
num = [v0*z1 v0*z2];
den = [1 p1 (p2+v0*kyd) p3+v0*kyd*z2];

G_dyddot_dyddotc = tf(num,den)

disp('---------------------------------------------')

%%

