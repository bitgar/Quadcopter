close all
clear all
clc

%%
% 参数
k1 = 2;
k2 = 5;

% 时间轴
t = linspace(0,10,1000);

% 解析解 (欠阻尼情况)
omega = 0.5*sqrt(4*k2 - k1^2);
C1 = 1;          % e(0) = 1
C2 = (0 + k1/2*C1)/omega; % 由 e_dot(0)=0 求 C2

e = exp(-k1/2*t).*( C1*cos(omega*t) + C2*sin(omega*t) );

% 绘图
figure;
plot(t, e,'b','LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('e(t)');
title('Solution of e'''' + k1 e'' + k2 e = 0');
%%

%%