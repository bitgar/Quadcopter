close all
clear all
clc

%% 
% 定义传递函数 G(s) = (kp*s + ki) / (J*s^2 + kp*s + ki)
J = 1.239e-2;
kp = 0.6823;
ki = 37.5782;
num = [kp ki];              % 分子系数
den = [J kp ki];        % 分母系数
sys = tf(num, den);     % 生成传递函数对象


[y,t] = step(sys,5);

% 计算并绘制单位阶跃响应
figure;
hold on
plot(t,y,'ro','linewidth',2)
% step(sys)
grid on;
title('单位阶跃响应');
xlabel('时间 (秒)');
ylabel('输出');
hold off

%%

%%

