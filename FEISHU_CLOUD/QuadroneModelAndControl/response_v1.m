close all
clear all
clc

%% 
% 定义传递函数 G(s) = 133 / (s^2 + 25*s + 10)
num = 133;              % 分子系数
den = [1 25 10];        % 分母系数
sys = tf(num, den);     % 生成传递函数对象

[y,t] = step(sys);

% 计算并绘制单位阶跃响应
figure;
hold on
plot(t,y,'ro','linewidth',2)
step(sys)
grid on;
title('单位阶跃响应');
xlabel('时间 (秒)');
ylabel('输出');
hold off

%%

%%

