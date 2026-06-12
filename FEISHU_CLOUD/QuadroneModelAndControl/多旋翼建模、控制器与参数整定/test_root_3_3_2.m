close all
clear all
clc

%% 已知参数
J   = 1.239e-2;
% 角速度控制器
kp  = 5.3487;   % 比例增益
ki  = 1154.8532;% 积分增益
% 姿态角控制器
kpa = 113.3148; % 比例增益
% 位置控制器
km = 5.5832; % 比例增益
kn = 25.0811;% 微分增益
targetFc = 5;            % Hz
targetWc = targetFc*2*pi;% rad/s

%% 闭环传递函数（km、kn 为待定参数）
sysCL = @(km,kn) tf( ...
        [kpa*kn*kp, kpa*(km*kp+kn*ki), kpa*km*kp], ...
        [J, kp, (kpa*kp+ki), (kpa*ki+kpa*kn*kp), kpa*(km*kp+kn*ki), kpa*km*kp]);

%% 辅助：求当前截止频率（rad/s）
wcFun = @(sys) bandwidth(sys);   % MATLAB 内置，单位 rad/s

%% 目标函数：与目标截止频率的误差平方
cost = @(x) (wcFun(sysCL(x(1),x(2))) - targetWc)^2;

%% 初值 & 无约束优化
x0 = [1; 1];
opts = optimset('Display','iter','TolX',1e-6);
xOpt = fminsearch(cost,x0,opts);
kmOpt = xOpt(1);
knOpt = xOpt(2);

fprintf('整定结果：km = %.6f ，kn = %.6f\n',kmOpt,knOpt);

%% 构造最终闭环模型
% sysFinal = sysCL(kmOpt,knOpt);
km = 2.5; % 比例增益
kn = 2;% 微分增益
sysFinal = sysCL(km,kn);

fcActual = wcFun(sysFinal)/(2*pi);
fprintf('实际截止频率：%.4f Hz\n',fcActual);

%% Bode 图（横轴 Hz）—— 修正版本
h  = bodeplot(sysFinal);          % 获取图柄
grid on
setoptions(h,'FreqUnits','Hz');   % 正确用法：作用于图柄 h
title(sprintf('Bode 图（km=%.4f, kn=%.4f）',kmOpt,knOpt));

%% 在 –3 dB 处画参考线
hold on;
yline(-3,'r--',sprintf('-3 dB @ %.2f Hz',fcActual));
hold off;

%%
[y,t] = step(sysFinal);

figure;
hold on
plot(t,y,'b-','linewidth',2)
% step(sys)
grid on;
title('单位阶跃响应');
xlabel('时间 (秒)');
ylabel('输出');
hold off

%%


