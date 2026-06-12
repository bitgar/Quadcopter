close all
clear all
clc

%% 已知参数
% 位置控制器
km = 5.5832; % 比例增益
kn = 25.0811;% 微分增益

% 姿态角控制器
kpa = 113.3148; % 比例增益

% 角速度控制器
kp  = 5.3487;   % 比例增益
ki  = 1154.8532;% 积分增益

% 转动惯量
J   = 1.239e-2;


%% 
num = [0,     0,      0,              kpa*kn*kp,          kpa*(km*kp+kn*ki), kpa*km*ki]
den = [J,     kp,     (kpa*kp+ki),    (kpa*ki+kpa*kn*kp), kpa*(km*kp+kn*ki), kpa*km*ki]

G = tf(num,den)

den0 = den(1);                  % 分母首项系数
G_norm = tf(num/den0, den/den0)

%% Bode 图（横轴 Hz）—— 修正版本
% h  = bodeplot(sysFinal);          % 获取图柄
% grid on
% setoptions(h,'FreqUnits','Hz');   % 正确用法：作用于图柄 h
% title(sprintf('Bode 图（km=%.4f, kn=%.4f）',km,kn));
% % 在 –3 dB 处画参考线
% % hold on;
% % yline(-3,'r--',sprintf('-3 dB @ %.2f Hz',fcActual));
% % hold off;

%%
[y,t] = step(G_norm);

% figure;
% hold on
% plot(t,y,'b-','linewidth',2)
% % step(sys)
% grid on;
% title('单位阶跃响应');
% xlabel('时间 (秒)');
% ylabel('输出');
% hold off

%%
%%
% [num, den] = tfdata(G, 'v');  % 提取分子分母向量


