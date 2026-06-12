close all
clear all
clc

%%
%% 参数设置
J   = 1.239e-2;   % 转动惯量
% 角速度控制器参数 PI控制器
kp = 5.3487;      % 比例
ki = 1154.8532;   % 积分
% 角度控制器 P控制器
kpa = 0.8;        % 举例，可自行修改
% 位置控制器 PD控制器
% km = ; % P参数
% kn = ; % D参数

wc_des = 5 *2 *pi;

%% 建立开环与闭环传递函数
optfun = @(km kn) 20*log10(abs(freqresp(tf([kpa*kp kpa*ki],[J kp (kpa*kp+ki) kpa*ki]), wc_des))) + 3;
kpa0 = 0.01;      % 初值
kpa  = fzero(optfun, kpa0);  % 求解 kpa

fprintf('调节结果: kpa = %.4f\n', kpa);

%% 验证与绘图
%================= 验证与绘图 =================
T = tf([kpa*kp kpa*ki],[J kp (kpa*kp+ki) kpa*ki]);
figure;
bode(T,'b-');            % 画Bode图
grid on;
title('闭环带宽25Hz')

%%
num = [kpa*kp kpa*ki];                  % 分子系数
den = [J kp (kpa*kp+ki) kpa*ki];        % 分母系数
sys = tf(num, den)                      % 生成传递函数对象

% [y,t] = step(sys,5);
[y,t] = step(sys);

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


