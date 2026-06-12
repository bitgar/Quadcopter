close all
clear all
clc

%% 设置参数
J      = 1.239e-2;            % 惯量 (示例)
wc_des = 100*2*pi;              % 目标截止频率 [rad/s]
zeta   = 0.707;             % 期望阻尼比
ki = 5.0;

%% 建立开环与闭环传递函数
L = tf([2*sqrt(J*ki)*zeta ki],[J 2*sqrt(J*ki)*zeta ki])

%% 0 Hz (DC) 增益
[mag0,~] = bode(L,0);   % mag0 是线性幅值
mag0 = squeeze(mag0)
target = mag0 / sqrt(2) % 下降 3 dB
figure
bode(L)

mag0_bode = 20*log10(mag0);
target_bode = 20*log10(target);

optfun = @(ki) 20*log10(abs(freqresp(tf([2*sqrt(J*ki)*zeta ki],[J 2*sqrt(J*ki)*zeta ki]), wc_des))) - target_bode;
ki0 = 0.01;      % 初值
ki  = fzero(optfun, ki0);  % 求解 ki
kp = 2*sqrt(J*ki)*zeta;

fprintf('DC增益 = %.3f (%.2f dB)\n', mag0, 20*log10(mag0));
% fprintf('-3 dB 截止频率 = %.2f rad/s\n', wc);
% fprintf('-3 dB 截止频率 = %.2f Hz\n', fc);
fprintf('调节结果: kp = %.4f , ki = %.4f\n', kp, ki);

%================= 验证与绘图 =================
T = tf([kp ki],[J kp ki]);
figure;
bode(T,'b-');            % 画Bode图
grid on;
title('闭环带宽100Hz')

%% PID控制器
% kp = 0;
% ki = 0;
% kd = 1;
% num = [kd kp ki];              % 分子系数
% den = [(J+kd) kp ki];        % 分母系数

%% PI控制器
kp = 1;
ki = 50;
num = [kp ki];              % 分子系数
den = [J kp ki];        % 分母系数

%% PD控制器
% kp = 1;
% kd = 10000;
% num = [kd kp];              % 分子系数
% den = [(J+kd) kp];        % 分母系数

%% P控制器
% kp = 10;
% num = [kp];              % 分子系数
% den = [J kp];        % 分母系数

%%
sys = tf(num, den)     % 生成传递函数对象

% [y,t] = step(sys,2);
[y,t] = step(sys);
%%
% 计算并绘制单位阶跃响应
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