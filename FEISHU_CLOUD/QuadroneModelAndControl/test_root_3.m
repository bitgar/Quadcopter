close all
clear all
clc

%================= 参数设定 =================
J      = 1.239e-2;            % 惯量 (示例)
wc_des = 100*2*pi;              % 目标截止频率 [rad/s]
zeta   = 0.5;             % 期望阻尼比

%================= 优化搜索 =================
optfun = @(ki) 20*log10(abs(freqresp(tf([2*sqrt(J*ki)*zeta ki],[J 2*sqrt(J*ki)*zeta ki]), wc_des))) + 3;

ki0 = 0.01;      % 初值
ki  = fzero(optfun, ki0);  % 求解 ki
kp = 2*sqrt(J*ki)*zeta;
fprintf('调节结果: kp = %.4f , ki = %.4f\n', kp, ki);

%================= 验证与绘图 =================
T = tf([kp ki],[J kp ki]);
figure;
margin(T);            % 画Bode图并显示增益交叉、相位裕度等
grid on;

%%
num = [kp ki];              % 分子系数
den = [J kp ki];        % 分母系数
sys = tf(num, den)     % 生成传递函数对象

% [y,t] = step(sys,5);
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

%% 新测试，标准二阶振荡环节和带零点的二阶振荡环节的组合
% clear; close all; clc;
% 
% % 参数（你可以改 wn）
% wn = 5;                % 自然频率
% t = 0:0.001:2;         % 时间向量
% 
% % 标准二阶系统响应（不同阻尼比）
% zeta_vals = [0.5, 1.0, 1.5];   % 欠阻尼, 临界阻尼, 过阻尼
% figure; hold on;
% colors = {'r','b','g'};
% for k = 1:length(zeta_vals)
%     zeta = zeta_vals(k);
%     sys = tf(wn^2, [1 2*zeta*wn wn^2]);
%     y = step(sys, t);
%     plot(t, y, 'Color', colors{k}, 'LineWidth', 2);
% end
% grid on; xlabel('Time (s)'); ylabel('Response');
% legend('ζ=0.5 (underdamped)','ζ=1.0 (critical)','ζ=1.5 (overdamped)');
% title('Standard 2nd-order Step Response Comparison');
% 
% % 如果你想看带零点的情形（例如 (s+z)/(... )）
% z = 2;  % 零点位置 (可以改)
% num = [1 z];               % 分子 (s + z)
% den = [1 2*1*wn wn^2];     % 这里以 ζ=1 构造分母(标准化形式)
% sys_with_zero = tf(num, den);
% figure;
% [yz, tz] = step(sys_with_zero, t);
% plot(tz, yz, 'b-', 'LineWidth', 2); grid on;
% xlabel('Time (s)'); ylabel('Response');
% title(['Step response with zero at s = -' num2str(z) ' (ζ=1)']);

%%