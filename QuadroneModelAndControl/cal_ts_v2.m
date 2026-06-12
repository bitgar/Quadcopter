% compute_closed_loop.m
% 串联 G1 * G2 * G3，单位负反馈，求闭环 T = L/(1+L)

clear; clc; close all;

% 声明 s
s = tf('s');

% 定义各段传递函数（按题目数值）
G1 = 5.5832 + 25.0811*s   % 注意：这是非真分式单元，但与其他两段串联后整体为真分式
G2 = (4.892e4 * s + 1.056e7) / (s^3 + 431.7*s^2 + 1.421e5*s + 1.056e7)
G3 = 1 / s^2

% 开环
L = G1 * G2 * G3;

% 可选：化简开环（去公因子）
L = minreal(L);

% 闭环（单位负反馈）
T = feedback(L, 1);   % 默认 feedback(sys,1) 表示 sys/(1+sys)

% 化简闭环
T = minreal(T);

% 显示结果
disp('--- Open-loop L(s) ---');
L
disp('--- Closed-loop T(s) ---');
T

% 导出分子与分母多项式（如果需要）
[num_T, den_T] = tfdata(T, 'v');
fprintf('Closed-loop numerator coefficients (highest->lowest):\n'); disp(num_T);
fprintf('Closed-loop denominator coefficients (highest->lowest):\n'); disp(den_T);

%%
% 极点 / 零点
% z = zero(T);
% p = pole(T);
% k = dcgain(T);
% fprintf('Closed-loop zeros:\n'); disp(z);
% fprintf('Closed-loop poles:\n'); disp(p);
% fprintf('Closed-loop DC gain: %g\n', k);
% 
% % 绘图：阶跃响应与波特图
% figure;
% step(T);
% title('Closed-loop step response');
% 
% figure;
% bode(L); grid on;
% title('Open-loop Bode (L)');
% 
% figure;
% bode(T); grid on;
% title('Closed-loop Bode (T)');
% 
% % 如需绘制闭环根轨迹（用开环 L）
% figure;
% rlocus(L);
% title('Root locus of L');
% 
% % 保存到 .mat（可选）
% save('closed_loop_results.mat', 'G1', 'G2', 'G3', 'L', 'T');

% End of script