% 清除环境变量并关闭所有图形窗口
clear; clc; close all;

% 定义符号变量s
s = tf('s');

% 定义各部分的传递函数
% 被控对象的传递函数
G = (4.892e4*s + 1.056e7) / (s^3 + 431.7*s^2 + 1.421e5*s + 1.056e7);

% 控制器的传递函数，包括P和PD控制器
Kp_x7 = 5.5832;
Kp_x9 = 25.0811;
C = Kp_x7 + Kp_x9*s;

% 闭环传递函数
H = 1; % 反馈路径传递函数，假设为单位反馈
T = feedback(C*G, H);

% 化简闭环传递函数
T = minreal(T);

% 显示结果
disp('--- Open-loop L(s) ---');
L = C*G;
disp(L);

disp('--- Closed-loop T(s) ---');
disp(T);

% 导出分子与分母多项式（如果需要）
[num_T, den_T] = tfdata(T, 'v');
fprintf('Closed-loop numerator coefficients (highest->lowest):\n'); disp(num_T);
fprintf('Closed-loop denominator coefficients (highest->lowest):\n'); disp(den_T);

% 极点 / 零点
z = zero(T);
p = pole(T);
k = dcgain(T);
fprintf('Closed-loop zeros:\n'); disp(z);
fprintf('Closed-loop poles:\n'); disp(p);
fprintf('Closed-loop DC gain: %g\n', k);

% 绘图：阶跃响应与波特图
figure;
step(T);
title('Closed-loop step response');

figure;
bode(L); grid on;
title('Open-loop Bode (L)');

figure;
bode(T); grid on;
title('Closed-loop Bode (T)');

% 如需绘制闭环根轨迹（用开环 L）
figure;
rlocus(L);
title('Root locus of L');

% 保存到 .mat（可选）
save('closed_loop_results.mat', 'G', 'C', 'L', 'T');

% End of script