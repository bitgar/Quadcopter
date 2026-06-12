close all
clear all
clc

%%
% 定义分子与分母
num = [1.227e06 2.652e08 2.731e05];
den = [1 431.7 1.421e05 1.179e07 2.652e08 2.731e05];

% 创建传递函数对象
sys = tf(num, den);

% 输出系统信息
disp('传递函数为:');
sys

% 绘制零极点图
figure;
pzmap(sys);
grid on;
title('传递函数的零极点分布图');

% 计算零点与极点数值
[z, p, k] = tf2zp(num, den);
disp('零点:');
disp(z);
disp('极点:');
disp(p);
disp('增益:');
disp(k);

%%
% close all
clear all
clc
%%
% 原始传递函数
num = [1.227e6 2.652e8 2.731e5];
den = [1 431.7 1.421e5 1.179e7 2.652e8 2.731e5];
sys = tf(num, den);

% 尝试 minreal 降阶（消去近似极零对）
tol = 1e-6;
sys_min = minreal(sys, tol);

% 1) 单独阶跃响应与 stepinfo
figure; step(sys); grid on; title('Step Response - Original');
si_orig = stepinfo(sys);
disp('Step info (original):'); disp(si_orig);
fv_orig = dcgain(sys); fprintf('Original DC gain (dcgain) = %.6f\n', fv_orig);

figure; step(sys_min); grid on; title('Step Response - Reduced (minreal)');
si_red = stepinfo(sys_min);
disp('Step info (reduced):'); disp(si_red);
fv_red = dcgain(sys_min); fprintf('Reduced DC gain (dcgain) = %.6f\n', fv_red);

% 2) 在同一图用手工绘制来比较（避免 step 的语法限制）
[t1,y1] = step(sys);
[t2,y2] = step(sys_min);
figure; 
plot(t1,y1,'b','LineWidth',1.4); hold on;
plot(t2,y2,'r--','LineWidth',1.4);
legend('Original','Reduced');
xlabel('Time (s)'); ylabel('Amplitude');
title('Step Response Comparison');
grid on;

% 3) 打印稳态误差（相对于理想 1 的百分比）
fprintf('Steady-state error original = %.4f%%\n', (1 - fv_orig)*100);
fprintf('Steady-state error reduced  = %.4f%%\n', (1 - fv_red)*100);

% 4) 若需绘制 bode 并标 -3dB 点（示例）
w = logspace(-1,3,2000); % rad/s grid (0.1 ~ 1000 rad/s)
[mag,~] = bode(sys, w); mag = squeeze(mag);
mag0 = mag(1);
target = mag0 / sqrt(2);
idx = find(mag <= target,1,'first');
if ~isempty(idx)
    bw_rad = w(idx); bw_Hz = bw_rad/(2*pi);
    fprintf('Estimated -3dB bandwidth ≈ %.4f Hz (%.4f rad/s)\n', bw_Hz, bw_rad);
else
    fprintf('No -3dB crossing found in w grid.\n');
end
figure; semilogx(w/(2*pi), 20*log10(mag)); hold on; grid on;
if ~isempty(idx)
    semilogx(bw_Hz, 20*log10(mag(idx)), 'ro', 'MarkerSize',8,'LineWidth',1.2);
    text(bw_Hz, 20*log10(mag(idx)), sprintf('  %.4g Hz', bw_Hz));
end
xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)'); title('Bode Magnitude');
%%
