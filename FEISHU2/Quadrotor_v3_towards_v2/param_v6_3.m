close all
clear all
clc

%% 
% dphi/dphi_c 手动降阶后的系统 G(s) = (0.629s + 0.7108) / (1.13s + 0.7108)
num = [0.629, 0.7108];
den = [1.13, 0.7108];
G = tf(num, den)

% dphi/dphi_c 原始三阶系统
num2 = [3.384, 3.822];
den2 = [1, 4.585, 8.562,3.822];
G2 = tf(num2, den2)

% dchi/dchi_c 原始传递函数
num3 = [0.00019, 0.35145];
den3 = [1, 0.14107];
G3 = tf(num3, den3)

%% G3 性质分析 (带宽、极零点、DC增益)
disp('--------------------------------------------------');
disp('Analyzing G3 properties...');

% 1. DC Gain
dc_gain_G3 = dcgain(G3);
fprintf('G3 DC Gain: %.4f\n', dc_gain_G3);

% 2. Bandwidth
bw_G3 = bandwidth(G3);
fprintf('G3 Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G3, bw_G3/(2*pi));

% 3. Poles and Zeros
p_G3 = pole(G3);
z_G3 = zero(G3);
disp('G3 Poles:');
disp(p_G3);
disp('G3 Zeros:');
disp(z_G3);

%% 串联分析 Verification
disp('--------------------------------------------------');
disp('Analyzing Series Connection G_series = G2 * G3 ...');

G_series = series(G2, G3);

% Calculate Bandwidth
bw_series = bandwidth(G_series);
fprintf('Series System Bandwidth: %.4f rad/s\n', bw_series);
fprintf('G3 (Slowest) Bandwidth : %.4f rad/s\n', bw_G3);

% Check matches
if abs(bw_series - bw_G3) < 0.05 * bw_G3
    disp('Result: The series bandwidth is dominanted by G3 (Match).');
else
    disp('Result: The series bandwidth shifted slightly (due to coupling effects).');
end

% Plot Bode for Series
figure('Name', 'Series Connection Bode');
bode(G2, 'b');
hold on;
bode(G3, 'g');
bode(G_series, 'r--');
grid on;
legend('G2 (Fast)', 'G3 (Slow)', 'G_series (Total)');
title('Series Connection Bandwidth Dominance');

%% 闭环控制提升带宽演示 (Closed-Loop Control)
disp('--------------------------------------------------');
disp('Demonstrating Bandwidth Improvement with Feedback Control...');

% 原始开环系统 (Open-loop plant)
Plant = G_series;
bw_open = bandwidth(Plant);
fprintf('Open-Loop Bandwidth: %.4f rad/s\n', bw_open);

% 设计一个简单的控制器 C(s) = Kp (比例控制)
% 目标：通过反馈将极点向左移动，从而提高响应速度（带宽）
Kp = 2; % 尝试提?增益
C = tf(Kp, 1);

% 闭环系统 T(s) = L / (1 + L), where L = P*C
L = series(C, Plant);
T_closed = feedback(L, 1);

% 分析闭环带宽
bw_closed = bandwidth(T_closed);
fprintf('Closed-Loop Bandwidth with Kp=%.1f: %.4f rad/s\n', Kp, bw_closed);
fprintf(' Improvement Factor: %.2fx\n', bw_closed / bw_open);

% 进一步提高增益看看极限 (注意：增益过?会导致G2的高频极点引起不稳?)
Kp_high = 10;
C_high = tf(Kp_high, 1);
T_closed_high = feedback(series(C_high, Plant), 1);
bw_closed_high = bandwidth(T_closed_high);

figure('Name', 'Closed-Loop Bandwidth Improvement');
bode(Plant, 'r');
hold on;
bode(T_closed, 'b');
bode(T_closed_high, 'k--');
grid on;
legend('Open-Loop Plant (Slow)', ...
       ['Closed-Loop (Kp=', num2str(Kp), ')'], ...
       ['Closed-Loop (Kp=', num2str(Kp_high), ')']);
title('Bandwidth Extension via Feedback Control');

figure('Name', 'Closed-Loop Step Response');
step(Plant, 'r');
hold on;
step(T_closed, 'b');
step(T_closed_high, 'k--');
grid on;
legend('Open-Loop', ['Closed-Loop (Kp=', num2str(Kp), ')'], ['Closed-Loop (Kp=', num2str(Kp_high), ')']);
title('Time Domain Speed Improvement');




%% G2 性质分析 (带宽、极零点、DC增益)
disp('--------------------------------------------------');
disp('Analyzing G2 properties...');

% 1. DC Gain
dc_gain_G2 = dcgain(G2);
fprintf('G2 DC Gain: %.4f\n', dc_gain_G2);

% 2. Bandwidth
bw_G2 = bandwidth(G2);
fprintf('G2 Bandwidth: %.4f rad/s (%.4f Hz)\n', bw_G2, bw_G2/(2*pi));

% 3. Poles and Zeros
p_G2 = pole(G2);
z_G2 = zero(G2);
disp('G2 Poles:');
disp(p_G2);
disp('G2 Zeros:');
disp(z_G2);

% Determine dominance
disp('Analyzing dominant poles...');
[wn, damping] = damp(G2);
disp('Damping and Natural Frequency of G2 poles:');
% Table with Pole, Damping, Frequency
for i = 1:length(p_G2)
    fprintf('Pole: %.4f + %.4fi, Damping: %.4f, Freq: %.4f rad/s\n', ...
        real(p_G2(i)), imag(p_G2(i)), damping(i), wn(i));
end

%% 模型降阶分析
disp('--------------------------------------------------');
disp('Performing Model Reduction on G2...');

% 尝试使用 balred (Balanced Truncation) 降阶�? 1 �? (与手�? G 对比)
% 注意：balred �?要系统稳定�?�G2 看起来是稳定的�??
try
    G2_red1 = balred(G2, 1);
    disp('Reduced Order Model (Order 1) via balred:');
    G2_red1
catch ME
    disp('balred failed (possibly due to toolbox or unstable system). Using approximation.');
    warning(ME.message);
    % Fallback: Dominant pole approximation if balred fails
    % Assuming real dominant pole is closest to imaginary axis
    [~, idx] = min(abs(real(p_G2)));
    dom_pole = real(p_G2(idx));
    % Match DC gain
    G2_red1 = tf(dc_gain_G2 * abs(dom_pole), [1, abs(dom_pole)]);
end

% Manual G analysis
disp('--------------------------------------------------');
disp('Analyzing Manual G properties...');
bw_G = bandwidth(G);
fprintf('G (Manual) Bandwidth: %.4f rad/s\n', bw_G);

%% Plot Comparison
figure('Name', 'Bode Plot Comparison');
bode(G2, 'b');
hold on;
bode(G, 'r--');
if exist('G2_red1', 'var')
    bode(G2_red1, 'g-.');
    legend('G2 (Original 3rd)', 'G (Manual 1st)', 'G2 Reduced (Auto 1st)');
else
    legend('G2 (Original 3rd)', 'G (Manual 1st)');
end
grid on;
title('Bode Diagram Comparison');

figure('Name', 'Step Response Comparison');
step(G2, 'b');
hold on;
step(G, 'r--');
if exist('G2_red1', 'var')
    step(G2_red1, 'g-.');
    legend('G2 (Original 3rd)', 'G (Manual 1st)', 'G2 Reduced (Auto 1st)');
else
    legend('G2 (Original 3rd)', 'G (Manual 1st)');
end
grid on;
title('Step Response Comparison');

%% 详细对比分析
disp('--------------------------------------------------');
disp('Comparison Summary:');
fprintf('G2 Bandwidth: %.4f rad/s\n', bw_G2);
fprintf('G  Bandwidth: %.4f rad/s\n', bw_G);
if exist('G2_red1', 'var')
    bw_red = bandwidth(G2_red1);
    fprintf('G_red Bandwidth: %.4f rad/s\n', bw_red);
end

fprintf('\nDC Gain Comparison:\n');
fprintf('G2 DC Gain: %.4f\n', dcgain(G2));
fprintf('G  DC Gain: %.4f\n', dcgain(G));


figure('Name', 'Step Response');
hold on
step(G);
step(G2);
grid on;
hold off
title('Step Response');
xlabel('Time (s)');
ylabel('Amplitude');


%%
