%% 1. 定义原始系统（5 阶 / 2 阶）
num_orig = [1.227e6 2.652e8 2.731e5];
den_orig = [1 431.7 1.421e5 1.179e7 2.652e8 2.731e5];
sys_orig = tf(num_orig, den_orig);

%% 2. 定义识别出的系统（从图中抄的系数，注意符号/数量级）
% 这里按你截图写的 tf1: (1.267e06*s + 2.646e08) / (s^4 + 438 s^3 + 1.431e5 s^2 + 1.179e7 s + 2.647e8)
num_id = [1.267e6 2.646e8];
den_id = [1 438 1.431e5 1.179e7 2.647e8];
sys_id = tf(num_id, den_id);

%% 3. 输出极零与增益对比
[z_orig,p_orig,k_orig] = tf2zp(num_orig, den_orig);
[z_id,p_id,k_id] = tf2zp(num_id, den_id);

disp('原始系统零点：'); disp(z_orig);
disp('原始系统极点：'); disp(p_orig);
disp('识别系统零点：'); disp(z_id);
disp('识别系统极点：'); disp(p_id);

fprintf('原始系统 DC gain (dcgain) = %.6g\n', dcgain(sys_orig));
fprintf('识别系统 DC gain (dcgain) = %.6g\n', dcgain(sys_id));

%% 4. 画极零图对比
figure; 
subplot(1,2,1); pzmap(sys_orig); title('Original PZ map'); grid on;
subplot(1,2,2); pzmap(sys_id);    title('Identified PZ map'); grid on;

%% 5. Bode 对比并画出相对误差（频域比较）
w = logspace(-4, 3, 2000); % rad/s
[mag_orig, ph_orig] = bode(sys_orig, w); mag_orig = squeeze(mag_orig);
[mag_id, ph_id] = bode(sys_id, w);       mag_id   = squeeze(mag_id);

% 画幅频（Hz 为横轴）
figure;
semilogx(w/(2*pi), 20*log10(mag_orig), 'b', w/(2*pi), 20*log10(mag_id), 'r--');
legend('orig','id'); xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
grid on; title('Bode Magnitude Comparison');

% 相对幅值误差（线性）
rel_err = abs(mag_orig - mag_id) ./ max(mag_orig, eps);
figure; semilogx(w/(2*pi), rel_err); grid on;
xlabel('Frequency (Hz)'); ylabel('Relative error'); title('Relative magnitude error');

% 计算 L2 频域差异（可量化）
freq_norm = sqrt(trapz(w, (mag_orig - mag_id).^2));
fprintf('Frequency-domain L2 norm of mag difference = %.6e\n', freq_norm);

%% 6. 阶跃响应对比与 stepinfo
[t1,y1] = step(sys_orig);
[t2,y2] = step(sys_id);
figure;
plot(t1,y1,'b','LineWidth',1.2); hold on;
plot(t2,y2,'r--','LineWidth',1.2);
legend('orig','id'); xlabel('Time (s)'); ylabel('Step response'); grid on;
title('Step response comparison');

si_orig = stepinfo(sys_orig);
si_id = stepinfo(sys_id);
disp('stepinfo original:'); disp(si_orig);
disp('stepinfo id:');       disp(si_id);

%% 7. 检查原始系统是否确实含有靠近 0 的极零对（若存在则显示）
% 找出极点/零点模值接近 0 的项
small_p = p_orig(abs(p_orig) < 1e-2);
small_z = z_orig(abs(z_orig) < 1e-2);
disp('原始系统非常小的极点：'); disp(small_p);
disp('原始系统非常小的零点：'); disp(small_z);

%% 8. 备选：用 minreal 看原始系统是否可化简（消去近似抵消对）
sys_min = minreal(sys_orig, 1e-6);
disp('minreal 后系统：'); sys_min

%% 9. 输出 Fit 指标（如果你有用于辨识的 iddata，可用 compare）
% 假如你有 mydata（iddata 类型），可以：
% compare(mydata, sys_orig, sys_id);