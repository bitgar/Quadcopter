close all
clear all
clc

%% 偏航通道
% 计算满足阻尼比 zeta=0.8 的比例增益 kt，并绘制根轨迹
% 开环 L(s) = kt * G(s) * (1/s)，单位负反馈
% G(s) = (1.12*s + 0.49) / (s^2 + 1.12*s + 0.49)

% 连续时间传递函数
s = tf('s');
G  = (1.12*s + 0.49) / (s^2 + 1.12*s + 0.49);
G3 = 1/s;                % 积分环节
L0 = G * G3;             % 未含增益的开环

target_zeta = 0.79;

% 根轨迹与阻尼线
figure('Name','Root Locus with zeta=0.8'); 
rlocus(L0); grid on; hold on;
sgrid(target_zeta, []);    % 叠加阻尼比线
title('偏航通道')
% sgrid(0.2:0.2:1,[]) 
% sgrid(0.8, 0.5:0.1:1.0);

% 扫描增益范围（避开完全为0的增益）
kvec = logspace(-6, 3, 4000);    % 从 1e-6 到 1e3 扫描
best = struct('kt', NaN, 'zeta', Inf, 'poles', []);

best_err = Inf;
best_idx = NaN;

for i = 1:numel(kvec)
    K = kvec(i);
    % 闭环：T(s) = feedback(K*L0, 1)
    T = feedback(K * L0, 1);
    p = pole(T);           % 闭环极点
    % 计算各极点的阻尼比（仅考虑欠阻尼的共轭对）
    zetas = [];
    poles_cplx = [];
    for j = 1:numel(p)
        if abs(imag(p(j))) > 1e-9   % 复杂极点
            sigma = real(p(j));
            omega = abs(imag(p(j)));
            zeta  = -sigma / sqrt(sigma^2 + omega^2);
            zetas(end+1) = zeta; 
            poles_cplx(end+1) = p(j); 
        end
    end

    if ~isempty(zetas)
        % 选择与目标阻尼比最接近的复杂极点
        [err, kmin] = min(abs(zetas - target_zeta));
        if err < best_err
            best_err = err;
            best.kt = K;
            best.zeta = zetas(kmin);
            best.poles = p;
            best_idx = i;
        end
    end
end

% 若扫描未找到（极少发生），回退到 K=0 的开环极点（其复共轭对阻尼恰为0.8）
if isnan(best.kt)
    warning('未在扫描范围找到满足条件的增益，回退到 K=0（zeta=0.8 的开环复对）。');
    best.kt = 0;
    T0 = feedback(0 * L0, 1);
    best.poles = pole(T0);
    % 理论上开环复对为 -0.56 ± j0.42，阻尼=0.8
    best.zeta = 123;
end

% 可视化：在根轨迹上标出该增益下的闭环极点
[r, k_rl] = rlocus(L0, kvec);
if ~isnan(best_idx)
    poles_at_best = r(:, best_idx);
else
    poles_at_best = best.poles;
end
plot(real(poles_at_best), imag(poles_at_best), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
title(sprintf('Root Locus (best kt=%.6g, zeta=%.4f)', best.kt, best.zeta));

% 打印结果
fprintf('目标阻尼比 zeta = %.3f\n', target_zeta);
fprintf('推荐增益 kt = %.6g\n', best.kt);
fprintf('该增益下闭环极点:\n');
disp(best.poles);

% 可选：验证最“主导”复对的阻尼比
dom = poles_at_best(abs(imag(poles_at_best)) == max(abs(imag(poles_at_best))));
if ~isempty(dom)
    sigma = real(dom(1));
    omega = abs(imag(dom(1)));
    zeta_dom = -sigma / sqrt(sigma^2 + omega^2);
    fprintf('主导复对阻尼比≈ %.4f\n', zeta_dom);
end

%%
syms s2
p1 = -0.56 + 0.42i;
p2 = -0.56 - 0.42i;
p3 = -0.10;


Phi = expand((s2 - best.poles(1))*(s2 - best.poles(2))*(s2 - best.poles(3)));
Phi = simplify(Phi);
Phi = collect(Phi, s2)
Phi = vpa(Phi,4);

% result_poles = [, best.poles(2), best.poles(3)];
% coeff = poly(result_poles)

%% 滚转通道
% 连续时间传递函数
s = tf('s');
G  = (4.598*s + 5.051) / (s^2 + 4.582*s + 5.069);
G3 = 1/s;                % 积分环节
L0 = G * G3;             % 未含增益的开环

target_zeta = 0.79;

% 根轨迹与阻尼线
figure('Name','Root Locus with zeta=0.8'); 
rlocus(L0); grid on; hold on;
sgrid(target_zeta, []);    % 叠加阻尼比线
title('滚转通道')

% 扫描增益范围（避开完全为0的增益）
kvec = logspace(-6, 3, 4000);    % 从 1e-6 到 1e3 扫描
best = struct('kt', NaN, 'zeta', Inf, 'poles', []);

best_err = Inf;
best_idx = NaN;

for i = 1:numel(kvec)
    K = kvec(i);
    % 闭环：T(s) = feedback(K*L0, 1)
    T = feedback(K * L0, 1);
    p = pole(T);           % 闭环极点
    % 计算各极点的阻尼比（仅考虑欠阻尼的共轭对）
    zetas = [];
    poles_cplx = [];
    for j = 1:numel(p)
        if abs(imag(p(j))) > 1e-9   % 复杂极点
            sigma = real(p(j));
            omega = abs(imag(p(j)));
            zeta  = -sigma / sqrt(sigma^2 + omega^2);
            zetas(end+1) = zeta; 
            poles_cplx(end+1) = p(j); 
        end
    end

    if ~isempty(zetas)
        % 选择与目标阻尼比最接近的复杂极点
        [err, kmin] = min(abs(zetas - target_zeta));
        if err < best_err
            best_err = err;
            best.kt = K;
            best.zeta = zetas(kmin);
            best.poles = p;
            best_idx = i;
        end
    end
end

% 若扫描未找到（极少发生），回退到 K=0 的开环极点（其复共轭对阻尼恰为0.8）
if isnan(best.kt)
    warning('未在扫描范围找到满足条件的增益，回退到 K=0（zeta=0.8 的开环复对）。');
    best.kt = 0;
    T0 = feedback(0 * L0, 1);
    best.poles = pole(T0);
    % 理论上开环复对为 -0.56 ± j0.42，阻尼=0.8
    best.zeta = 123;
end

% 可视化：在根轨迹上标出该增益下的闭环极点
[r, k_rl] = rlocus(L0, kvec);
if ~isnan(best_idx)
    poles_at_best = r(:, best_idx);
else
    poles_at_best = best.poles;
end
plot(real(poles_at_best), imag(poles_at_best), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
title(sprintf('Root Locus (best kt=%.6g, zeta=%.4f)', best.kt, best.zeta));

% 打印结果
fprintf('目标阻尼比 zeta = %.3f\n', target_zeta);
fprintf('推荐增益 kt = %.6g\n', best.kt);
fprintf('该增益下闭环极点:\n');
disp(best.poles);

% 可选：验证最“主导”复对的阻尼比
dom = poles_at_best(abs(imag(poles_at_best)) == max(abs(imag(poles_at_best))));
if ~isempty(dom)
    sigma = real(dom(1));
    omega = abs(imag(dom(1)));
    zeta_dom = -sigma / sqrt(sigma^2 + omega^2);
    fprintf('主导复对阻尼比≈ %.4f\n', zeta_dom);
end

%% 俯仰通道
% 连续时间传递函数
s = tf('s');
G  = (4.813*s + 5.054) / (s^2 + 4.796*s + 5.073);
G3 = 1/s;                % 积分环节
L0 = G * G3;             % 未含增益的开环

target_zeta = 0.79;

% 根轨迹与阻尼线
figure('Name','Root Locus with zeta=0.8'); 
rlocus(L0); grid on; hold on;
sgrid(target_zeta, []);    % 叠加阻尼比线
title('俯仰通道')

% 扫描增益范围（避开完全为0的增益）
kvec = logspace(-6, 3, 4000);    % 从 1e-6 到 1e3 扫描
best = struct('kt', NaN, 'zeta', Inf, 'poles', []);

best_err = Inf;
best_idx = NaN;

for i = 1:numel(kvec)
    K = kvec(i);
    % 闭环：T(s) = feedback(K*L0, 1)
    T = feedback(K * L0, 1);
    p = pole(T);           % 闭环极点
    % 计算各极点的阻尼比（仅考虑欠阻尼的共轭对）
    zetas = [];
    poles_cplx = [];
    for j = 1:numel(p)
        if abs(imag(p(j))) > 1e-9   % 复杂极点
            sigma = real(p(j));
            omega = abs(imag(p(j)));
            zeta  = -sigma / sqrt(sigma^2 + omega^2);
            zetas(end+1) = zeta; 
            poles_cplx(end+1) = p(j); 
        end
    end

    if ~isempty(zetas)
        % 选择与目标阻尼比最接近的复杂极点
        [err, kmin] = min(abs(zetas - target_zeta));
        if err < best_err
            best_err = err;
            best.kt = K;
            best.zeta = zetas(kmin);
            best.poles = p;
            best_idx = i;
        end
    end
end

% 若扫描未找到（极少发生），回退到 K=0 的开环极点（其复共轭对阻尼恰为0.8）
if isnan(best.kt)
    warning('未在扫描范围找到满足条件的增益，回退到 K=0（zeta=0.8 的开环复对）。');
    best.kt = 0;
    T0 = feedback(0 * L0, 1);
    best.poles = pole(T0);
    % 理论上开环复对为 -0.56 ± j0.42，阻尼=0.8
    best.zeta = 123;
end

% 可视化：在根轨迹上标出该增益下的闭环极点
[r, k_rl] = rlocus(L0, kvec);
if ~isnan(best_idx)
    poles_at_best = r(:, best_idx);
else
    poles_at_best = best.poles;
end
plot(real(poles_at_best), imag(poles_at_best), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
title(sprintf('Root Locus (best kt=%.6g, zeta=%.4f)', best.kt, best.zeta));

% 打印结果
fprintf('目标阻尼比 zeta = %.3f\n', target_zeta);
fprintf('推荐增益 kt = %.6g\n', best.kt);
fprintf('该增益下闭环极点:\n');
disp(best.poles);

% 可选：验证最“主导”复对的阻尼比
dom = poles_at_best(abs(imag(poles_at_best)) == max(abs(imag(poles_at_best))));
if ~isempty(dom)
    sigma = real(dom(1));
    omega = abs(imag(dom(1)));
    zeta_dom = -sigma / sqrt(sigma^2 + omega^2);
    fprintf('主导复对阻尼比≈ %.4f\n', zeta_dom);
end

%%
