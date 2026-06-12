close all
clear all
clc

%% 主导极点阻尼比计算脚本
%% =================== 已知参数 ===================
a = 3.1588;
b = 0.4;
c = 3.3;
d = 0.4;

zeta_target = 0.505;

%% =================== KH 扫描设置 ===================
KH_min = 0.01;
KH_max = 200;
N_scan = 5000;

KH_vec = linspace(KH_min, KH_max, N_scan);
zeta_vec = NaN(size(KH_vec));

%% =================== 扫描主导极点阻尼比 ===================
for i = 1:length(KH_vec)
    zeta_vec(i) = dominant_zeta(KH_vec(i), a, b, c, d);
end

%% =================== 绘制阻尼比曲线 ===================
figure;
plot(KH_vec, zeta_vec, 'LineWidth', 1.5); grid on;
yline(zeta_target, 'r--', 'LineWidth', 1.2);
xlabel('K_H');
ylabel('Dominant pole damping ratio \zeta');
title('Dominant damping ratio vs K_H');

%% =================== 构造误差并筛选合法区间 ===================
zeta_err = zeta_vec - zeta_target;

valid_idx = find(~isnan(zeta_err));
KH_valid = KH_vec(valid_idx);
err_valid = zeta_err(valid_idx);

%% =================== 自动寻找 fzero 区间 ===================
sign_change_idx = find(err_valid(1:end-1).*err_valid(2:end) < 0);

if isempty(sign_change_idx)
    error('在扫描范围内未找到满足 ζ = %.3f 的 K_H', zeta_target);
end

KH_left  = KH_valid(sign_change_idx(1));
KH_right = KH_valid(sign_change_idx(1) + 1);

fprintf('找到合法求解区间： [%.6f, %.6f]\n', KH_left, KH_right);

%% =================== 精确求解 K_H ===================
zeta_err_fun = @(KH) dominant_zeta(KH, a, b, c, d) - zeta_target;

KH_sol = fzero(zeta_err_fun, [KH_left KH_right]);

fprintf('满足主导极点阻尼比 ζ = %.3f 的 K_H ≈ %.6f\n', ...
        zeta_target, KH_sol);

%% =================== 结果验证 ===================
poly = [1, c, (KH_sol*a + d), KH_sol*b];
poles = roots(poly);

disp('对应闭环极点：');
disp(poles.');

cp = poles(imag(poles) ~= 0);
zeta_check = -real(cp(1)) / abs(cp(1));

fprintf('验证得到的阻尼比 ζ ≈ %.6f\n', zeta_check);

%% ==========  用最优 K_H 打印传递函数并画零极点图  ==========
% 1. 最优增益
KH_opt = KH_sol;

% 2. 构造传递函数分子/分母多项式（s 降幂排列）
num = [KH_opt*a, KH_opt*b];               % K_H·a·s + K_H·b
den = [1, c, (KH_opt*a + d), KH_opt*b];   % s? + c·s? + (K_H·a+d)s + K_H·b

% 3. 创建 TF 对象
sys_cl = tf(num, den);

% 4. 打印传递函数
fprintf('\n最优闭环传递函数（K_H = %.6f）：\n', KH_opt);
disp(sys_cl);

% 5. 零极点图
figure;
pzmap(sys_cl);  grid on;
title(sprintf('Pole-Zero Map  (K_H = %.6f, \\zeta_{dom} = %.3f)', KH_opt, zeta_check));
set(gcf, 'Color', 'w');

% 6. 高亮主导极点（最靠右的复极点）
poles_opt = pole(sys_cl);
cp_opt    = poles_opt(imag(poles_opt)~=0);
[~,idx]   = max(real(cp_opt));   % 最靠右 = 主导
dom_pole  = cp_opt(idx);

hold on;
plot(real(dom_pole), imag(dom_pole), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(real(conj(dom_pole)), imag(conj(dom_pole)), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(real(dom_pole)+0.02*abs(real(dom_pole)), ...
     imag(dom_pole)+0.02*abs(imag(dom_pole)), ...
     sprintf('Dominant\\n%.3f±%.3fi', real(dom_pole), imag(dom_pole)), ...
     'Color', 'r', 'FontSize', 9, 'HorizontalAlignment', 'left');

% 7. 零点标注
zeros_opt = zero(sys_cl);
if ~isempty(zeros_opt)
    text(real(zeros_opt), imag(zeros_opt), '  Zero', 'Color', 'b', 'FontSize', 9);
end

%% =================== 局部函数 ===================
function zeta = dominant_zeta(KH, a, b, c, d)

    poly = [1, c, (KH*a + d), KH*b];
    poles = roots(poly);

    % 仅考虑复极点
    cp = poles(imag(poles) ~= 0);

    if isempty(cp)
        zeta = NaN;
        return;
    end

    % 选取实部最大的复极点作为主导极点
    [~, idx] = max(real(cp));
    p = cp(idx);

    zeta = -real(p) / abs(p);
end
%%
