% filepath: e:\FEISHU\Quadrotor_v3_towards_v3\ident_script_fixed.m
%% 基于 out.in1 和 out.out1 (Array格式) 的系统辨识脚本 - 修复版
clc; close all;

% ==========================================
% 1. 参数设置 (请与 Simulink 设置保持一致)
% ==========================================
% ?? 重要：必须手动指定采样时间！因为 Array 格式不带时间戳
Ts = 0.0005; 

fprintf('设定采样时间: %.6f s (频率 %.0f Hz)\n', Ts, 1/Ts);

% ==========================================
% 2. 数据提取 (增强鲁棒性)
% ==========================================
if ~exist('out', 'var')
    error('错误：工作区中找不到 "out" 变量。请先运行 Simulink 仿真。');
end

try
    % 1. 提取原始数据
    u_raw_obj = out.in1; 
    y_raw_obj = out.out1;
    
    % 2. 智能判断数据类型并提取数值
    % 如果是 timeseries 或 struct (To Workspace有时会自动变)，提取 .Data
    if isa(u_raw_obj, 'timeseries')
        u_raw = u_raw_obj.Data;
    elseif isstruct(u_raw_obj) && isfield(u_raw_obj, 'signals')
        u_raw = u_raw_obj.signals.values;
    else
        u_raw = u_raw_obj; % 假设已经是数组
    end
    
    if isa(y_raw_obj, 'timeseries')
        y_raw = y_raw_obj.Data;
    elseif isstruct(y_raw_obj) && isfield(y_raw_obj, 'signals')
        y_raw = y_raw_obj.signals.values;
    else
        y_raw = y_raw_obj; % 假设已经是数组
    end
    
    % 3. 强制转换为 double (解决报错的核心！)
    u_raw = double(u_raw);
    y_raw = double(y_raw);
    
    % 4. 挤压维度 (去除多余的维度，例如 1x1xN 变成 N)
    u_raw = squeeze(u_raw);
    y_raw = squeeze(y_raw);

    % 5. 确保是列向量 (N x 1)
    if size(u_raw, 2) > size(u_raw, 1)
        u_raw = u_raw'; 
    end
    if size(y_raw, 2) > size(y_raw, 1)
        y_raw = y_raw'; 
    end
    
    % 6. 长度对齐
    len = min(length(u_raw), length(y_raw));
    u_raw = u_raw(1:len);
    y_raw = y_raw(1:len);
    
    % 重建时间轴
    t = (0:len-1)' * Ts;
    
    fprintf('成功提取数据: %d 个采样点，总时长 %.2f 秒\n', len, t(end));
    
catch ME
    error('数据提取失败。请确认 out.in1 和 out.out1 是否存在。\n错误信息: %s', ME.message);
end

% ==========================================
% 3. 数据预处理
% ==========================================
% 截取数据：去掉前 0.5 秒 (防止初始震荡影响)
start_time = 0.5; 
start_idx = max(1, round(start_time/Ts));

if start_idx >= len
    error('截取起始时间超过了数据总长度！请检查 start_time 设置。');
end

u_segment = u_raw(start_idx:end);
y_segment = y_raw(start_idx:end);

% **关键步骤**：去趋势 (Remove Mean / Detrend)
u_det = detrend(u_segment, 'constant');
y_det = detrend(y_segment, 'constant');

% 创建系统辨识数据对象 (iddata)
% 这里应该不会再报错了，因为 u_det 和 y_det 已经是纯 double 列向量
data = iddata(y_det, u_det, Ts);

% 绘制预处理后的数据
figure('Name', '辨识数据预览', 'Color', 'w');
subplot(2,1,1); plot(t(start_idx:end), u_det); 
title('输入信号 (Detrended)'); grid on; ylabel('Input');
subplot(2,1,2); plot(t(start_idx:end), y_det); 
title('输出信号 (Detrended)'); grid on; ylabel('Output');

% ==========================================
% 4. 执行模型辨识
% ==========================================
fprintf('\n-----------------------------------\n');
fprintf('开始辨识...\n');

% 目标结构：3个极点 (np=3), 1个零点 (nz=1)
np = 3; 
nz = 1;

% 配置项
opt = tfestOptions('Display', 'on', ...
                   'EnforceStability', true, ...     % 强制稳定
                   'SearchMethod', 'auto');         % 自动搜索

% 执行辨识
sys_id = tfest(data, np, nz, opt);

% ==========================================
% 5. 结果验证与对比
% ==========================================
% 真实目标模型 (Ground Truth)
num_true = [55280, 2170000];
den_true = [1, 100.5, 107900, 4081000];
G_true = tf(num_true, den_true);

% 终端打印对比
disp(' ');
disp('>>> 真实模型 G_true:');
G_true
disp('>>> 辨识结果 sys_id:');
sys_id

% 计算匹配度 (防止 compare 返回 cell 导致 fprintf 报错)
[~, fit, ~] = compare(data, sys_id);
if iscell(fit)
    fit_percent = fit{1};
else
    fit_percent = fit;
end
fprintf('\n>>> 训练数据拟合度 (Fit): %.2f%%\n', fit_percent);

% 核心绘图：Bode图对比
figure('Name', '辨识结果验证 - Bode图', 'Color', 'w');
h = bodeplot(G_true, sys_id);
setoptions(h, 'FreqUnits', 'Hz', 'PhaseMatching', 'on', 'Grid', 'on');
legend('真实模型 (Ground Truth)', '辨识模型 (Identified)');
title(sprintf('Bode 图对比 (拟合度: %.2f%%)', fit_percent));

if fit_percent > 85
    msgbox('恭喜！辨识非常成功！', 'Success');
else
    warning('拟合度较低，请检查：1.Simulink步长是否为0.0005？ 2.Chirp频率是否设置正确？');
end