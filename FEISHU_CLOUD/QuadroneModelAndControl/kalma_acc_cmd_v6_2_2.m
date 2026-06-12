clear all
close all
clc

%% add the function path
addpath('..\..\MyFunc_public');
addpath('..\..\MyFunc_estimate');
%% 设置采样频率、地球重力加速度和高度传感器的偏置
fs = 400; 
fs_acc = fs;
fs_acc_use = 100;
% fs_acc_use = 200;
% fs_acc_use = 400;
fs_gps = 10;
fs_rtk = 10;
fs_baro = 100;

earth_gn = 9.950;

base_height_gps = 0;
base_height_rtk = 0;
base_height_baro = 0;
%% 导入数据
flag_data = 1; % 
% flag_data = 2; % 
% flag_data = 3; % 

if(flag_data == 1)
    load('E:\Data_Origin\高度观测器\20240620\424\4242.mat'); 
elseif(flag_data == 2)
    load('E:\Data_Origin\高度观测器\20240620\425\4252.mat'); 
elseif(flag_data == 3)
    load('E:\Data_Origin\高度观测器\20240620\426\4262.mat'); 
end 

%% 降采样Acc、RTK、GPS、Baro
% GPS 源
t_gps = t_gps(1:fs_acc/fs_gps:end);
vel_n_gps = vel_n_gps(1:fs_acc/fs_gps:end);
vel_e_gps = vel_e_gps(1:fs_acc/fs_gps:end);
vel_d_gps = vel_d_gps(1:fs_acc/fs_gps:end);
h_gps = h_gps(1:fs_acc/fs_gps:end);

% RTK 源
t_rtk = t_rtk(1:fs_acc/fs_rtk:end);
vel_d_rtk = vel_d_rtk(1:fs_acc/fs_rtk:end);
h_rtk = h_rtk(1:fs_acc/fs_rtk:end);

% 气压计 源
t_baro = t_baro(1:fs_acc/fs_baro:end);
h_baro = h_baro(1:fs_acc/fs_baro:end);

% 融合高度 源
t_acc_cmd_pos = t_acc_cmd_pos(1:fs_acc/fs_acc_use:end);
cmd_acc_target_pos = cmd_acc_target_pos(1:fs_acc/fs_acc_use:end);
cmd_vel_guided_pos = cmd_vel_guided_pos(1:fs_acc/fs_acc_use:end);

t_acc_fusion = t_acc_fusion(1:fs_acc/fs_acc_use:end);
acc_z_fusion = acc_z_fusion(1:fs_acc/fs_acc_use:end);

% 融合速度 源
t_acc_cmd = t_acc_cmd(1:fs_acc/fs_acc_use:end);
cmd_acc_target = cmd_acc_target(1:fs_acc/fs_acc_use:end);
cmd_acc_reference = cmd_acc_reference(1:fs_acc/fs_acc_use:end);
cmd_vel_guided = cmd_vel_guided(1:fs_acc/fs_acc_use:end);

t_acc_raw = t_acc_raw(1:fs_acc/fs_acc_use:end);
acc_x_raw = acc_x_raw(1:fs_acc/fs_acc_use:end);
acc_y_raw = acc_y_raw(1:fs_acc/fs_acc_use:end);
acc_z_raw = acc_z_raw(1:fs_acc/fs_acc_use:end);

fusion_roll = fusion_roll(1:fs_acc/fs_acc_use:end);
fusion_pitch = fusion_pitch(1:fs_acc/fs_acc_use:end);
fusion_yaw = fusion_yaw(1:fs_acc/fs_acc_use:end);

%% 将数据转换为公制
% GPS 源
% time s
t_gps = t_gps*1e-6;

% altitude m
h_gps = h_gps * 1e-3;

% velocity m/s
vel_n_gps = vel_n_gps*1e-2;
vel_e_gps = vel_e_gps*1e-2;
vel_d_gps = vel_d_gps*1e-2;

% RTK 源
t_rtk = t_rtk*1e-6;
h_rtk = h_rtk * 1e-2;
vel_d_rtk = vel_d_rtk*1e-2;
% h_rtk = h_rtk*10.0;

% 气压计 源
t_baro = t_baro*1e-6;
h_baro = h_baro * 1e-2;

% 融合高度 源
t_fusion_pos = t_fusion_pos*1e-6;
t_fusion_vel = t_fusion_vel*1e-6;
t_acc_cmd_pos = t_acc_cmd_pos*1e-6;
t_acc_fusion = t_acc_fusion*1e-6;

fusion_vel_n_ms = fusion_vel_n_cms*1e-2;
fusion_vel_e_ms = fusion_vel_e_cms*1e-2;
fusion_vel_d_ms = fusion_vel_d_cms*1e-2;

acc_z_fusion = acc_z_fusion*1e-2;

% 融合速度 源
t_acc_raw = t_acc_raw*1e-6;
t_acc_cmd = t_acc_cmd*1e-6;

cmd_acc_target = cmd_acc_target*1e-2;
cmd_acc_reference = cmd_acc_reference*1e-2;
cmd_vel_guided = cmd_vel_guided*1e-2;

% accel m/s^2
acc_x_raw = acc_x_raw*1e-2;
acc_y_raw = acc_y_raw*1e-2;
acc_z_raw = acc_z_raw*1e-2;

fusion_roll = fusion_roll*0.01;
fusion_pitch = fusion_pitch*0.01;
fusion_yaw = fusion_yaw*0.01;
%% 分段选择数据
switch flag_data
    
case 1  
% 全程
% time_acc_down = 88;
% time_acc_up = 720;
% acc_z_raw_offset = 9.88;

% time_acc_down = 540;
% time_acc_up = 600;
% acc_z_raw_offset = 9.88;

% 不变高，10m/s
% time_acc_down = 450;
% time_acc_up = 540;
% acc_z_raw_offset = 9.88;

% 升高，速度降低
% time_acc_down = 180;
% time_acc_up = 280;
% acc_z_raw_offset = 9.88;

% 飞书
% time_acc_down = 230;
% time_acc_up = 260;
% acc_z_raw_offset = 9.88;


% time_acc_down = 230;
% time_acc_up = 245;
% acc_z_raw_offset = 9.88;

% 降低，速度降低
% time_acc_down = 320;
% time_acc_up = 390;
% acc_z_raw_offset = 9.88;

% 升高、降低，速度降低
time_acc_down = 180;
time_acc_up = 390;
acc_z_raw_offset = 9.88;

% 多次升高、降低，速度降低（几乎是整个数据）
% time_acc_down = 180;
% time_acc_up = 650;
% acc_z_raw_offset = 9.88;

% time_acc_down = 625;
% time_acc_up = 650;
% acc_z_raw_offset = 9.88;

% id = find(t_baro == 5.426912050000000e+02)
% t_baro_jiao = t_baro(id);
% t_baro(id) = t_baro(id-1);
% t_baro(id-1) = t_baro(id-2);
% t_baro(id-2) = t_baro_jiao;

id = find(t_baro == 5.426912050000000e+02)
t_baro_jiao = t_baro(id);
t_baro(id) = t_baro(id-1);
t_baro(id-1) = t_baro_jiao;


case 2  
% 全程
time_acc_down = 65;
time_acc_up = 717;
acc_z_raw_offset = 9.88;

% time_acc_down = 214;
% time_acc_up = 400;
% acc_z_raw_offset = 9.88;

% time_acc_down = 100;
% time_acc_up = 300;
% acc_z_raw_offset = 9.88;

% time_acc_down = 350;
% time_acc_up = 500;
% acc_z_raw_offset = 9.88;


id = find(t_baro == 4.941612270000000e+02)
t_baro_jiao = t_baro(id);
t_baro(id) = t_baro(id-1);
t_baro(id-1) = t_baro_jiao;

    case 3  
time_acc_down = 85;
time_acc_up = 686;
acc_z_raw_offset = 9.88;

% time_acc_down = 200;
% time_acc_up = 500;
% acc_z_raw_offset = 9.88;

% 悬停，加速，减速
% time_acc_down = 282;
% time_acc_up = 363;
% acc_z_raw_offset = 9.88;

end
%% 去除偏置
h_rtk = h_rtk - base_height_rtk;
h_gps = h_gps - base_height_gps;
acc_z_raw = acc_z_raw - acc_z_raw_offset;

%% 补偿时间滞后
a1=0.0;
a2=0.0;
time_acc_down_2 = time_acc_down-a1;
time_acc_up_2 = time_acc_up-a2;
%% 获得可利用的数据
% GPS
time_gps_down = time_acc_down_2;
time_gps_up = time_acc_up_2;

index_time_gps = find((t_gps>=time_gps_down) & (t_gps<=time_gps_up));
t_gps = t_gps(index_time_gps);
vel_n_gps = vel_n_gps(index_time_gps);
vel_e_gps = vel_e_gps(index_time_gps);
vel_d_gps = vel_d_gps(index_time_gps);
h_gps = h_gps(index_time_gps);

%-------------------------------------------------------%
% RTK 
time_rtk_down = time_gps_down;
time_rtk_up = time_gps_up;

index_time_rtk = find((t_rtk>=time_rtk_down) & (t_rtk<=time_rtk_up));
t_rtk = t_rtk(index_time_rtk);
vel_d_rtk = vel_d_rtk(index_time_rtk);
h_rtk = h_rtk(index_time_rtk);

%-------------------------------------------------------%
% baro
time_baro_down = time_rtk_down;
time_baro_up = time_rtk_up;

index_time_baro = find((t_baro>=time_baro_down) & (t_baro<=time_baro_up));
t_baro = t_baro(index_time_baro);
h_baro = h_baro(index_time_baro);

% 融合高度
time_fusion_down = time_gps_down;
time_fusion_up = time_gps_up;

index_time_fusion = find((t_fusion_pos>=time_fusion_down) & (t_fusion_pos<=time_fusion_up));
t_fusion_pos = t_fusion_pos(index_time_fusion);
fusion_pos_alt_cm = fusion_pos_alt_cm(index_time_fusion);

index_time_fusion_v = find((t_fusion_vel>=time_fusion_down) & (t_fusion_vel<=time_fusion_up));
t_fusion_vel = t_fusion_vel(index_time_fusion_v);
fusion_vel_n_ms = fusion_vel_n_ms(index_time_fusion_v);
fusion_vel_e_ms = fusion_vel_e_ms(index_time_fusion_v);
fusion_vel_d_ms = fusion_vel_d_ms(index_time_fusion_v);

index_time_fusion_acc_cmd_pos = find((t_acc_cmd_pos>=time_fusion_down) & (t_acc_cmd_pos<=time_fusion_up));
t_acc_cmd_pos = t_acc_cmd_pos(index_time_fusion_acc_cmd_pos);
cmd_acc_target_pos = cmd_acc_target_pos(index_time_fusion_acc_cmd_pos);
cmd_vel_guided_pos = cmd_vel_guided_pos(index_time_fusion_acc_cmd_pos); 

index_time_fusion_a = find((t_acc_fusion>=time_fusion_down) & (t_acc_fusion<=time_fusion_up));
t_acc_fusion = t_acc_fusion(index_time_fusion_a);
acc_z_fusion = acc_z_fusion(index_time_fusion_a);

% 融合速度
index_time_cmd_acc = find((t_acc_cmd>=time_fusion_down) & (t_acc_cmd<=time_fusion_up));
t_acc_cmd = t_acc_cmd(index_time_cmd_acc);
cmd_acc_target = cmd_acc_target(index_time_cmd_acc);
cmd_acc_reference = cmd_acc_reference(index_time_cmd_acc);
cmd_vel_guided = cmd_vel_guided(index_time_cmd_acc); 
    
index_time_raw_a = find((t_acc_raw>=time_fusion_down) & (t_acc_raw<=time_fusion_up));
t_acc_raw = t_acc_raw(index_time_raw_a);
acc_x_raw = acc_x_raw(index_time_raw_a);
acc_y_raw = acc_y_raw(index_time_raw_a);
acc_z_raw = acc_z_raw(index_time_raw_a);

fusion_roll = fusion_roll(index_time_raw_a);
fusion_pitch = fusion_pitch(index_time_raw_a);
fusion_yaw = fusion_yaw(index_time_raw_a);

%% 修正不合理的数据
% if(flag_data == 1)
%     id_error = find(t_acc_raw == 5.426986130000000e+02);
%     t_acc_raw(id_error(end))=542.70;
% end

%% 计算水平速度
vel_horizon_gps = [];
for(i=1:length(vel_n_gps))
    vel_horizon_gps(i) = sign(vel_e_gps(i))*sqrt(vel_n_gps(i)*vel_n_gps(i)+vel_e_gps(i)*vel_e_gps(i));
end

vel_horizon_fusion = [];
for(i=1:length(fusion_vel_e_ms))
    vel_horizon_fusion(i) = sign(fusion_vel_e_ms(i))*sqrt(fusion_vel_e_ms(i)*fusion_vel_e_ms(i)+fusion_vel_n_ms(i)*fusion_vel_n_ms(i));
end

%% 计算融合水平速度
% lpf_a = 0.99; % 0.2Hz
lpf_a = 0.995; % 0.1Hz
% lpf_a = 0.9975; % 0.Hz

vel_horizon_fusion_lpf = [vel_horizon_fusion(1)];
vel_vertical_fusion_lpf = [fusion_vel_d_ms(1)];
for(i=2:length(t_fusion_vel))
    vel_horizon_fusion_lpf(i) = lpf_a*vel_horizon_fusion_lpf(i-1) + (1-lpf_a)*vel_horizon_fusion(i);
    vel_vertical_fusion_lpf(i) = lpf_a*vel_vertical_fusion_lpf(i-1) + (1-lpf_a)*fusion_vel_d_ms(i);
end

%% 定义一些变量
count_rtk_updated = 1;
count_gps_updated = 1;
count_baro_updated = 1;

t_gps_cal_tmp = -5;
t_rtk_cal_tmp = -5;
t_baro_cal_tmp = -5;

t_acc_cal_tmp = -5;

%% kalman变量
% x_hat_k = [0.1;0.1;0.1;0.1];
% x_hat_k = [h_rtk(1);0;0;0.0];
x_hat_k = [h_rtk(1);0;0;0.0];
x_hat_k1 = x_hat_k;

x_hat_k_k1 = x_hat_k;

% p_k = [1 0 0 0;...
%        0 1 0 0;...
%        0 0 1 0;...
%        0 0 0 1];
   
% p_k = [1 0 0 0;...
%        0 1 0 0;...
%        0 0 1000 0;...
%        0 0 0 1000];

p_k = [1000 0 0 0;...
       0 1 0 0;...
       0 0 1000 0;...
       0 0 0 1000];

p_k1 = p_k;
p_k_k1 = p_k;

state_height = [];
state_vel = [];
state_acc_true = [];
state_delta_accel = [];

t_acc_kalman = [];

%% RTK方差参数
% std_vel_gps = 0.2;
% std_vel_gps = 1.5;
% std_acc = 0.05;

std_vel_rtk = 0.2;
var_rtk = std_vel_rtk*std_vel_rtk;

%% kalmanQ矩阵方差参数
% 好
% std_dacc_q = 0.1;
% std_acc_q = 5.0;
% std_gps_q = 5;
% std_baro_q = 5;


% std_dacc_q = 0.1;
% std_acc_q = 5.0;
% std_gps_q = 0.1;
% std_baro_q = 0;

% std_dacc_q = 0.1;
% std_acc_q = 5.0;
% std_gps_q = 0.1;
% std_baro_q = 5.0;

% std_dacc_q = 0.1;
% std_acc_q = 5.0;
% std_gps_q = 0.1;
% std_baro_q = 0.0;

% std_dacc_q = 0.01;
% std_acc_q = 10.0;
% std_gps_q = 0.1;
% std_baro_q = 5.0;

std_dacc_q = 4.0;
std_acc_q = 3.0;
std_gps_q = 1;
% std_baro_q = 5;
std_baro_q = 1;

var_dacc_q = std_dacc_q*std_dacc_q;
var_acc_q = std_acc_q*std_acc_q;
var_gps_q = std_gps_q*std_gps_q;
var_baro_q = std_baro_q*std_baro_q;

%% kalman R矩阵方差参数
% std_acc = 30000;
% std_acc = 3.5;
% std_acc = 0.0063;
std_acc = 1;
var_acc = std_acc*std_acc;

% std_vel_gps = 5000.0;
% std_vel_gps = 10; % 0.26
% std_vel_gps = 8; % 0.32
% std_vel_gps = 6; % 0.4
std_vel_gps = 4; % 0.48
var_gps = std_vel_gps*std_vel_gps; 

%% 残差检测、kalman增益存储
t_innov_h = [];
t_innov_v = [];
t_innov_a = [];

innov_h_store = [];
innov_v_store = [];
innov_a_store = [];

innov_h = 0;
innov_v = 0;
innov_a = 0;

kalman_gain_h = [];
kalman_gain_v = [];
kalman_gain_a = [];

kalman_gain_h_quan = [];
kalman_gain_v_quan = [];
kalman_gain_a_quan = [];

%% 加速度指令参数
count_acc_cmd = 1;
% alpha_t = 0.1;
% alpha_t = 0.0025; % 0.9975
% alpha_t = 0.0125; % 0.9875
% alpha_t = 0.1; % 
% alpha_t = 0.15; % 
% alpha_t = 0.2; % 
alpha_t = 0.3; % 
% alpha_t = 0.0; % 

acc_cmd_cal = [];
t_acc_cmd_cal = [];
%% GPS速度 洗出
% 0.01Hz
% num = [0.9969 -0.9969]
% den = [1 -0.9937]

% 0.02Hz
% num = [0.9938 -0.9938]
% den = [1 -0.9875]

% 0.1Hz
% num = [0.9695 -0.9695]
% den = [1 -0.9391]

% 0.2Hz
% num = [0.9409 -0.9409]
% den = [1 -0.8818]

% 0.3Hz
num = [0.9139 -0.9139]
den = [1 -0.8277]

% 0.4Hz
% num = [0.8884 -0.8884]
% den = [1 -0.7767]

% 0.5Hz
% num = [0.8642 -0.8642]
% den = [1 -0.7285]

% 2.8Hz
% num = [0.4431 -0.4431]
% den = [1 0.1137]

% 3.45Hz
% num = [0.3466 -0.3466]
% den = [1 0.3067]

vel_d_gps_filter = filter(num,den,vel_d_gps);

[H_gps, freq_gps] = freqz(num,den,1024,10);
phase_gps = unwrap(angle(H_gps)) * (180/pi);

%% 设计加速度低通滤波器
wp = 4/(fs_acc_use/2);  
ws = 5/(fs_acc_use/2);  

alpha_p = 3;
alpha_s = 6;

% [N_acc_raw wc_acc_raw] = buttord(wp,ws,alpha_p,alpha_s)
% [b_acc_raw a_acc_raw] = butter(N_acc_raw,wc_acc_raw,'low');

% [b_acc_raw a_acc_raw] = butter(1,wp,'low');
[b_acc_raw a_acc_raw] = butter(2,wp,'low');

[H_acc_raw, freq_acc_raw] = freqz(b_acc_raw,a_acc_raw,1024,fs_acc_use);

phase_acc_raw = unwrap(angle(H_acc_raw)) * (180/pi);

acc_x_lpf = filter(b_acc_raw,a_acc_raw,acc_x_raw);
acc_y_lpf = filter(b_acc_raw,a_acc_raw,acc_y_raw);
acc_z_lpf = filter(b_acc_raw,a_acc_raw,acc_z_raw);

%% 机体加速度转NED加速度
acc_x_lpf_ef = [];
acc_y_lpf_ef = [];
acc_z_lpf_ef = [];

deg_to_rad = 180.0/pi;
for(i=1:length(acc_z_lpf))
    % 计算DCM方向余弦矩阵 sin对弧度进行操作
    sr = sin(fusion_roll(i)/deg_to_rad);
    cr = cos(fusion_roll(i)/deg_to_rad);
    sp = sin(fusion_pitch(i)/deg_to_rad);
    cp = cos(fusion_pitch(i)/deg_to_rad);
    sy = sin(fusion_yaw(i)/deg_to_rad);
    cy = cos(fusion_yaw(i)/deg_to_rad);

    dcm_a_x = cp * cy;
    dcm_a_y = (sr * sp * cy) - (cr * sy);
    dcm_a_z = (cr * sp * cy) + (sr * sy);
    dcm_b_x = cp * sy;
    dcm_b_y = (sr * sp * sy) + (cr * cy);
    dcm_b_z = (cr * sp * sy) - (sr * cy);
    dcm_c_x = -sp;
    dcm_c_y = sr * cp;
    dcm_c_z = cr * cp; 

    dcm = [dcm_a_x dcm_a_y dcm_a_z;...
           dcm_b_x dcm_b_y dcm_b_z;...
           dcm_c_x dcm_c_y dcm_c_z];
    
    acc_x_lpf_ef(i) = dcm_a_x*acc_x_lpf(i) + dcm_a_y*acc_y_lpf(i) + dcm_a_z*acc_z_lpf(i);   
    acc_y_lpf_ef(i) = dcm_b_x*acc_x_lpf(i) + dcm_b_y*acc_y_lpf(i) + dcm_b_z*acc_z_lpf(i);   
    acc_z_lpf_ef(i) = dcm_c_x*acc_x_lpf(i) + dcm_c_y*acc_y_lpf(i) + dcm_c_z*acc_z_lpf(i);   

end
    
%% 构造加速度指令
% t_acc_cmd = t_acc_raw;
% cmd_acc_target = zeros(size(acc_z_lpf_ef));

cmd_acc_input = [0];
for(i=2:length(t_acc_cmd))
    if(t_acc_cmd(i) > t_acc_cmd(i-1))
        dt_cmd = t_acc_cmd(i) - t_acc_cmd(i-1);
        cmd_acc_input(i) = (cmd_vel_guided(i)-cmd_vel_guided(i-1))/dt_cmd;
    else
        cmd_acc_input(i) = 0;
    end
end

%% 计算水平加速度
acc_horizon_lpf_ef = [];
for(i=1:length(t_acc_raw))
%     acc_horizon_lpf_ef(i) = sign(acc_x_lpf_ef(i))*sqrt(acc_x_lpf_ef(i)*acc_x_lpf_ef(i)+acc_y_lpf_ef(i)*acc_y_lpf_ef(i));
    acc_horizon_lpf_ef(i) =sqrt(acc_x_lpf_ef(i)*acc_x_lpf_ef(i)+acc_y_lpf_ef(i)*acc_y_lpf_ef(i));
end
%% 对速度指令做低通滤波 二阶巴特沃斯低通滤波器
% wp_v_cmd = 0.2/(fs_acc_use/2);  
% ws_v_cmd = 0.4/(fs_acc_use/2);  

% alpha_p_v_cmd = 3;
% alpha_s_v_cmd = 6;

% [N_v_cmd wc_v_cmd] = buttord(wp_v_cmd,ws_v_cmd,alpha_p_v_cmd,alpha_s_v_cmd)
% [b_v_cmd a_v_cmd] = butter(N_v_cmd,wc_v_cmd,'low')

% 0.6Hz大体能够跟上
fc = 0.6/(fs_acc_use/2); 
wc_v_cmd =  fc;

[b_v_cmd a_v_cmd] = butter(2,wc_v_cmd,'low')

[H_v_cmd, freq_v_cmd] = freqz(b_v_cmd,a_v_cmd,1024,fs_acc_use);

phase_v_cmd = unwrap(angle(H_v_cmd)) * (180/pi);

cmd_vel_guided_lpf = filter(b_v_cmd,a_v_cmd,cmd_vel_guided);

%% 对二阶低通速度指令做低通
% 0.3Hz
a = 0.98; 

% 0.5Hz
% a = 0.97; 

% 2.5Hz
% a = 0.85; 

num = [1-a]
den = [1,-a]

cmd_vel_guided_lpf = filter(num,den,cmd_vel_guided_lpf);

%% 对加速度测量值洗出
% 0.05Hz
% num_acc_mes = [0.9984 -0.9984]
% den_acc_mes = [1 -0.9969]

% 0.1Hz
num_acc_mes = [0.9969 -0.9969]
den_acc_mes = [1 -0.9937]

% 0.25Hz
% num_acc_mes = [0.9938 -0.9938]
% den_acc_mes = [1 -0.9875]

% 0.53Hz
% num_acc_mes = [0.9845 -0.9845]
% den_acc_mes = [1 -0.9691]

acc_z_md_ef = filter(num_acc_mes,den_acc_mes,acc_z_lpf_ef);

[H_acc_mes, freq_acc_mes] = freqz(num_acc_mes,den_acc_mes,1024,fs_acc_use);
phase_acc_mes = unwrap(angle(H_acc_mes)) * (180/pi);

%% 预测
pred_state = [];

%% 气压高度的使用
th_baro = 2.5; % 地效高度，单位m
rate_baro = [0];
h_baro_lpf = [h_baro(1)];

std_h_baro_good = 20000.0; 
% std_h_baro_good = 8000.0; 
% std_h_baro_good = 7000.0; 
% std_h_baro_good = 3000.0; % 0.0166
% std_h_baro_good = 2000.0; % 0.025
% std_h_baro_good = 1500.0; 
% std_h_baro_good = 1000.0; 
% std_h_baro_good = 15.0; 
% std_h_baro_good = 2.0; 
std_h_baro = std_h_baro_good;
var_baro = std_h_baro*std_h_baro;

est_minus_baro = 0;
est_minus_baro_lpf = 0;
coef_lpf_baro = 0.99;
est_minus_baro_store = [];

h_baro_input = [];

flag_baro = [];
t_flag_baro = [];

t_baro_bianli = [];
h_baro_bianli = [];
t_baro_cal_tmp_2 = -5;
%% 计算气压速度和气压高度的低通滤波
for(i=2:length(t_baro))
    dt_baro = t_baro(i)-t_baro(i-1);
    if(dt_baro > 0)
        rate_baro(i) = (h_baro(i)-h_baro(i-1))/dt_baro;
        h_baro_lpf(i) = lpf_a*h_baro_lpf(i-1) + (1-lpf_a)*h_baro(i);
    else
        rate_baro(i) = 0;
        h_baro_lpf(i) = h_baro_lpf(i-1);
    end
    
end
%% 遍历水平速度，与气压匹配
bian_baro = 1;
t_fusion_vel_baro = [];
vel_horizon_fusion_baro = [];

for(i=1:length(t_fusion_vel))
    if(t_fusion_vel(i)>=t_baro(bian_baro))
%         t_fusion_vel_baro = [t_fusion_vel_baro;t_baro(bian_baro)];
        t_fusion_vel_baro = [t_fusion_vel_baro;t_fusion_vel(i)];
        vel_horizon_fusion_baro = [vel_horizon_fusion_baro;vel_horizon_fusion(i)];
        
        if(bian_baro<=(length(t_baro)-1))
            bian_baro = bian_baro+1;
        end
    end
end
%% Kalman滤波
tic
counter_loop = 0;
dt = 0;
dt_store = [];

fai_k_k1 = [1 0.01 0 0;...
            0 1 0.01 0;...
            0 0 1-alpha_t 0;...
            0 0 0 1];
                     
B = [0;0;alpha_t;0];

rou_k1 = [1 0 0 0;...
          0 1 0 0;...
          0 0 1 0;...
          0 0 0 1];

 Q_k1 = [var_baro_q 0 0 0;...
         0 var_gps_q 0 0;...
         0 0 var_acc_q 0;...
         0 0 0 var_dacc_q];
                 
for(i=1:1:length(acc_z_lpf_ef)) 
    if(i == 1) % 第1个量
        state_height = [state_height h_rtk(1)];
        state_vel = [state_vel vel_d_gps(1)];
        state_acc_true = [state_acc_true acc_z_lpf_ef(1)];
        state_delta_accel = [state_delta_accel 0];
        
        pred_state = [pred_state [state_height;state_vel;state_acc_true;state_delta_accel]];
        
        if((t_acc_raw(i) > t_acc_cal_tmp))
            dt = 0.01;    
            t_acc_cal_tmp = t_acc_raw(1);
        end % 加计更新结束
        
        if((t_acc_raw(i) >= t_gps(count_gps_updated)) &&...
           (t_gps(count_gps_updated) > t_gps_cal_tmp) &&...
           (count_gps_updated <= length(t_gps)) ) 
                t_gps_cal_tmp = t_gps(count_gps_updated);
           
                if(count_gps_updated <= length(t_gps)-1)
                    count_gps_updated = count_gps_updated+1;
                end 
        end % GPS速度更新结束    
        
        %--------------------------------------------%
        % 气压计高度更新
        if((t_acc_raw(i) > t_baro(count_baro_updated)) &&...
           (t_baro(count_baro_updated) > t_baro_cal_tmp) &&...
           (count_baro_updated <= length(t_baro)) ) 
                t_baro_cal_tmp_2 = t_baro(count_baro_updated);

                if(count_baro_updated <= length(t_baro)-1)
                    count_baro_updated = count_baro_updated+1;
                end 
        end % 气压计高度更新结束
            
    end   % 第1个量结束
    
    %--------------------------------------------%
    if(i >= 2) % 第1个量结束，从第2个量开始
        
        % IMU更新
        if((t_acc_raw(i) > t_acc_cal_tmp))
            dt = t_acc_raw(i)-t_acc_cal_tmp;

            fai_k_k1 = [1 dt 0 0;...
                        0 1 dt 0;...
                        0 0 1-alpha_t 0;...
                        0 0 0 1];
                
            x_hat_k_k1 = fai_k_k1*x_hat_k1 + B*[cmd_acc_input(i)];
            p_k_k1 = fai_k_k1*p_k1*fai_k_k1' + rou_k1*Q_k1*rou_k1';  

            pred_state = [pred_state x_hat_k_k1];
            
            z_k = [acc_z_md_ef(i)];

            R_k = [var_acc];
            H_k = [0 0 1 1];
            K_k = p_k_k1*H_k'*inv(H_k*p_k_k1*H_k'+R_k);

            innov = z_k - H_k*x_hat_k_k1;
            x_hat_k_k1 = x_hat_k_k1 + K_k*(innov);
            p_k_k1 = (eye(length(x_hat_k)) - K_k*H_k)*p_k_k1;
            
            innov_a = innov;
            innov_a_store = [innov_a_store innov_a];
                
            kalman_gain_a_quan = [kalman_gain_a_quan [K_k(1);K_k(2);K_k(3);K_k(4)]];
            
            dt_store = [dt_store dt];
            t_innov_a = [t_innov_a t_acc_raw(i)];
            
            t_acc_cal_tmp = t_acc_raw(i);                  
        end % 加计更新结束
        
            %--------------------------------------------%
            % GPS速度量测更新的部分
            if((t_acc_raw(i) >= t_gps(count_gps_updated)) &&...
               (t_gps(count_gps_updated) > t_gps_cal_tmp) &&...
               (count_gps_updated <= length(t_gps)) ) 
%                 z_k = [vel_d_gps(count_gps_updated)];
%                 z_k = [vel_d_gps_filter(count_gps_updated)];
                z_k = [vel_d_gps_filter(count_gps_updated)+cmd_vel_guided_lpf(i)];
                 
                R_k = [var_gps];
                H_k = [0 1 0 0];
                K_k = p_k_k1*H_k'*inv(H_k*p_k_k1*H_k'+ R_k);
                
                innov = z_k - H_k*x_hat_k_k1;
                x_hat_k_k1 = x_hat_k_k1 + K_k*innov;

                p_k_k1 = (eye(length(x_hat_k)) - K_k*H_k)*p_k_k1;

                kalman_gain_v = [kalman_gain_v K_k(2)];
                kalman_gain_v_quan = [kalman_gain_v_quan [K_k(1);K_k(2);K_k(3);K_k(4)]];
                
                innov_v = innov;
                innov_v_store = [innov_v_store innov_v];
                t_innov_v = [t_innov_v t_gps(count_gps_updated)];

                t_gps_cal_tmp = t_gps(count_gps_updated);
                if(count_gps_updated <= length(t_gps)-1)
                    count_gps_updated = count_gps_updated+1;
                end 
            end % GPS速度更新结束
            
            %--------------------------------------------%
            x_hat_k = x_hat_k_k1;
            p_k = p_k_k1;

            state_height(i) = x_hat_k(1);
            state_vel(i) = x_hat_k(2);
            state_acc_true(i) = x_hat_k(3);
            state_delta_accel(i) = x_hat_k(4);
            
            %--------------------------------------------%
            % 更新状态变量（上一时刻）
            x_hat_k1 = x_hat_k;
            p_k1 = p_k;
            
            %--------------------------------------------%
            % 更新上一时刻的加速度时间
            

        end  % 第2个量到结束

    t_acc_kalman = [t_acc_kalman t_acc_raw(i)];
    counter_loop = counter_loop + 1;
end % 框架遍历结束
toc
disp(['运行时间: ',num2str(toc)]);
disp(['加速度个数: ',num2str(length(t_acc_raw))]);
disp(['迭代次数: ',num2str(counter_loop)]);

%% GPS速度积分
h_gps_vel_integ = [h_rtk(1)];
h_gps_vel_filter_integ = [h_rtk(1)];
for i=2:length(t_gps)
    dt = t_gps(i) - t_gps(i-1);
    h_gps_vel_integ(i) = h_gps_vel_integ(i-1) + dt*vel_d_gps(i-1);
    h_gps_vel_filter_integ(i) = h_gps_vel_filter_integ(i-1) + dt*vel_d_gps_filter(i-1);
end

%% RTK速度积分
h_rtk_vel_integ = [h_rtk(1)];
for i=2:length(t_rtk)
    dt = t_rtk(i) - t_rtk(i-1);
    h_rtk_vel_integ(i) = h_rtk_vel_integ(i-1) + dt*vel_d_rtk(i-1);
end

%% 加速度积分
v_acc = [0];
% v_acc = [vel_d_rtk(1)];
h_acc = [h_rtk(1)];
h_v_est = [h_rtk(1)];

for(i=2:length(t_acc_kalman))
    dt = t_acc_kalman(i)-t_acc_kalman(i-1);
    v_acc(i) = v_acc(i-1) + state_acc_true(i-1)*dt; 
end
for(i=2:length(t_acc_kalman))
    dt = t_acc_kalman(i)-t_acc_kalman(i-1);
    
    h_acc(i) = h_acc(i-1) + v_acc(i-1)*dt; 
    h_v_est(i) = h_v_est(i-1) + state_vel(i-1)*dt; 
end

v_acc_cmd = [0];
% v_acc_cmd = [vel_d_rtk(1)];
h_acc_cmd = [h_rtk(1)];
for(i=2:length(t_acc_cmd))
    dt = t_acc_cmd(i)-t_acc_cmd(i-1);
    v_acc_cmd(i) = v_acc_cmd(i-1) + cmd_acc_input(i-1)*dt;     
end
for(i=2:length(t_acc_cmd))
    dt = t_acc_cmd(i)-t_acc_cmd(i-1);
    h_acc_cmd(i) = h_acc_cmd(i-1) + v_acc_cmd(i-1)*dt; 
end

%% 加速度低通滤波器频率响应
figure
subplot(2,1,1)
hold on
plot(freq_acc_raw,20*log10(abs(H_acc_raw)),'b-','linewidth',2);
plot(freq_acc_raw,-3*ones(size(H_acc_raw)),'k-','linewidth',2);
grid on
title('加速度测量值低通滤波器频响')
xlabel('频率/Hz');
ylabel('幅度/dB');
xlim([0 6])

subplot(2,1,2)
hold on
plot(freq_acc_raw, phase_acc_raw);
grid on;
xlim([0 6])
xlabel('frequency(Hz)');
ylabel('phase/°');
grid on
hold off

%% 画图—速度指令的二阶低通滤波频率响应
% figure
% subplot(2,1,1)
% hold on
% plot(freq_v_cmd,20*log10(abs(H_v_cmd)),'b-','linewidth',2);
% plot(freq_v_cmd,-3*ones(size(H_v_cmd)),'k-','linewidth',2);
% grid on
% title('速度指令二阶低通滤波器频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% xlim([0 1])
% 
% subplot(2,1,2)
% hold on
% plot(freq_v_cmd, phase_v_cmd);
% grid on;
% xlim([0 1])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off
%% 速度指令二阶低通时域响应
% figure;
% plot(t_acc_cmd,cmd_vel_guided*10,'o','Color','r','linewidth',2);
% hold on
% plot(t_fusion_vel,fusion_vel_d_ms*10,'-','Color','b','linewidth',2);
% plot(t_acc_cmd,cmd_vel_guided_lpf*10,'-','Color','k','linewidth',2);
% hold off
% grid on
% axis tight
% title('速度指令二阶低通响应')
% xlabel('time/s');
% ylabel('vel/(dm/s)');
%% 画图—速度指令的一阶低通滤波频率响应
% figure
% subplot(2,1,1)
% hold on
% plot(freq_v_cmd_1,20*log10(abs(H_v_cmd_1)),'b-','linewidth',2);
% plot(freq_v_cmd_1,-3*ones(size(H_v_cmd_1)),'k-','linewidth',2);
% grid on
% title('速度指令一阶低通滤波器频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% xlim([0 1])
% 
% subplot(2,1,2)
% hold on
% plot(freq_v_cmd_1, phase_v_cmd_1);
% grid on;
% xlim([0 1])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off
%% 速度指令一阶低通时域响应
% figure;
% plot(t_acc_cmd,cmd_vel_guided*10,'o','Color','r','linewidth',2);
% hold on
% plot(t_fusion_vel,fusion_vel_d_ms*10,'-','Color','b','linewidth',2);
% plot(t_acc_cmd,cmd_vel_guided_lpf_1*10,'-','Color','k','linewidth',2);
% plot(t_acc_cmd,cmd_vel_guided_lpf_2*10,'m--','linewidth',2);
% plot(t_acc_cmd,cmd_vel_guided_lpf*10,'g-','linewidth',2);
% hold off
% grid on
% axis tight
% title('速度指令一阶低通响应')
% xlabel('time/s');
% ylabel('vel/(dm/s)');
%% 画图—GPS速度洗出 高通滤波器频率响应
% figure
% subplot(2,1,1)
% plot(freq_gps,20*log10(abs(H_gps)),'b-','linewidth',2);
% grid on
% title('GPS速度高通频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% xlim([0 5])
% 
% subplot(2,1,2)
% hold on
% plot(freq_gps, phase_gps);
% grid on;
% xlim([0 5])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off
%% GPS速度洗出时域响应
% figure;
% plot(t_gps,vel_d_gps*10,'-s','Color','r','linewidth',2);
% hold on
% plot(t_gps,vel_d_gps_filter*10,'-s','Color','b','linewidth',2);
% hold off
% grid on
% axis tight
% title('速度')
% xlabel('time/s');
% ylabel('vel/(dm/s)');
%% 画图—加速度低通滤波频率响应
% figure
% subplot(2,1,1)
% plot(freq_acc_raw,20*log10(abs(H_acc_raw)),'b-','linewidth',2);
% grid on
% title('加速度低通滤波器频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% xlim([0 5])
% 
% subplot(2,1,2)
% hold on
% plot(freq_acc_raw, phase_acc_raw);
% grid on;
% xlim([0 5])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off
%% 画图—加速度洗出 高通滤波器频率响应
% figure
% subplot(2,1,1)
% plot(freq_acc_mes,20*log10(abs(H_acc_mes)),'b-','linewidth',2);
% grid on
% title('加速度高通频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% xlim([0 5])
% 
% subplot(2,1,2)
% hold on
% plot(freq_acc_mes, phase_acc_mes);
% grid on;
% xlim([0 5])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off


%% 高度、速度、加速度测量
% figure
% subplot(2,1,1)
% hold on
% plot(t_rtk,h_rtk,'r-','linewidth',2);
% plot(t_baro,h_baro,'b-','linewidth',2);
% plot(t_gps,h_gps_vel_integ,'m-','linewidth',2);
% hold off
% grid on
% axis tight
% 
% hold on
% plot(t_gps,vel_d_gps,'r-','linewidth',2);
% plot(t_gps,vel_d_gps_filter,'b-','linewidth',2);
% plot(t_acc_raw,cmd_vel_guided,'k-','linewidth',2);
% plot(t_acc_raw,cmd_acc_input,'m-','linewidth',2);
% plot(t_acc_raw,cmd_acc_target,'b--','linewidth',2);
% hold off
% grid on
% axis tight
% legend('RTK高度','气压高度','GPS速度积分','GPS速度','GPS速度洗出','规划速度','规划速度微分','加速度target指令')
% xlabel('time/s');
% title('高度和速度测量值')
% 
% subplot(2,1,2)
% hold on
% plot(t_rtk,h_rtk,'r-','linewidth',2);
% plot(t_baro,h_baro,'b-','linewidth',2);
% plot(t_gps,h_gps_vel_integ,'m-','linewidth',2);
% grid on
% 
% hold on
% plot(t_acc_raw,acc_z_raw,'r-','linewidth',2);
% plot(t_acc_fusion,acc_z_fusion,'k-','linewidth',2);
% plot(t_acc_raw,acc_z_lpf_ef,'b-','linewidth',2);
% axis tight
% legend('RTK高度','气压高度','GPS速度积分','原始加速度','融合加速度','原始加速度(lpf-ef)')
% xlabel('time/s');
% title('高度和加速度测量值')

%% 高度、速度、加速度测量2
% figure
% hold on
% plot(t_rtk,h_rtk,'b-','linewidth',2);
% plot(t_fusion_vel,vel_horizon_fusion,'k-','linewidth',2);
% 
% plot(t_acc_raw,acc_x_lpf_ef,'r-','linewidth',2);
% plot(t_acc_raw,acc_y_lpf_ef,'b-','linewidth',2);
% plot(t_acc_raw,acc_z_lpf_ef,'k-','linewidth',2);
% plot(t_acc_raw,acc_horizon_lpf_ef,'m-','linewidth',2);
% 
% axis tight
% % legend('RTK高度','气压高度','GPS速度积分','原始加速度','融合加速度','原始加速度(lpf-ef)')
% xlabel('time/s');
% title('高度和水平加速度')
% grid on
%% kalman增益
figure;
subplot(2,1,1)
% plot(t_innov_v,innov_v_store,'g-','linewidth',2);
hold on
plot(t_innov_v,kalman_gain_v_quan(1,:)*10,'-','Color','r','linewidth',2);
plot(t_innov_v,kalman_gain_v_quan(2,:)*10,'-','Color','b','linewidth',2);
plot(t_innov_v,kalman_gain_v_quan(3,:)*10,'-','Color','k','linewidth',2);
plot(t_innov_v,kalman_gain_v_quan(4,:)*10,'-','Color','m','linewidth',2);
% legend('速度新息','高度增益','速度增益','加速度增益','加速度漂移增益')
legend('高度增益','速度增益','加速度增益','加速度漂移增益')
hold off
grid on
axis tight
title('融合速度')
xlabel('time/s');
ylabel('vel/(dm/s)');

subplot(2,1,2)
% plot(t_innov_a,innov_a_store*10,'g-','linewidth',2);
hold on
plot(t_innov_a,kalman_gain_a_quan(1,:)*10,'-','Color','r','linewidth',2);
plot(t_innov_a,kalman_gain_a_quan(2,:)*10,'-','Color','b','linewidth',2);
plot(t_innov_a,kalman_gain_a_quan(3,:)*10,'-','Color','k','linewidth',2);
plot(t_innov_a,kalman_gain_a_quan(4,:)*10,'--','Color','m','linewidth',2);
hold off
grid on
axis tight
% legend('加速度新息','高度增益','速度增益','加速度增益','加速度漂移增益')
legend('高度增益','速度增益','加速度增益','加速度漂移增益')
title('融合加速度')
xlabel('time/s');

%% 计算观测器的传递函数(加速度指令和测量值)
[m,n] = size(kalman_gain_a_quan);
kalman_gain_a_quan(3,n)

K_acc = [kalman_gain_a_quan(1,n);kalman_gain_a_quan(2,n);kalman_gain_a_quan(3,n);kalman_gain_a_quan(4,n)]
H_acc = [0 0 1 1]
K_H_acc =  K_acc * H_acc
I1 = eye(size(K_H_acc));
I1_minus_KH_acc = I1 - K_H_acc
A3 = I1_minus_KH_acc*fai_k_k1
B1 = I1_minus_KH_acc * B
B2 = K_acc
B3 = [B1 B2]

% C3 = [0 0 1 0]
C3 = eye(4,4)

syms z;

I3 = eye(size(A3))
zi_minus_a = z*I3 - A3;
zi_minus_a = vpa(zi_minus_a,4)

zi_minus_a_inv = inv(zi_minus_a);
zi_minus_a_inv = vpa(zi_minus_a_inv,4)

H_z = C3 * zi_minus_a_inv * B3;
H_z = vpa(H_z,4);
disp('传递函数 H(z):');
disp(H_z);

% 化简传递函数
H_z_simplified = simplify(H_z);
H_z_simplified = vpa(H_z_simplified,4);
disp('化简后的传递函数 H(z):');
disp(H_z_simplified);

% 提取 h1 和 h2
h_acmd = H_z_simplified(1,1);
h_amd = H_z_simplified(1,2);
v_acmd = H_z_simplified(2,1);
v_amd = H_z_simplified(2,2);
a_acmd = H_z_simplified(3,1);
a_amd = H_z_simplified(3,2);
da_acmd = H_z_simplified(4,1);
da_amd = H_z_simplified(4,2);

% 获取 acmd->x_hat 和 amd->x_hat 的分子和分母
[num_h_acmd, den_h_acmd] = numden(h_acmd);
[num_h_amd, den_h_amd] = numden(h_amd);

[num_v_acmd, den_v_acmd] = numden(v_acmd);
[num_v_amd, den_v_amd] = numden(v_amd);

[num_a_acmd, den_a_acmd] = numden(a_acmd);
[num_a_amd, den_a_amd] = numden(a_amd);

[num_da_acmd, den_da_acmd] = numden(da_acmd);
[num_da_amd, den_da_amd] = numden(da_amd);

%% acmd->h 和cmd->h 脉冲传递函数
% 获取acmd->h 和cmd->h系数并转换为双精度数值
num_h_acmd_coeff = double(coeffs(num_h_acmd, z, 'All'));
den_h_acmd_coeff = double(coeffs(den_h_acmd, z, 'All'));
num_h_acmd_coeff = num_h_acmd_coeff/den_h_acmd_coeff(1);
den_h_acmd_coeff = den_h_acmd_coeff/den_h_acmd_coeff(1);

num_h_amd_coeff = double(coeffs(num_h_amd, z, 'All'));
den_h_amd_coeff = double(coeffs(den_h_amd, z, 'All'));
num_h_amd_coeff = num_h_amd_coeff/den_h_amd_coeff(1);
den_h_amd_coeff = den_h_amd_coeff/den_h_amd_coeff(1);

% 显示结果
disp('传递函数 a cmd->h 的分子系数:');
disp(num_h_acmd_coeff);
disp('传递函数 a cmd->h 的分母系数:');
disp(den_h_acmd_coeff);

disp('传递函数 a md->h 的分子系数:');
disp(num_h_amd_coeff);
disp('传递函数 a md->h 的分母系数:');
disp(den_h_amd_coeff);

%% acmd->v 和cmd->v 脉冲传递函数
% 获取acmd->v 和cmd->v系数并转换为双精度数值
num_v_acmd_coeff = double(coeffs(num_v_acmd, z, 'All'));
den_v_acmd_coeff = double(coeffs(den_v_acmd, z, 'All'));
num_v_acmd_coeff = num_v_acmd_coeff/den_v_acmd_coeff(1);
den_v_acmd_coeff = den_v_acmd_coeff/den_v_acmd_coeff(1);

num_v_amd_coeff = double(coeffs(num_v_amd, z, 'All'));
den_v_amd_coeff = double(coeffs(den_v_amd, z, 'All'));
num_v_amd_coeff = num_v_amd_coeff/den_v_amd_coeff(1);
den_v_amd_coeff = den_v_amd_coeff/den_v_amd_coeff(1);

% 显示结果
disp('传递函数 a cmd->v 的分子系数:');
disp(num_v_acmd_coeff);
disp('传递函数 a cmd->v 的分母系数:');
disp(den_v_acmd_coeff);

disp('传递函数 a md->v 的分子系数:');
disp(num_v_amd_coeff);
disp('传递函数 a md->v 的分母系数:');
disp(den_v_amd_coeff);

%% acmd->a 和cmd->a 脉冲传递函数
% 获取acmd->a 和cmd->a系数并转换为双精度数值
num_a_acmd_coeff = double(coeffs(num_a_acmd, z, 'All'));
den_a_acmd_coeff = double(coeffs(den_a_acmd, z, 'All'));
num_a_acmd_coeff = num_a_acmd_coeff/den_a_acmd_coeff(1);
den_a_acmd_coeff = den_a_acmd_coeff/den_a_acmd_coeff(1);

num_a_amd_coeff = double(coeffs(num_a_amd, z, 'All'));
den_a_amd_coeff = double(coeffs(den_a_amd, z, 'All'));
num_a_amd_coeff = num_a_amd_coeff/den_a_amd_coeff(1);
den_a_amd_coeff = den_a_amd_coeff/den_a_amd_coeff(1);

lpf_acc_raw_discrete = tf(b_acc_raw,a_acc_raw,0.01)
washout_acc_lpf_discrete = tf([0.9969 -0.9969],[1 -0.9937],0.01)
lpf_acc_raw_continous = d2c(lpf_acc_raw_discrete)
washout_acc_lpf_continous = d2c(washout_acc_lpf_discrete)

h_a_acmd = d2c(tf(num_a_acmd_coeff,den_a_acmd_coeff,0.01))
h_a_amd = d2c(tf(num_a_amd_coeff,den_a_amd_coeff,0.01))

% 显示结果
disp('传递函数 a cmd->a 的分子系数:');
disp(num_a_acmd_coeff);
disp('传递函数 a cmd->a 的分母系数:');
disp(den_a_acmd_coeff);

disp('传递函数 a md->a 的分子系数:');
disp(num_a_amd_coeff);
disp('传递函数 a md->a 的分母系数:');
disp(den_a_amd_coeff);

%% acmd->da 和cmd->da 脉冲传递函数
% 获取acmd->h 和cmd->h系数并转换为双精度数值
num_da_acmd_coeff = double(coeffs(num_da_acmd, z, 'All'));
den_da_acmd_coeff = double(coeffs(den_da_acmd, z, 'All'));
% num_da_acmd_coeff = num_da_acmd_coeff/den_da_acmd_coeff(1);
% den_da_acmd_coeff = den_da_acmd_coeff/den_da_acmd_coeff(1);

num_da_amd_coeff = double(coeffs(num_da_amd, z, 'All'));
den_da_amd_coeff = double(coeffs(den_da_amd, z, 'All'));
% num_da_amd_coeff = num_da_amd_coeff/den_da_amd_coeff(1);
% den_da_amd_coeff = den_da_amd_coeff/den_da_amd_coeff(1);

% 显示结果
disp('传递函数 a cmd->da 的分子系数:');
disp(num_da_acmd_coeff);
disp('传递函数 a cmd->da 的分母系数:');
disp(den_da_acmd_coeff);

disp('传递函数 a md->da 的分子系数:');
disp(num_da_amd_coeff);
disp('传递函数 a md->h 的分母系数:');
disp(den_da_amd_coeff);

%% 计算 加速度指令 和 加速度测量（低通滤波+洗出）->速度估计的频率响应
% 加速度指令->速度估计的频率响应
[H_v_cmd, freq_v_cmd] = freqz(num_v_acmd_coeff,den_v_acmd_coeff,1024*4,fs_acc_use);
phase_v_cmd = unwrap(angle(H_v_cmd)) * (180/pi);
% 加速度测量（低通滤波+洗出）->速度估计的频率响应
[H_v_md, freq_v_md] = freqz(num_v_amd_coeff,den_v_amd_coeff,1024*4,fs_acc_use);
phase_v_md = unwrap(angle(H_v_md)) * (180/pi);
%% 计算 加速度指令 和 加速度测量（低通滤波+洗出）->加速度估计的频率响应
% 加速度指令->加速度估计的频率响应
[H_acc_cmd, freq_acc_cmd] = freqz(num_a_acmd_coeff,den_a_acmd_coeff,1024*4,fs_acc_use);
phase_acc_cmd = unwrap(angle(H_acc_cmd)) * (180/pi);
% 加速度测量（低通滤波+洗出）->加速度估计的频率响应
[H_acc_md, freq_acc_md] = freqz(num_a_amd_coeff,den_a_amd_coeff,1024*4,fs_acc_use);
phase_acc_md = unwrap(angle(H_acc_md)) * (180/pi);
%% 计算 加速度指令 和 加速度测量（低通滤波+洗出）->加速度漂移估计的频率响应
% 加速度指令->加速度漂移估计的频率响应
[H_dacc_cmd, freq_dacc_cmd] = freqz(num_da_acmd_coeff,den_da_acmd_coeff,1024*4,fs_acc_use);
phase_dacc_cmd = unwrap(angle(H_dacc_cmd)) * (180/pi);
% 加速度测量（低通滤波+洗出）->加速度漂移估计的频率响应
[H_dacc_md, freq_dacc_md] = freqz(num_da_amd_coeff,den_da_amd_coeff,1024*4,fs_acc_use);
phase_dacc_md = unwrap(angle(H_dacc_md)) * (180/pi);
%% 不用 绘图 加速度指令频率响应
% figure
% subplot(2,1,1)
% hold on
% plot(freq_acc_cmd,20*log10(abs(H_acc_cmd)),'b-','linewidth',2);
% plot(freq_acc_cmd,-3*ones(size(H_acc_cmd)),'k-','linewidth',2);
% grid on
% title('加速度指令频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% xlim([0 20])
% 
% subplot(2,1,2)
% hold on
% plot(freq_acc_cmd, phase_acc_cmd);
% grid on;
% xlim([0 20])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off
%% 不用 绘图 加速度测量（低通滤波+洗出）频率响应
% figure
% subplot(2,1,1)
% hold on
% plot(freq_acc_cmd,20*log10(abs(H_acc_cmd)),'r-','linewidth',2);
% plot(freq_acc_md,20*log10(abs(H_acc_md)),'b-','linewidth',2);
% plot(freq_acc_md,-3*ones(size(H_acc_md)),'k-','linewidth',2);
% grid on
% title('加速度测量（低通滤波+洗出）频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% % legend('')
% xlim([0 20])
% 
% subplot(2,1,2)
% hold on
% plot(freq_acc_cmd, phase_acc_cmd,'r-','linewidth',2);
% plot(freq_acc_md, phase_acc_md,'b-','linewidth',2);
% grid on;
% xlim([0 20])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% grid on
% hold off

%% 绘图 加速度指令和测量（低通滤波+洗出）->速度估计的频率响应
figure
subplot(2,1,1)
hold on
plot(freq_v_cmd,20*log10(abs(H_v_cmd)),'r-','linewidth',2);
plot(freq_v_cmd,20*log10(abs(H_v_md)),'b-','linewidth',2);
plot(freq_v_cmd,-3*ones(size(H_v_cmd)),'k-','linewidth',2);
grid on
title('加速度指令和测量（低通滤波+洗出）->速度估计的频响')
xlabel('频率/Hz');
ylabel('幅度/dB');
legend('加速度指令->速度估计','加速度测量->速度估计','-3dB')
xlim([0 20])

subplot(2,1,2)
hold on
plot(freq_v_cmd, phase_v_cmd,'r-','linewidth',2);
plot(freq_v_md, phase_v_md,'b-','linewidth',2);
grid on;
xlim([0 20])
xlabel('frequency(Hz)');
ylabel('phase/°');
legend('加速度指令->速度估计','加速度测量->速度估计')
grid on
hold off

%% 绘图 加速度指令和测量（低通滤波+洗出）->加速度估计的频率响应
figure
subplot(2,1,1)
hold on
plot(freq_acc_cmd,20*log10(abs(H_acc_cmd)),'r-','linewidth',2);
plot(freq_acc_md,20*log10(abs(H_acc_md)),'b-','linewidth',2);
plot(freq_acc_md,-3*ones(size(H_acc_md)),'k-','linewidth',2);
grid on
title('加速度指令和测量（低通滤波+洗出）->加速度估计的频响')
xlabel('频率/Hz');
ylabel('幅度/dB');
legend('加速度指令->加速度估计','加速度测量->加速度估计','-3dB')
xlim([0 20])

subplot(2,1,2)
hold on
plot(freq_acc_cmd, phase_acc_cmd,'r-','linewidth',2);
plot(freq_acc_md, phase_acc_md,'b-','linewidth',2);
grid on;
xlim([0 20])
xlabel('frequency(Hz)');
ylabel('phase/°');
legend('加速度指令->加速度估计','加速度测量->加速度估计')
grid on
hold off

%% 绘图 bode图
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 100];
opt.Title.String = 'kalman滤波器的伯德图';
hold on
bode(h_a_acmd, opt, 'r-.');
hold on
bode(h_a_amd, opt, 'b-.');
legend('加速度指令对应环节','加速度测量（低通+洗出）对应环节');

% lpf_acc_raw_continous 加速度测量：低通滤波
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 100];
opt.Title.String = '加速度测量低通滤波的伯德图';
hold on
bode(lpf_acc_raw_continous, opt, 'r-.');
legend('低通滤波器');

% washout_acc_lpf_continous 加速度测量：washout
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 100];
opt.Title.String = '加速度测量洗出环节的伯德图';
hold on
bode(washout_acc_lpf_continous, opt, 'b-.');
legend('洗出环节');

% G1 加速度测量：lpf+washout
G1 = lpf_acc_raw_continous*washout_acc_lpf_continous;
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 100];
opt.Title.String = '低通+洗出环节的伯德图';
hold on
bode(G1, opt, 'b-.');
legend('低通+洗出环节');

% G2 加速度测量：lpf+washout+kalman
G2 = G1*h_a_amd
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 100];
opt.Title.String = '加速度测量低通+洗出+kalman的伯德图';
hold on
bode(G2, opt, 'b-.');
legend('低通+洗出+kalman');

%% 绘图 加速度指令和测量（低通滤波+洗出）->加速度漂移估计的频率响应
% figure
% subplot(2,1,1)
% hold on
% plot(freq_dacc_cmd,20*log10(abs(H_dacc_cmd)),'r-','linewidth',2);
% plot(freq_dacc_md,20*log10(abs(H_dacc_md)),'b-','linewidth',2);
% plot(freq_dacc_md,-3*ones(size(H_dacc_cmd)),'k-','linewidth',2);
% grid on
% title('加速度指令和测量（低通滤波+洗出）->加速度漂移估计的频响')
% xlabel('频率/Hz');
% ylabel('幅度/dB');
% legend('加速度指令->加速度漂移估计','加速度测量->加速度漂移估计','-3dB')
% xlim([0 20])
% 
% subplot(2,1,2)
% hold on
% plot(freq_dacc_cmd, phase_dacc_cmd,'r-','linewidth',2);
% plot(freq_dacc_md, phase_dacc_md,'b-','linewidth',2);
% grid on;
% xlim([0 20])
% xlabel('frequency(Hz)');
% ylabel('phase/°');
% legend('加速度指令->加速度漂移估计','加速度测量->加速度漂移估计')
% grid on
% hold off
%% 融合结果
% figure;
% % subplot(2,1,1)
% hold on
% plot(t_rtk,h_rtk,'r-','linewidth',2);
% plot(t_acc_kalman,state_height,'b-','linewidth',2);
% 
% hold on
% % plot(t_acc_cmd,cmd_acc_input*10,'mo','linewidth',2);
% % plot(t_acc_cmd,v_acc_cmd*10,'b--','linewidth',2);
% % plot(t_acc_raw,acc_z_lpf_ef*10,'-','Color','r','linewidth',2);
% % plot(t_acc_raw,acc_z_md_ef*10,'b-','linewidth',2);
% % plot(t_acc_kalman,pred_state(3,:)*10,'-','Color','g','linewidth',2);
% % plot(t_acc_kalman,state_acc_true*10,'-','Color','k','linewidth',2);
% % plot(t_acc_cmd,cmd_vel_guided*10,'g-','linewidth',2);
% hold off
% grid on
% axis tight
% legend('RTK高度','融合高度',...
%     '加速度指令','速度指令','加速度测量(lpf-ef)','加速度测量洗出(ef)','融合加速度')
% 
% % legend('RTK高度','GPS洗出速度积分','GPS速度积分','融合高度','融合加速度二次积分','加速度指令的二次积分',...
% %     '加速度指令','速度指令','加速度测量(lpf-ef)','加速度测量洗出(ef)','融合加速度')
% % legend('RTK高度','GPS洗出速度积分','GPS速度积分','融合加速度二次积分',...
% %     '加速度指令','加速度测量(lpf-ef)','加速度测量(mlpf-ef)','融合加速度','速度指令')
% xlabel('time/s');
% title('高度和加速度')
% 
% % subplot(2,1,2)
% % hold on
% % plot(t_rtk,h_rtk,'r-','linewidth',2);
% % plot(t_baro,h_baro,'m-','linewidth',2);
% % plot(t_gps,h_gps_vel_filter_integ,'bo','linewidth',2);
% % plot(t_gps,h_gps_vel_integ,'b+','linewidth',2);
% % % plot(t_acc_kalman,h_v_est,'o','Color','g','linewidth',2);
% % plot(t_acc_kalman,state_height,'k--','linewidth',2);
% % plot(t_acc_kalman,h_v_est,'-','Color','g','linewidth',2);
% % 
% % hold on
% % plot(t_gps,vel_d_gps*10,'-','Color','r','linewidth',2);
% % plot(t_gps,vel_d_gps_filter*10,'-','Color','b','linewidth',2);
% % % plot(t_acc_kalman,pred_state(2,:)*10,'o','Color','g','linewidth',2);
% % plot(t_acc_cmd,cmd_vel_guided*10,'g-','linewidth',2);
% % plot(t_acc_kalman,state_vel*10,'-','Color','k','linewidth',2);
% % xlabel('time/s');
% % ylabel('vel/(dm/s)');
% % title('高度和速度')
% % legend('RTK高度','气压高度','GPS洗出速度积分','GPS速度积分','融合高度','融合速度积分',...
% %     '原始GPS速度','GPS速度洗出','速度指令','融合速度')
% % % legend('RTK高度','GPS洗出速度积分','GPS速度积分','融合高度','融合速度积分',...
% % %     '原始GPS速度','GPS速度洗出','一步预测速度','融合速度')
% % grid on
% % axis tight
%% 加速度
% figure;
% % subplot(2,1,1)
% hold on
% plot(t_rtk,h_rtk,'r-','linewidth',2);
% plot(t_acc_kalman,state_height,'b-','linewidth',2);
% plot(t_acc_kalman,h_acc,'k-','linewidth',2);
% 
% hold on
% % plot(t_acc_cmd,cmd_acc_input*10,'mo','linewidth',2);
% % plot(t_acc_cmd,v_acc_cmd*10,'b--','linewidth',2);
% % plot(t_acc_raw,acc_z_lpf_ef*10,'-','Color','r','linewidth',2);
% % plot(t_acc_raw,acc_z_md_ef*10,'b-','linewidth',2);
% % plot(t_acc_kalman,pred_state(3,:)*10,'-','Color','g','linewidth',2);
% plot(t_acc_kalman,state_acc_true*10,'-','Color','k','linewidth',2);
% plot(t_acc_kalman,state_vel*10,'-','Color','r','linewidth',2);
% plot(t_acc_kalman,v_acc*10,'-','Color','b','linewidth',2);
% % plot(t_acc_cmd,cmd_vel_guided*10,'g-','linewidth',2);
% hold off
% grid on
% axis tight
% legend('RTK高度','融合高度',...
%     '加速度指令','速度指令','加速度测量(lpf-ef)','加速度测量洗出(ef)','融合加速度')
%% 分析加速度的模型
motor_bw = 0.8;
motor_tf = tf(1,[1/(2*pi*motor_bw) 1]);
motor_thrust_amp = 52*9.806/775;

weight = 80;
accel_amp_thrust = motor_thrust_amp * 4 /weight;

ctrl_gain = 100;

accel_ctrl_tf = tf(pid(.5, 1.2, 0)) * ctrl_gain;
accel_open_tf = accel_ctrl_tf * motor_tf * accel_amp_thrust;

% 加速度开环
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 10000];
opt.Title.String = '加速度模型与加速度指令滤波的伯德图';

% 加速度模型+加速度指令滤波
hold on;
bode(accel_open_tf, opt, 'r-.');
bode(h_a_acmd, opt, 'b-.');
bode(accel_open_tf*h_a_acmd, opt, 'k-.');
legend('加速度模型开环','加速度指令滤波','加速度模型+加速度指令滤波')
hold on;

% 加速度模型与加速度测量值滤波的伯德图
figure;
opt=bodeoptions;
opt.FreqUnits = 'Hz';
opt.Grid = 'on';
opt.Xlim = [0.01 10000];
opt.Title.String = '加速度模型与加速度测量值滤波的伯德图';

% hold on;
% bode(accel_open_tf, opt, 'r-.');
% bode(h_a_amd, opt, 'b-.');
% bode(accel_open_tf*h_a_amd, opt, 'k-.');
% legend('加速度模型开环','加速度测量值滤波','加速度模型+加速度测量值滤波')
% hold on;

% figure;
% opt=bodeoptions;
% opt.FreqUnits = 'Hz';
% opt.Grid = 'on';
% opt.Xlim = [0.01 10000];
% opt.Title.String = '加速度模型与加速度测量值总滤波的伯德图';

hold on;
bode(accel_open_tf, opt, 'r-.');
bode(G2, opt, 'b-.');
bode(accel_open_tf*G2, opt, 'k-.');
legend('加速度模型开环','加速度测量值总滤波','加速度模型+加速度测量值总滤波')
hold on;

%%
state_vel=state_vel';
state_acc_true = state_acc_true';
state_delta_accel = state_delta_accel';
t_acc_kalman = t_acc_kalman';
acc_z_md_ef=acc_z_md_ef';
acc_z_lpf_ef= acc_z_lpf_ef';
innov_a_store = innov_a_store';
cmd_acc_input = cmd_acc_input';
%% get rid of the function paths
rmpath('..\..\MyFunc_public');
rmpath('..\..\MyFunc_estimate');
            
%%

