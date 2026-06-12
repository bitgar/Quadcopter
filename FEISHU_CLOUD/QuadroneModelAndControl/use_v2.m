close all
clc

%%
fs = 10000; %对应采样时间0.0001s。
N = length(In);
n = 0: (N - 1);
fn  = n * fs / N;

%% 统一输入/输出信号的接口
data_input = In;
data_input = data_input(:);

data_output = Out;
data_output = data_output(:);

%% 扫频信号的频谱—幅度谱和相位谱
data_input_fft = fft(data_input);
data_input_fft_abs = abs(data_input_fft);

data_input_spectrum = data_input_fft_abs(1:N/2+1)/N;
data_input_spectrum(2:end-1) = 2*data_input_spectrum(2:end-1);

mag_data_input_spectrum = data_input_spectrum;
phase_data_input_spectrum = angle(data_input_fft(1:N/2+1)/N);
phase_data_input_spectrum = phase_data_input_spectrum/pi*180;

%% 响应信号的频谱—幅度谱和相位谱
data_output_fft = fft(data_output);
data_output_fft_abs = abs(data_output_fft);

data_output_spectrum = data_output_fft_abs(1:N/2+1)/N;
data_output_spectrum(2:end-1) = 2*data_output_spectrum(2:end-1);

mag_data_output_spectrum = data_output_spectrum;
phase_data_output_spectrum = angle(data_output_fft(1:N/2+1)/N);
phase_data_output_spectrum = phase_data_output_spectrum/pi*180;

%% 计算输出与输入信号间的幅频比和相位差
Amp = [];
Phi = [];
w = [];
Amp_log = [];
for i=1:N/2+1
    Amp(i) = data_output_spectrum(i)/data_input_spectrum(i);
    Phi(i) = phase_data_output_spectrum(i) - phase_data_input_spectrum(i);
    Amp_log(i) = 20 * log10(Amp(i));
    
    if(Phi(i)>0)
        Phi(i) = Phi(i) - 360;
    end
    
end

f_single_sided = (0:N/2)*fs/N;
f_single_sided = f_single_sided(:);
w = (2*pi) .* f_single_sided;

%% 画Bode图
figure
subplot(211);semilogx((w),(Amp_log));grid on;
subplot(212);semilogx((w),(Phi));grid on;

% figure
% subplot(211);semilogx((w),20*log10(data_input_spectrum));grid on;
% subplot(212);semilogx((w),(phase_data_input_spectrum));grid on;
% 
% figure
% subplot(211);semilogx((w),20*log10(data_output_spectrum));grid on;
% subplot(212);semilogx((w),(phase_data_output_spectrum));grid on;
