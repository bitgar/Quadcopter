close all
clc

%%
fs = 10000; %对应采样时间0.0001s。
N = length(In);
n = 0: (N - 1);
fn  = n * fs / N;

%% 
y = fft(In,N);
mag  = abs(y);
pha = angle(y);
%% 
y1 = fft(Out,N);
magl  = abs(y1);
phal = angle(y1);
%% 
%%计算幅值比，相位差
for i = 1 : N / 2
    mag_el(i) = magl(i)/mag(i);
    w(i) = fn(i) * 2 * pi;
    ph_el(i) = (phal(i) - pha(i)) * 180 / pi;
    mag_log(i) = 20 * log10(mag_el(i));
end
%%对相位差的数据进行预处理
for i = 1: length(ph_el)
    if ph_el(i) > 0
        ph_el(i) =  ph_el(i) - 360;
    end
end
%% 画Bode图
subplot(211);semilogx((w),(mag_log));grid on;
subplot(212);semilogx((w),(ph_el));grid on;
