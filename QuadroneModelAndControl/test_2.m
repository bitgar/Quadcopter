close all
clear all
clc

s = tf('s');
G = 2/s;
Kp = 1; Ki = 1;
C = Kp + Ki/s;
L = C*G;
T = feedback(L,1);

% 画图与计算关键量
figure; bode(L); title('Open-loop L(s)');
figure; bode(T); title('Closed-loop T(s)');
[gm,pm,wgc] = margin(L);
w_bw = bandwidth(T);
stepinfo_T = stepinfo(T);

disp(['w_gc = ', num2str(wgc), ' rad/s']);
disp(['bandwidth = ', num2str(w_bw), ' rad/s']);
disp(stepinfo_T);

