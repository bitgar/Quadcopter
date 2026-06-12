close all
clear all
clc

% Root locus of G(s) = K*(s + 1)/s^2
num = [1 1];      % 对应 (s + 1)
den = [1 0 0];    % 对应 s^2
figure;
rlocus(tf(num,den));
grid on;
title('Root Locus of (s+1)/s^2');
