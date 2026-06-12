close all
clear all
clc

K = 1;              % ÔöÒæ
s = tf('s');
G = K/s;
bode(G); grid on;
figure;
pzmap(G); grid on;




