clear all
close all
clc

%%
syms s f m g Jx Jy Jz;

% roll phi
% pitch theta
% yaw psi
syms phi theta psi; 

% [x_dot y_dot z_dot p q r roll pitch yaw]
% [x_dot y_dot z_dot w_roll w_pitch w_yaw roll pitch yaw]
A = [0 0 0 0 0 0 -f/m*sin(psi) -f/m*cos(psi) 0;...
     0 0 0 0 0 0 f/m*cos(psi) -f/m*sin(psi) 0;...
     0 0 0 0 0 0 0 0 0;...
     
     0 0 0 0 0 0 0 0 0;...
     0 0 0 0 0 0 0 0 0;...
     0 0 0 0 0 0 0 0 0;...
     
     0 0 0 1 0 0 0 0 0;...
     0 0 0 0 1 0 0 0 0;...
     0 0 0 0 0 1 0 0 0]
 
B = [0 0 0 0;...
     0 0 0 0;...
     -1/m 0 0 0;...
     
     0 1/Jx 0 0;...
     0 0 1/Jy 0;...
     0 0 0 1/Jz;...
     
     0 0 0 0;...
     0 0 0 0;...
     0 0 0 0]
 
C = [0 0 0 0 0 0 1 0 0;...
     0 0 0 0 0 0 0 1 0;...
     0 0 0 0 0 0 0 0 1]
 
I = eye(size(A))
sI_minus_A = s*I - A;
sI_minus_A = vpa(sI_minus_A,4)

sI_minus_A_inv = inv(sI_minus_A);
sI_minus_A_inv = vpa(sI_minus_A_inv,4)

% H_s = C * sI_minus_A_inv * B;
H_s = sI_minus_A_inv * B;
H_s = vpa(H_s,4);
disp('传递函数 H(s):');
disp(H_s);

% 化简传递函数
H_s_simplified = simplify(H_s);
H_s_simplified = vpa(H_s_simplified,4);
disp('化简后的传递函数 H(z):');
disp(H_s_simplified);

% 提取传递函数
% h_1 = H_s_simplified(1,1)

% 提取传递函数的分子和分母
% [num_1, den_1] = numden(h_1)


%%


%%
