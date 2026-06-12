clear all
close all
clc

%%
syms z;

A = [1 0 0;...
     0 1 0;...
     0 0 1]
B = [1;...
     0;...
     0]
C = [1 1 1]
I3 = eye(size(A))
zi_minus_a = z*I3 - A;
zi_minus_a = vpa(zi_minus_a,4)

zi_minus_a_inv = inv(zi_minus_a);
zi_minus_a_inv = vpa(zi_minus_a_inv,4)

H_z = C * zi_minus_a_inv * B;
H_z = vpa(H_z,4);
disp('传递函数 H(z):');
disp(H_z);

% 化简传递函数
H_z_simplified = simplify(H_z);
H_z_simplified = vpa(H_z_simplified,4);
disp('化简后的传递函数 H(z):');
disp(H_z_simplified);

% 提取 h1 和 h2
h_acmd = H_z_simplified(1,1)

% 获取 acmd->x_hat 和 amd->x_hat 的分子和分母
[num_h_acmd, den_h_acmd] = numden(h_acmd)


%%


%%
