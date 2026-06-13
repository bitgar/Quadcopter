close all
clear all
clc

%% 配平计算
z0 = [0;0;500;500;500;500];   % 初始猜测
opts = optimoptions('fsolve','Display','iter');
[z_opt,~,exitflag] = fsolve(@trimFun,z0,opts);

phi_opt  = z_opt(1)
theta_opt= z_opt(2)
n_opt    = z_opt(3:6)

%% 输出打印结果
fprintf('配平结果:\n');
fprintf('  phi   = %.6f  rad  = %.3f  deg\n', phi_opt, rad2deg(phi_opt));
fprintf('  theta = %.6f  rad  = %.3f  deg\n', theta_opt, rad2deg(theta_opt));
fprintf('  n1=n2 = %.2f  rad/s  = %.0f  rpm\n', n_opt(1), n_opt(1)*60/(2*pi));
fprintf('  n3=n4 = %.2f  rad/s  = %.0f  rpm\n', n_opt(3), n_opt(3)*60/(2*pi));

%% 迭代计算函数
function F = trimFun(z)
% z = [phi; theta; n1; n2; n3; n4]
phi  = z(1);
theta= z(2);
n1   = z(3); n2 = z(4); n3 = z(5); n4 = z(6);

kf = 0.00043; km = 0.00078;
x1 = 21; x2 = 29; y1 = 55; y2 = 45; g = 9.81;

F = zeros(8,1);
F(1) = -g*sin(theta);
F(2) =  g*cos(theta)*sin(phi);
F(3) = -kf*(n1^2+n2^2+n3^2+n4^2) + g*cos(theta)*cos(phi);
F(4) =  kf*y1*(n1^2-n2^2) - kf*y2*(n3^2-n4^2);
F(5) =  kf*x1*(n1^2+n2^2) - kf*x2*(n3^2+n4^2);
F(6) =  km*(n1^2-n2^2+n3^2-n4^2);
F(7) =  n1 - n2;
F(8) =  n3 - n4;
end
%%
