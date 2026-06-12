close all
clear all
clc

%% 飞机参数
mass = 40.37;
g = 9.81;
x1 = 0.52;
x2 = 0.7107;
y1 = 1.375;
y2 = 1.126;
zr = -0.1;
R = 0.28;
D = 2*R;

Ix = 13.59;
Iy = 9.052;
Iz = 21.238;
Ixy = 0.508;
Ixz = 0.348;
Iyz = -0.118;
Ix_1 = (Ix*Iz-Ixz*Ixz)/Iz
Iz_1 = (Ix*Iz-Ixz*Ixz)/Ix
Ixz_1 = Ixz/(Ix*Iz-Ixz*Ixz)

fb = 4.0648;
fb0 = 0.309;

CT = 0.1452;
CQ = 0.0115;
CDr = 0.01;
Cfa = 0.37;

rho = 1.225;
v0 = 30;

kf = rho*D^4*CT
km = rho*D^5*CQ
kd = rho*D^4*CDr

epsilon = atan(kd/kf);
% epsilon = atan(CDr/CT)*57.3

% rho = 0.06167*2*mass/fb;
% 40.37 9.81 0.52 0.7107 1.375 1.126 0.0184 8.1701e-04 13.59 9.052 21.238 0.348
%% 配平计算
z0 = [50;50;50;50;0];   % 初始猜测
opts = optimoptions('fsolve','Display','iter');
[z_opt,~,exitflag] = fsolve(@trimFun,z0,opts,mass,g,zr,kf,km,kd,rho,v0,fb0,x1,x2,y1,y2);
% function F = trimFun(z,mass,g,zr,kf,km,kd,rho,v0,fb0,x1,x2,y1,y2)

n_opt    = z_opt(1:4)
alpha0_opt  = z_opt(5)

%% 输出打印结果
disp('-------------------------------------------------------')
fprintf('配平结果:\n');
fprintf('  alpha0 = theta0 = %.6f  rad  = %.3f  deg\n', alpha0_opt, rad2deg(alpha0_opt));
fprintf('  n1=n2 = %.2f  r/s  = %.0f  rpm\n', n_opt(1), n_opt(1)*60);
fprintf('  n3=n4 = %.2f  r/s  = %.0f  rpm\n', n_opt(3), n_opt(3)*60);

disp('-------------------------------------------------------')
fprintf('  阻力角度ε = epsilon = %.6f  rad  = %.3f  deg\n', epsilon, rad2deg(epsilon));
disp('-------------------------------------------------------')

%% 迭代计算函数
function F = trimFun(z,mass,g,zr,kf,km,kd,rho,v0,fb0,x1,x2,y1,y2)
% z = [n10; n20; n30; n40 ; alpha0]
n10 = z(1); 
n20 = z(2); 
n30 = z(3); 
n40 = z(4);
alpha0  = z(5);

F = zeros(7,1);
F(1) = -1/mass*(kf*sin(alpha0) + kd*cos(alpha0))*(n10^2 + n20^2 + n30^2 + n40^2) - 0.5/mass*rho*fb0*v0^2;
F(2) = 1/mass*(-kf*cos(alpha0) + kd*sin(alpha0))*(n10^2 + n20^2 + n30^2 + n40^2) + g;
F(3) =  kf*y1*(n10^2-n20^2) + kf*y2*(n30^2-n40^2);
F(4) =  (-kd*zr+kf*x1)*(n10^2+n20^2) + (-kd*zr-kf*x2)*(n30^2+n40^2);
F(5) =  (km-kd*y1)*(n10^2-n20^2) - (km+kd*y2)*(n30^2-n40^2);
F(6) =  n10 - n20;
F(7) =  n30 - n40;

% F(2) = 1/mass*(kf*sin(alpha0)*sin(beta0) + kd*cos(alpha0)*sin(beta0))*(n10^2 + n20^2 + n30^2 + n40^2) + g*cos(gamma)*sin(mu0);
% F(2) = 0;

end
%%
