close all
clear all
clc

%% parameters provided by the book
mass = 40.37;
g=9.81;
x1 = 0.52;
x2 = 0.7107;
y1 = 1.375;
y2 = 1.126;
zr = -0.1;
R = 0.28;
D = 2*R;
CT = 0.1452;
CQ = 0.0115;
CDr = 0.01;

Ix = 13.59;
Iy = 9.052;
Iz = 21.238;

Ixy = 0.508;
Ixz = 0.348;
Iyz = -0.118;

Ix_1 = (Ix*Iz-Ixz*Ixz)/Iz
Iz_1 = (Ix*Iz-Ixz*Ixz)/Ix
Ixz_1 = Ixz/(Ix*Iz-Ixz*Ixz)

n10 = 81.3391;
n20 = 81.3391;
n30 = 69.2165;
n40 = 69.2165;

%% calculate the detailed parameters
rho = 1.29
kf = rho*D^4*CT
km = rho*D^5*CQ
disp('-------------------------------------------------------')

%%
A_lon = [0      0       0      -g;...
         0      0       0       0;...
         0      0       0       0;...
         0      0       1       0]
     
B_lon = [0                     0                      0                      0;...
         -(2*kf*n10)/mass      -(2*kf*n20)/mass       -(2*kf*n30)/mass       -(2*kf*n40)/mass;...
         (2*kf*x1*n10)/Iy      (2*kf*x1*n20)/Iy       -(2*kf*x2*n30)/Iy      -(2*kf*x2*n40)/Iy;...
         0                     0                      0                      0]
     
bp1 = (2*kf*y1*Iz)/(Ix*Iz-Ixz*Ixz) + (2*km*Ixz)/(Ix*Iz-Ixz*Ixz);
bp2 = (2*kf*y2*Iz)/(Ix*Iz-Ixz*Ixz) - (2*km*Ixz)/(Ix*Iz-Ixz*Ixz);
br1 = (2*kf*y1*Ixz)/(Ix*Iz-Ixz*Ixz) + (2*km*Ix)/(Ix*Iz-Ixz*Ixz);
br2 = (2*kf*y2*Ixz)/(Ix*Iz-Ixz*Ixz) - (2*km*Ix)/(Ix*Iz-Ixz*Ixz);

A_lat = [0             0             0              g;...
         0             0             0              0;...
         0             0             0              0;...
         0             1             0              0]
     
B_lat = [0             0             0              0;...
         bp1*n10      -bp1*n20       bp2*n30       -bp2*n40;...
         br1*n10      -br1*n20       br2*n30       -br2*n40;...
         0             0             0              0]
disp('-------------------------------------------------------')

%% dq、dtheta、du传递函数
% 控制分配:dn_e = dn1 - sqrt(x2/x1)*dn3
% dn_e>=0 dn1=dn2
% dn_e<0 dn3=dn4

% dn_e = dn1 - sqrt(x2/x1)*dn3
disp('dn_e = dn1 - sqrt(x2/x1)*dn3中的sqrt(x2/x1)为')
disp(sqrt(x2/x1))

% dq/dn_e = (4*kf*x1*n10/Iy)/s
disp('dq/dn_e = (4*kf*x1*n10/Iy)/s中的系数为')
disp(4*kf*x1*n10/Iy)

% dtheta/dn_e = (4*kf*x1*n10/Iy)/s^2
disp('dtheta/dn_e = (4*kf*x1*n10/Iy)/s^2中的系数为')
disp(4*kf*x1*n10/Iy)

% du传递函数
% du/dn_e = (4*kf*g*x1*n10/Iy)/s^3
disp('du/dn_e = (4*kf*g*x1*n10/Iy)/s^3中的系数为')
disp(-4*kf*g*x1*n10/Iy)
disp('-------------------------------------------------------')

%% dw传递函数
% 控制分配 dn_h = dn1
% dn1 = dn2 ; dn3 = dn4 = sqrt(x1/x2)
disp('控制分配系数为')
disp(sqrt(x1/x2))

% dw/dn1 = -(4*kf*n10/mass)*((x1+x2)/x2)/s
disp('dw/dn1 = -(4*kf*n10/mass)*[(x1+x2)/x2]/s中的系数为')
disp(-(4*kf*n10/mass)*((x1+x2)/x2))
disp('-------------------------------------------------------')

%% dp、dphi、dv传递函数
% 控制分配：dn_a = dn1 - dn2
% dn_a>=0,dn3=sqrt(x1/x2)*dn1
% dn_a<0,dn4=sqrt(x1/x2)*dn2

% dp/dna = n10*(bp1*x2+bp2*x1)/x2/s
disp('dp/dna = n10*(bp1*x2+bp2*x1)/x2/s中的系数为')
disp(n10*(bp1*x2+bp2*x1)/x2)

% dphi/dna = n10*(bp1*x2+bp2*x1)/x2/s^2
disp('dphi/dna = n10*(bp1*x2+bp2*x1)/x2/s^2中的系数为')
disp(n10*(bp1*x2+bp2*x1)/x2)

% dv/dna = g*n10*(bp1*x2+bp2*x1)/x2/s^2
disp('dv/dna = g*n10*(bp1*x2+bp2*x1)/x2/s^2中的系数为')
disp(g*n10*(bp1*x2+bp2*x1)/x2)

disp('-------------------------------------------------------')

%% dr、dpsi传递函数
% 控制分配：dn_r = dn1 - dn2
% dn_r>=0 dn4=sqrt(x1/x2)*dn1
% dn_r<0 dn3=sqrt(x1/x2)*dn2

% dr/dnr = n10*(br1*x2-br2*x1)/x2/s
disp('dr/dnr = n10*(br1*x2-br2*x1)/x2/s中的系数为')
disp(n10*(br1*x2-br2*x1)/x2)

% dpsi/dnr = n10*(br1*x2-br2*x1)/x2/s^2
disp('dpsi/dnr = n10*(br1*x2-br2*x1)/x2/s^2中的系数为')
disp(n10*(br1*x2-br2*x1)/x2)

disp('-------------------------------------------------------')

%% 自己硬着头皮算的参数
% % 纵向通道
% [0 0 0 -9.8;...
%  0 0 0 0;...
%  0 0 0 0;...
%  0 0 1 0]
% 
% [0 0 0 0;...
% -0.07030 -0.07030 -0.06012 -0.06012;...
%  0.16303 0.16303 -0.19054 -0.19054;...
%  0 0 0 0]
% 
% % 计算飞机的质量
% kf = 0.00043 
% km = 0.00078
% n10 = 81.3391
% n20 = 81.3391
% n30 = 69.2165
% n40 = 69.2165
% x1 = 21
% x2 = 29
% y1 = 55
% y2 = 45
% 
% % x = -(2*kf*n10)/mass
% % mass =1.0
% x = -0.07030
% mass = -x/(2*kf*n10)
% mass = -x/(2*kf*n20)
% 
% x = -0.06012
% mass = -x/(2*kf*n30) 
% mass = -x/(2*kf*n40) 
% 
% % 计算飞机的转动惯量Ix
% % 计算飞机的转动惯量Iy
% % x = (2*kf*x1*n10)/Iy
% % Iy = 0.11
% x = 0.16303
% Iy = x/(2*kf*x1*n10)
% Iy = x/(2*kf*x1*n20)
% x = -0.19054
% Iy = -x/(2*kf*x2*n30)
% Iy = -x/(2*kf*x2*n40)
% % 0.3261 = (4*kf*x1*n10)/Iy
% % Iy = (4*kf*x1*n10)/0.3261
% % 计算飞机的转动惯量Iz
% 
% % 横侧向通道
% [0 0 0 9.8;...
%  0 0 0 0;...
%  0 0 0 0;...
%  0 1 0 0]
% 
% [0 0 0 0;...
%  0.28720 -0.28720 0.20120 -0.20120;...
%  0.01379 -0.01379 -0.00486 0.00486;...
%  0 0 0 0]

%%

%%

