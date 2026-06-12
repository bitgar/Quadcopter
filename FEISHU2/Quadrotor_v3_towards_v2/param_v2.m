close all
clear all
clc

%% 飞机参数
mass= 40.37;
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
V0 = 30;

kf = rho*D^4*CT
km = rho*D^5*CQ
kd = rho*D^4*CDr

epsilon = atan(kd/kf);

n10 = 83.85;
n20 = 83.85;
n30 = 72.55;
n40 = 72.55;

deg2rad = 0.0175;
alpha0 = -27.213*deg2rad;
theta0 = alpha0;
gamma0 = 0;

%% 纵向运动状态方程 page90
% [dv;dalpha;dq;dtheta]
a_v     = 1/mass*(kf*cos(alpha0)-kd*sin(alpha0))*(n10^2+n20^2+n30^2+n40^2);
a_alpha = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2);

% b_v和b_alpha的正确公式
b_v =     (-2/mass)*(kf*sin(alpha0)+kd*cos(alpha0));
b_alpha = (2/(mass*V0))*(-kf*cos(alpha0)+kd*sin(alpha0));

% b_v和b_alpha的错误公式，但是与page198的小扰动模型相匹配。
% page200的垂向纵向垂直运动的3各传递函数是在此错误条件下推导出来的。
% D-24,D-25,D-26这3个公式全部都是错误的。
% b_v =     (-1/mass)*(kf*sin(alpha0)+kd*cos(alpha0));
% b_alpha = (1/(1*V0))*(-kf*cos(alpha0)+kd*sin(alpha0));

bq1 =      2/Iy*(-kd*zr+kf*x1);
bq2 =      2/Iy*(-kd*zr-kf*x1);

A_lon = [-rho*V0*fb0/mass      -rho*V0^2*Cfa/(2*mass)-a_v+g*cos(gamma0)       0      -g*cos(gamma0);...
         0                      a_alpha+g/V0*sin(gamma0)                      1      -g/V0*sin(gamma0);...
         0                      0                                             0       0;...
         0                      0                                             1       0]
     
B_lon = [b_v*n10                     b_v*n20                      b_v*n30                      b_v*n40;...
         b_alpha*n10                 b_alpha*n20                  b_alpha*n30                  b_alpha*n40;...
         bq1*n10                     bq1*n20                      bq2*n30                      bq2*n40;...
          0                           0                            0                            0]


a_beta = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2)+g*sin(gamma0)/V0;

bp1 = (2*kf*y1)*Iz/(Ix*Iz-Ixz*Ixz)  +  2*(km-kd*y1)*Ixz/(Ix*Iz-Ixz*Ixz);
bp2 = (2*kf*y2)*Iz/(Ix*Iz-Ixz*Ixz)  -  2*(km+kd*y2)*Ixz/(Ix*Iz-Ixz*Ixz);
br1 = (2*kf*y1)*Ixz/(Ix*Iz-Ixz*Ixz) +  2*(km-kd*y1)*Ix/(Ix*Iz-Ixz*Ixz);
br2 = (2*kf*y2)*Ixz/(Ix*Iz-Ixz*Ixz) -  2*(km+kd*y2)*Ix/(Ix*Iz-Ixz*Ixz);

A_lat = [a_beta             sin(alpha0)             -cos(alpha0)              g*cos(theta0)/V0;...
         0                  0                        0                        0;...
         0                  0                        0                        0;...
         0                  1                        tan(theta0)              0]
     
B_lat = [0             0             0              0;...
         bp1*n10      -bp1*n20       bp2*n30       -bp2*n40;...
         br1*n10      -br1*n20       br2*n30       -br2*n40;...
         0             0             0              0]
disp('-------------------------------------------------------')

%% dq、dtheta、du传递函数
% dn_e = dn1 - sqrt((kf*x2+kd*zr)/(kf*x1-kd*zr))*dn3
% dn_e >= 0 抬头
% dn_e <  0 低头

disp('dn_e = dn1 - sqrt((kf*x2+kd*zr)/(kf*x1-kd*zr))*dn3中的sqrt((kf*x2+kd*zr)/(kf*x1-kd*zr))为')
disp(sqrt((kf*x2+kd*zr)/(kf*x1-kd*zr)))

% dq/dn_e = (2*bq1*n10)/s
disp('dq/dn_e = (2*bq1*n10)/s中的系数为')
disp(2*bq1*n10)

% dtheta/dn_e = (2*bq1*n10)/s^2
disp('dtheta/dn_e = (2*bq1*n10)/s^2中的系数为')
disp(2*bq1*n10)

disp('-------------------------------------------------------')

%% dalpha传递函数
% 控制分配 1 2 3 4电机同时改变
% dn1 = dn2 ; dn3 = dn4 = sqrt((kf*x1-kd*zr)/(kf*x2+kd*zr))*dn1

disp('n1和n3之间的系数为')
disp(sqrt((kf*x1-kd*zr)/(kf*x2+kd*zr)))

% dalpha = G_alpha1 * dn1 + G_alpha3 * dn3
G_alpha1 = 2 * b_alpha * n10 * tf([1 -bq1/b_alpha (bq1/b_alpha)*(g/V0)],[1 -a_alpha-g/V0*sin(gamma0) 0 0]);
G_alpha3 = 2 * b_alpha * n30 * tf([1 -bq2/b_alpha (bq2/b_alpha)*(g/V0)],[1 -a_alpha-g/V0*sin(gamma0) 0 0]);
disp('G_alpha1为')
% disp(G_alpha1)
G_alpha1

disp('G_alpha3为')
% disp(G_alpha3)
G_alpha3

disp('-------------------------------------------------------')

%% dv传递函数
% 控制分配：?
G_v1 = -tf([0 (rho*V0^2*Cfa)/(2*mass)+a_v-g*cos(gamma0)] , [1 rho*V0*fb0/mass]);
G_v2 = -tf([0  g*cos(gamma0)] , [1 rho*V0*fb0/mass]);
G_v3 = tf([0  2*b_v] , [1 rho*V0*fb0/mass]);

% dv = G_v1 * d_alpha + G_v2 * d_theta + G_v3 * (n10*dn1 + n30*dn3)
disp('G_v1')
% disp(G_v1)
G_v1

disp('G_v2为')
% disp(G_v2)
G_v2

disp('G_v3为')
% disp(G_v3)
G_v3

disp('-------------------------------------------------------')

G_vn1 = G_v1*G_alpha1 + 2*G_v2*bq1*n10*tf([0 0 1],[1 0 0]) + G_v3*n10;
G_vn3 = G_v1*G_alpha3 + 2*G_v2*bq2*n10*tf([0 0 1],[1 0 0]) + G_v3*n30;

disp('G_vn1为')
% disp(G_vn1)
G_vn1

disp('G_vn3为')
% disp(G_vn3)
G_vn3

%% dr、dpsi传递函数
% 控制分配：dn_r = dn1 - dn2

% dr/dn_r = br1*n10*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s
disp('dr/dnr = br1*n10*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s中的系数为')
disp(br1*n10*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))

% dpsi/dn_r = br1*n10/cos(theta0)*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s^2
disp('dpsi/dnr = br1*n10/cos(theta0)*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s^2中的系数为')
disp(br1*n10/cos(theta0)*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))

% 控制分配：dn_a = dn1 - dn2
% dr/dn_a = br1*n10*(1+br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s
disp('dr/dna = br1*n10*(1+br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s中的系数为')
disp(br1*n10*(1+br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))

disp('-------------------------------------------------------')

%% dp、dphi传递函数
% 控制分配：dn_a = dn1 - dn2

% dp/dn_a = bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s
disp('dp/dna = bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s中的系数为')
disp(bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))
disp('---------------------------------------------')

% dphi/dn_a = bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s^2
disp('dphi/dna = (a+b)/s^2')
disp(bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))
disp(br1*n10*tan(theta0)*(1+br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))
disp(bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))+br1*n10*tan(theta0)*(1+br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))
disp('---------------------------------------------')

% 控制分配：dn_r = dn1 - dn2
% dp/dn_r = bp1*n10*(1-bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s
disp('dp/dn_r = bp1*n10*(1-bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))/s中的系数为')
disp(bp1*n10*(1-bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr)))
disp('---------------------------------------------')

%% dbeta传递函数
% 控制分配：dn_r = dn1 - dn2
% dbeta/dn_r =
% [(Kpr*sin(alpha0)-Krr*cos(alpha0))s+g/V0*(Kpr+Krr*tan(theta0))] / s^2*(s-a_beta)
% page99 我推测上面的公式有问题，应该是
% [(Kpr*sin(alpha0)-Krr*cos(alpha0))s+g/V0*(Kpr*cos(theta0)+Krr*sin(theta0))] / s^2*(s-a_beta)
Kpr = bp1*n10*(1-bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))
Krr = br1*n10*(1-br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))
disp('dbeta/dn_r 传递函数：')
G_beta_dnr = tf([(Kpr*sin(alpha0)-Krr*cos(alpha0)),g/V0*(Kpr*cos(theta0)+Krr*sin(theta0))],[1 -a_beta 0 0])
disp('---------------------------------------------')

% 控制分配：dn_a = dn1 - dn2
% dbeta/dn_a =
% [(Kpa*sin(alpha0)-Kra*cos(alpha0))s+g/V0*(Kpa+Kra*tan(theta0))] / s^2*(s-a_beta)
% page99 我推测上面的公式有问题，应该是
% [(Kpa*sin(alpha0)-Kra*cos(alpha0))s+g/V0*(Kpa*cos(theta0)+Kra*sin(theta0))] / s^2*(s-a_beta)
Kpa = bp1*n10*(1+bp2/bp1*(kf*x1-kd*zr)/(kf*x2+kd*zr))
Kra = br1*n10*(1+br2/br1*(kf*x1-kd*zr)/(kf*x2+kd*zr))
disp('dbeta/dn_a 传递函数：')
G_beta_dna = tf([(Kpa*sin(alpha0)-Kra*cos(alpha0)),g/V0*(Kpa*cos(theta0)+Kra*sin(theta0))],[1 -a_beta 0 0])
disp('---------------------------------------------')

%% 纵向俯仰运动传递函数中的公式D-22   page199
% a_v=1/mass*(kf*cos(alpha0)-kd*sin(alpha0))*(n10^2+n20^2+n30^2+n40^2)
k1 = -(rho*V0^2*Cfa)/(2*mass) - a_v + g*cos(gamma0);
k2 = -g*cos(gamma0);

disp('dV/dn_e的传递函数为')
G_dV_dne = tf([2*bq1*n10*(k1+k2) -2*bq1*n10*a_alpha*k2],[1 rho*V0*Cfa/mass-a_alpha -a_alpha*rho*V0*Cfa/mass 0 0])
% 我怀疑page199的0.5097是错的，应该放大10倍才对

0.3416*(k1+k2)
0.3416*0.1421*k2
% 0.3416*0.1421*k2 or 0.04854*k2

0.3416*(k1+k2)/0.3416
0.3416*0.1421*k2/0.3416
disp('---------------------------------------------')

%% 纵向通道的垂直运动 page200
% sqrt((kf*x2+kd*zr)/(kf*x1-kd*zr))
% 1/sqrt((kf*x2+kd*zr)/(kf*x1-kd*zr))
k1 = -(rho*V0^2*Cfa)/(2*mass) - 1/mass*(kf*cos(alpha0)-kd*sin(alpha0))*(n10^2+n20^2+n30^2+n40^2) + g*cos(gamma0)
k2 = -g*cos(gamma0)
k3 = 2*b_v

p = n10*kf*(x1+x2)/(kf*x2+kd*zr)

% page200写错了，不是速度增量对dn1的传递函数，而应该是高度增量对dn1的传递函数
disp('---------------------------------------------')
disp('dH/dn1的传递函数为')
G_dH_dn1 = tf([0 0 -2*V0*b_alpha*n10*kf*(x1+x2)/(kf*x2+kd*zr)],[1 -a_alpha 0])
disp('---------------------------------------------')

disp('dalpha/dn1的传递函数为')
G_dalpha_dn1 = tf([0 2*b_alpha*n10*kf*(x1+x2)/(kf*x2+kd*zr)],[1 -a_alpha])
disp('---------------------------------------------')

disp('dV/dn1的传递函数为')
G_V_dn1 = tf([k3*p (2*b_alpha*k1-a_alpha*k3)*p],[1 rho*V0*Cfa/mass-a_alpha -a_alpha*rho*V0*Cfa/mass])
% fprintf('%.6g*s %+ .6g\n', k3*p, (2*b_alpha*k1-a_alpha*k3)*p);
disp('---------------------------------------------')

%% dpsi/dphi
K_psi_phi = ...
(br1*(kf*x2+kd*zr)+br2*(kf*x1-kd*zr))/...
((bp1*cos(theta0)+br1*sin(theta0))*(kf*x2+kd*zr) +...
 (bp2*cos(theta0)+br2*sin(theta0))*(kf*x1-kd*zr));

disp(['K_psi_phi为:  ' num2str(K_psi_phi)])

disp('---------------------------------------------')

%% d_beta/dphi
K_beta_phi = ...
((bp1*sin(theta0)-br1*cos(theta0))*(kf*x2+kd*zr) + (bp2*sin(theta0)-br2*cos(theta0))*(kf*x1-kd*zr))/...
((bp1+br1*tan(theta0))*(kf*x2+kd*zr)+(bp2+br2*tan(theta0))*(kf*x1-kd*zr));

b_beta = (g/V0*cos(theta0))/K_beta_phi;
a_beta = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2) + g*sin(gamma0)/V0;

disp(['K_beta_phi为:  ' num2str(K_beta_phi)])
disp(['b_beta为:  ' num2str(b_beta)])
disp(['a_beta为:  ' num2str(a_beta)])

disp('---------------------------------------------')

%% Last element
disp(['-sin(theta0)为:  ' num2str(-sin(theta0))])

disp('---------------------------------------------')

%%

