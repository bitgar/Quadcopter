function [sys,x0,str,ts,simStateCompliance] = SmallPerturbationLinearizedModelTowards(t,x,input,flag,mass,g,x1,x2,y1,y2,zr,rho,fb0,Cfa,kf,km,kd,Ix,Iy,Iz,Ixz)
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys = [];

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,input,mass,g,x1,x2,y1,y2,zr,rho,fb0,Cfa,kf,km,kd,Ix,Iy,Iz,Ixz);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end Quadrotor_Plant

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 20;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,input,mass,g,x1,x2,y1,y2,zr,rho,fb0,Cfa,kf,km,kd,Ix,Iy,Iz,Ixz)
% input(1): dn1                                     （当前时刻）
% input(2): dn2                                     （当前时刻）
% input(3): dn3                                     （当前时刻）
% input(4): dn4                                     （当前时刻）

% input(5): V0                                      （当前时刻）
% input(6): theta0                                  （当前时刻）
% input(7): alpha0                                  （当前时刻）
% input(8): gamma0                                  （当前时刻）

% input(9): n10                                     （当前时刻）
% input(10): n20                                    （当前时刻）
% input(11): n30                                    （当前时刻）
% input(12): n40                                    （当前时刻）

% input(13): dV                                      （当前时刻）
% input(14): dalpha                                  （当前时刻）
% input(15): dbeta                                   （当前时刻）
% input(16): dtheta                                  （当前时刻）
% input(17): dphi                                    （当前时刻）

% input(18): dp                                      （当前时刻）
% input(19): dq                                      （当前时刻）
% input(20): dr                                      （当前时刻）


% sys(1): 加速度dV_dot                               （当前时刻）
% sys(2): 加速度dalpha_dot                           （当前时刻）
% sys(3): 加速度dbeta_dot                            （当前时刻）
% sys(4): 角加速度dp_dot                             （当前时刻）
% sys(5): 角加速度dq_dot                             （当前时刻）
% sys(6): 角加速度dr_dot                             （当前时刻）
% sys(7): 滚转角速度dphi_dot                         （当前时刻）
% sys(8): 俯仰角速度dtheta_dot                       （当前时刻）
% sys(9): 偏航角速度dpsi_dot                         （当前时刻）

%% Input
dn1 =   input(1);
dn2 =   input(2);
dn3 =   input(3);
dn4 =   input(4);
% dn1 =   0;
% dn2 =   0;
% dn3 =   0;
% dn4 =   0;

V0 =    input(5);
theta0 =input(6);
alpha0 =input(7);
gamma0 =input(8);

n10 =   input(9);
n20 =   input(10);
n30 =   input(11);
n40 =   input(12);

dV =    input(13);
dalpha= input(14); 
dbeta=  input(15); 
dtheta= input(16);
dphi =  input(17);
dp =    input(18);
dq =    input(19);
dr =    input(20);

control = [dn1;dn2;dn3;dn4];

%% longitudinal dynamics 纵向动力学
% longitudinal control channel 纵向控制通道
x_lon = [dV;dalpha;dq;dtheta];
x_lon_dot = [0;0;0;0];

a_v = -1/mass*(kf*cos(alpha0)-kd*sin(alpha0))*(n10^2+n20^2+n30^2+n40^2);
% a_v = 1/mass*(kf*cos(alpha0)-kd*sin(alpha0))*(n10^2+n20^2+n30^2+n40^2);
a_a = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2);

bv = -2/mass*(kf*sin(alpha0)+kd*cos(alpha0));
ba = 2/(mass*V0)*(-kf*cos(alpha0)+kd*sin(alpha0));
% bv =     (-1/mass)*(kf*sin(alpha0)+kd*cos(alpha0));
% ba = (1/(1*V0))*(-kf*cos(alpha0)+kd*sin(alpha0));

bq1 = 2/Iy*(-kd*zr+kf*x1);
bq2 = 2/Iy*(-kd*zr-kf*x2);

A_lon = [-rho*V0*fb0/mass      -rho*V0^2*Cfa/(2*mass)-a_v+g*cos(gamma0)       0      -g*cos(gamma0);...
         0                      a_a+g/V0*sin(gamma0)                          1      -g*sin(gamma0)/V0;...
         0                      0                                             0       0;...
         0                      0                                             1       0];

B_lon = [bv*n10                     bv*n20                      bv*n30                      bv*n40;...
         ba*n10                     ba*n20                      ba*n30                      ba*n40;...
         bq1*n10                    bq1*n20                     bq2*n30                     bq2*n40;...
         0                          0                           0                           0];

x_lon_dot = A_lon * x_lon + B_lon * control;

dV_dot =            x_lon_dot(1);
dalpha_dot =        x_lon_dot(2);
dq_dot =            x_lon_dot(3);
dtheta_dot =        x_lon_dot(4);

%% lateral-directional dynamics 横侧向动力学
% lateral-directional control channel 横侧向控制通道
x_lat = [dbeta;dp;dr;dphi];
x_lat_dot = [0;0;0;0];

bp1 = (2*kf*y1*Iz)/(Ix*Iz-Ixz*Ixz)  +  2*(km-kd*y1)*Ixz/(Ix*Iz-Ixz*Ixz);
bp2 = (2*kf*y2*Iz)/(Ix*Iz-Ixz*Ixz)  -  2*(km+kd*y2)*Ixz/(Ix*Iz-Ixz*Ixz);
br1 = (2*kf*y1*Ixz)/(Ix*Iz-Ixz*Ixz) +  2*(km-kd*y1)*Ix/(Ix*Iz-Ixz*Ixz);
br2 = (2*kf*y2*Ixz)/(Ix*Iz-Ixz*Ixz) -  2*(km+kd*y2)*Ix/(Ix*Iz-Ixz*Ixz);

a_beta = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2) + g*sin(gamma0)/V0;

A_lat = [a_beta             sin(alpha0)             -cos(alpha0)              g/V0;...
         0                  0                        0                        0;...
         0                  0                        0                        0;...
         0                  1                        tan(theta0)              0];
     
B_lat = [0             0             0              0;...
         bp1*n10      -bp1*n20       bp2*n30       -bp2*n40;...
         br1*n10      -br1*n20       br2*n30       -br2*n40;...
         0             0             0              0];

x_lat_dot = A_lat * x_lat + B_lat * control;

dbeta_dot =            x_lat_dot(1);
dp_dot =            x_lat_dot(2);
dr_dot =            x_lat_dot(3);
dphi_dot =          x_lat_dot(4);     

%% Heading channel
dpsi_dot =          dr/cos(theta0);

%% Output
sys(1) = dV_dot;
sys(2) = dalpha_dot;
sys(3) = dbeta_dot;
sys(4) = dp_dot;
sys(5) = dq_dot;
sys(6) = dr_dot;
sys(7) = dphi_dot;
sys(8) = dtheta_dot;
sys(9) = dpsi_dot;

% end mdlOutputs    
