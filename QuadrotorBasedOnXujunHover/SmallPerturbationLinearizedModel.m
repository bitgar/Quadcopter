function [sys,x0,str,ts,simStateCompliance] = SmallPerturbationLinearizedModel(t,x,input,flag,mass,g,x1,x2,y1,y2,kf,km,Ix,Iy,Iz,Ixz)
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
    sys=mdlOutputs(t,x,input,mass,g,x1,x2,y1,y2,kf,km,Ix,Iy,Iz,Ixz);

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
sizes.NumInputs      = 17;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,input,mass,g,x1,x2,y1,y2,kf,km,Ix,Iy,Iz,Ixz)

% input(1): dn1 （当前时刻）
% input(2): dn2 （当前时刻）
% input(3): dn3 （当前时刻）
% input(4): dn4  （当前时刻）

% input(5): du （机体系x轴） （当前时刻）
% input(6): dv （机体系y轴） （当前时刻）
% input(7): dw （机体系z轴） （当前时刻）

% input(8): dp （机体系x轴） （当前时刻）
% input(9): dq （机体系y轴） （当前时刻）
% input(10): dr （机体系z轴） （当前时刻）

% input(11): dphi （当前时刻）
% input(12): dtheta （当前时刻）
% input(13): dpsi （当前时刻）

% input(14): n10 （当前时刻）
% input(15): n20 （当前时刻）
% input(16): n30 （当前时刻）
% input(17): n40 （当前时刻）

% sys(1): 加速度du_dot                              （机体系x）      （当前时刻）
% sys(2): 加速度dv_dot                              （机体系y）      （当前时刻）
% sys(3): 加速度dw_dot                              （机体系z）      （当前时刻）
% sys(4): 角加速度dp_dot                            （机体坐标系x）  （当前时刻）
% sys(5): 角加速度dq_dot                            （机体坐标系y）  （当前时刻）
% sys(6): 角加速度dr_dot                            （机体坐标系z）  （当前时刻）
% sys(7): 滚转角速度dphi_dot                        （NED坐标系x）   （当前时刻）
% sys(8): 俯仰角速度dtheta_dot                      （NED坐标系y）   （当前时刻）
% sys(9): 偏航角速度dpsi_dot                        （NED坐标系z）   （当前时刻）

%% Input
dn1 = input(1);
dn2 = input(2);
dn3 = input(3);
dn4 = input(4);

du = input(5);
dv = input(6);
dw = input(7);

dp = input(8);
dq = input(9);
dr = input(10);

dphi = input(11);
dtheta = input(12);
dpsi = input(13);

n10 = input(14);
n20 = input(15);
n30 = input(16);
n40 = input(17);

control = [dn1;dn2;dn3;dn4];


%% longitudinal dynamics 纵向动力学
% longitudinal control channel 纵向控制通道
x_lon = [du;dw;dq;dtheta];
x_lon_dot = [0;0;0;0];
A_lon = [0      0       0      -g;...
         0      0       0       0;...
         0      0       0       0;...
         0      0       1       0];
     
B_lon = [0                     0                      0                      0;...
         -(2*kf*n10)/mass      -(2*kf*n20)/mass       -(2*kf*n30)/mass       -(2*kf*n40)/mass;...
         (2*kf*x1*n10)/Iy      (2*kf*x1*n20)/Iy       -(2*kf*x2*n30)/Iy      -(2*kf*x2*n40)/Iy;...
         0                     0                      0                      0];
     
x_lon_dot = A_lon * x_lon + B_lon * control;

du_dot = x_lon_dot(1);
dw_dot = x_lon_dot(2);
dq_dot = x_lon_dot(3);
dtheta_dot = x_lon_dot(4);

%% lateral-directional dynamics 横侧向动力学
% lateral-directional control channel 横侧向控制通道
x_lat = [dv;dp;dr;dphi];
x_lat_dot = [0;0;0;0];

bp1 = (2*kf*y1*Iz)/(Ix*Iz-Ixz*Ixz) + (2*km*Ixz)/(Ix*Iz-Ixz*Ixz);
bp2 = (2*kf*y2*Iz)/(Ix*Iz-Ixz*Ixz) - (2*km*Ixz)/(Ix*Iz-Ixz*Ixz);
br1 = (2*kf*y1*Ixz)/(Ix*Iz-Ixz*Ixz) + (2*km*Ix)/(Ix*Iz-Ixz*Ixz);
br2 = (2*kf*y2*Ixz)/(Ix*Iz-Ixz*Ixz) - (2*km*Ix)/(Ix*Iz-Ixz*Ixz);

A_lat = [0             0             0              g;...
         0             0             0              0;...
         0             0             0              0;...
         0             1             0              0];
     
B_lat = [0             0             0              0;...
         bp1*n10      -bp1*n20       bp2*n30       -bp2*n40;...
         br1*n10      -br1*n20       br2*n30       -br2*n40;...
         0             0             0              0];

x_lat_dot = A_lat * x_lat + B_lat * control;

dv_dot = x_lat_dot(1);
dp_dot = x_lat_dot(2);
dr_dot = x_lat_dot(3);
dphi_dot = x_lat_dot(4);     

%% Heading channel
dpsi_dot = dr;

%% Output
sys(1) = du_dot;
sys(2) = dv_dot;
sys(3) = dw_dot;
sys(4) = dp_dot;
sys(5) = dq_dot;
sys(6) = dr_dot;
sys(7) = dphi_dot;
sys(8) = dtheta_dot;
sys(9) = dpsi_dot;

% end mdlOutputs    
