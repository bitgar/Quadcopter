function [sys,x0,str,ts,simStateCompliance] = LateralDynamics(t,x,input,flag,mass,g,x1,x2,y1,y2,zr,kf,km,kd,Ix,Iy,Iz,Ixz)
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    
  case 1  % ★★★ 新增：计算状态导数 (dx/dt)
    sys = mdlDerivatives(t,x,input,mass,g,kf,kd);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys = [];

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,input,mass,g,x1,x2,y1,y2,zr,kf,km,kd,Ix,Iy,Iz,Ixz);

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

sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = 0;        % ★★★ 初始状态为0（或根据物理意义设置）

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

% ============================================================
function sys = mdlDerivatives(~,x,input,mass,g,kf,kd)
% flag=1: 计算状态导数 dx/dt
% x: 当前状态（标量，对应 d_beta 动态中的内部状态）

% 提取输入
dphi   = input(1);
V0     = input(2);
theta0 = input(3);
alpha0 = theta0;
gamma0 = input(4);
n10    = input(5);
n20    = input(6);
n30    = input(7);
n40    = input(8);

a_beta = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2) + g*sin(gamma0)/V0;

% 状态方程: dx/dt = a_beta * x + dphi
% 这就是传递函数 (s+b)/(s-a) 的状态实现
sys = a_beta * x + dphi;


% ============================================================
function sys=mdlOutputs(~,x,input,mass,g,x1,x2,y1,y2,zr,kf,km,kd,Ix,Iy,Iz,Ixz)
% Input
dphi =   input(1);

V0   =   input(2);
theta0 = input(3);
alpha0 = theta0;
gamma0 = input(4);

n10 =    input(5);
n20 =    input(6);
n30 =    input(7);
n40 =    input(8);

% lateral-directional dynamics 横侧向动力学
bp1 = (2*kf*y1*Iz)/(Ix*Iz-Ixz*Ixz)  +  2*(km-kd*y1)*Ixz/(Ix*Iz-Ixz*Ixz);
bp2 = (2*kf*y2*Iz)/(Ix*Iz-Ixz*Ixz)  -  2*(km+kd*y2)*Ixz/(Ix*Iz-Ixz*Ixz);
br1 = (2*kf*y1*Ixz)/(Ix*Iz-Ixz*Ixz) +  2*(km-kd*y1)*Ix/(Ix*Iz-Ixz*Ixz);
br2 = (2*kf*y2*Ixz)/(Ix*Iz-Ixz*Ixz) -  2*(km+kd*y2)*Ix/(Ix*Iz-Ixz*Ixz);

% dpsi/dphi
K_psi_phi = ...
(br1*(kf*x2+kd*zr)+br2*(kf*x1-kd*zr))/...
((bp1*cos(theta0)+br1*sin(theta0))*(kf*x2+kd*zr) +...
 (bp2*cos(theta0)+br2*sin(theta0))*(kf*x1-kd*zr));

dpsi = K_psi_phi * dphi;
dpsi = 0;

% d_beta/dphi
K_beta_phi = ...
((bp1*sin(theta0)-br1*cos(theta0))*(kf*x2+kd*zr) + (bp2*sin(theta0)-br2*cos(theta0))*(kf*x1-kd*zr))/...
((bp1+br1*tan(theta0))*(kf*x2+kd*zr)+(bp2+br2*tan(theta0))*(kf*x1-kd*zr));

b_beta = (g/V0)*K_beta_phi;
a_beta = 1/(mass*V0)*(kf*sin(alpha0)+kd*cos(alpha0))*(n10^2+n20^2+n30^2+n40^2) + g*sin(gamma0)/V0;

% ★★★ 正确的输出方程：d_beta = K_beta_phi * [(a_beta+b_beta)*x + dphi]
% 对应传递函数 K*(s+b)/(s-a) 的实现
d_beta = K_beta_phi * ((a_beta + b_beta) * x + dphi);
d_beta = 0;

% Last element
last_element = -sin(theta0) * dphi;

% d_chi
d_chi = dpsi + ...
        d_beta + ...
        last_element;
    
% d_chi = dpsi + ...
%         d_beta/cos(gamma0) + ...
%         last_element/cos(gamma0);

% Output
sys(1) = d_chi;

% end mdlOutputs    
% ============================================================
          
          