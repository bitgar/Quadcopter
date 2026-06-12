function [sys,x0,str,ts,simStateCompliance] = Plant_Dynamics(t,x,u,flag,mass,g,Jxx,Jyy,Jzz)
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
    sys=mdlOutputs(t,x,u,mass,g,Jxx,Jyy,Jzz);

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
sizes.NumOutputs     = 6;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,u,mass,g,Jxx,Jyy,Jzz)

% u(1): 拉力                    
% u(2): 力矩 x
% u(3): 力矩 y
% u(4): 力矩 z  
% u(5): 滚转角（欧拉角）
% u(6): 俯仰角（欧拉角）
% u(7): 偏航角（欧拉角）

% sys(1): px加速度（惯性系）
% sys(2): py加速度（惯性系）
% sys(3): pz加速度（惯性系）
% sys(4): 滚转角加速度（机体坐标系）
% sys(5): 俯仰角加速度（机体坐标系）
% sys(6): 偏航角加速度（机体坐标系）

% 输出线加速度（惯性系）
% 加速度非线性模型如下
% sys(1) = -u(1)/mass*(cos(u(5))*sin(u(6))*cos(u(7)) + sin(u(5))*sin(u(7)));
% sys(2) = -u(1)/mass*(cos(u(5))*sin(u(6))*sin(u(7)) - sin(u(5))*cos(u(7)));
% sys(3) = g - u(1)/mass*(cos(u(5))*cos(u(6)));

% 输入
T       = u(1);
tau_x   = u(2);
tau_y   = u(3);
tau_z   = u(4);
roll    = u(5);
pitch   = u(6);
yaw     = u(7);

% 加速度线性模型如下
sys(1) = -g*(pitch*cos(yaw) + roll*sin(yaw));
sys(2) = -g*(pitch*sin(yaw) - roll*cos(yaw));
sys(3) = g - T/mass;

% 输出角加速度（机体坐标系）
% 角加速度线性模型如下
sys(4) = tau_x/Jxx;
sys(5) = tau_y/Jyy;
sys(6) = tau_z/Jzz;

% end mdlOutputs    
