function [sys,x0,str,ts,simStateCompliance] = Attitude_Dynamics(t,x,u,flag)
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
    sys=mdlOutputs(t,x,u);

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

function sys=mdlOutputs(~,~,u)
% 输入1：机体系旋转角速度（上一时刻）
% u(1): 机体轴x旋转角速度 rad/s                    
% u(2): 机体轴y旋转角速度 rad/s
% u(3): 机体轴z旋转角速度 rad/s

% 输入2：姿态角（上一时刻）
% u(4): 上一时刻滚转角phi rad  
% u(5): 上一时刻俯仰角theta rad
% u(6): 上一时刻偏航角psi rad

% 输出：姿态角速度（当前时刻）
% sys(1): 当前时刻滚转角加速度 rad/ss
% sys(2): 当前时刻俯仰角加速度 rad/ss
% sys(3): 当前时刻偏航角加速度 rad/ss

p = u(1);
q = u(2);
r = u(3);

phi = u(4);
theta = u(5);
psi = u(6);

sys(1) = p + (q*sin(phi)+r*cos(phi))*tan(theta);
sys(2) = q*cos(phi) - r*sin(phi);
sys(3) = (q*sin(phi)+r*cos(phi))/cos(theta);

% end mdlOutputs    
