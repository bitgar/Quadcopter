function [sys,x0,str,ts,simStateCompliance] = angle_computation(t,x,u,flag,mass,g)
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
    sys=mdlOutputs(t,x,u,mass,g);

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
sizes.NumOutputs     = 3;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];


simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,u,mass,g)
% 小角度线性化模型控制器设计
R_yaw = [cos(u(4)),-sin(u(4));...
         sin(u(4)),cos(u(4))];
matrix_tmp = [0 1;...
             -1 0];
A_yaw = R_yaw * matrix_tmp;

acc_h = [u(1);u(2)];

sol = -1/g * inv(A_yaw) * acc_h;

% 滚转角
exp_roll = sol(1);
% 俯仰角
exp_pitch = sol(2);

% 拉力
Force = mass*g - mass*u(3);

sys(1) = Force;
sys(2) = exp_roll;
sys(3) = exp_pitch;

% end mdlOutputs    
