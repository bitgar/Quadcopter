function [sys,x0,str,ts,simStateCompliance] = Plant_Kinematics(t,x,u,flag)
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
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];


simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,u)
% u(1): px加速度（惯性系）
% u(2): py加速度（惯性系）
% u(3): pz加速度 （惯性系）
% u(4): 绕X轴旋转角速度（机体）加速度
% u(5): 俯仰角（机体）加速度
% u(6): 偏航角（机体）加速度

% u(7): 真实滚转角（欧拉） 暂时删除
% u(8): 真实俯仰角（欧拉） 暂时删除

% sys(1): px加速度（惯性系）
% sys(2): py加速度（惯性系）
% sys(3): pz加速度 （惯性系）
% sys(4): 滚转角（欧拉）加速度
% sys(5): 俯仰角（欧拉）加速度
% sys(6): 偏航角（欧拉）加速度

% 惯性坐标系下的线速度（线速度在动力学模型中已经进行了坐标转换）（平动）
sys(1) = u(1);
sys(2) = u(2);
sys(3) = u(3);
% 机体角速度转换为欧拉角速度（转动）
sys(4) = u(4);
sys(5) = u(5);
sys(6) = u(6);
% sys(4) = u(4) + sin(u(7))*tan(u(8))*u(5) + cos(u(7))*tan(u(8))*u(6);
% sys(5) =        cos(u(7))*u(5)                  - sin(u(7))*u(6);
% sys(6) =        sin(u(7))/cos(u(8))*u(5) + cos(u(7))/cos(u(8))*u(6);

% end mdlOutputs    
