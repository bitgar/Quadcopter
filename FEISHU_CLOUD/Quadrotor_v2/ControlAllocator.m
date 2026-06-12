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
sizes.NumOutputs     = 4;
sizes.NumInputs      = 4;
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
% u(1): 期望拉力 N*m                    
% u(2): 转矩Mx 
% u(3): 转矩My
% u(4): 转矩Mz

% 输出：姿态角速度（当前时刻）
% sys(1): 当前时刻电机1转速 
% sys(2): 当前时刻电机2转速
% sys(3): 当前时刻电机3转速
% sys(4): 当前时刻电机4转速

kf = 0.00043;
km = 0.00078;

x1 = 21;
x2 = 29;

y1 = 55;
y2 = 45;

T = u(1);
Mx = u(2);
My = u(3);
Mz = u(4);

A = [kf      kf      kf      kf;...
     kf*y1  -kfy1   -kf*y2   kf*y2;...
     kf*x1   kf*x1  -kf*x2  -kf*x2;...
     km      -km      km      -km];

sol = inv(A)*[T;Mx;My;Mz];

n1 = sqrt(sol(1));
n2 = sqrt(sol(2));
n3 = sqrt(sol(3));
n4 = sqrt(sol(4));

sys(1) = n1;
sys(2) = n2;
sys(3) = n3;
sys(4) = n4;

% end mdlOutputs    
