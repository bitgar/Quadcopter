function [sys,x0,str,ts,simStateCompliance] = AttitudeHorizon(t,x,u,flag,mass,g)
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
yaw_real = u(4);

% 小角度线性化模型控制器设计
R_yaw = [cos(yaw_real),-sin(yaw_real);...
         sin(yaw_real),cos(yaw_real)];
matrix_tmp = [0 1;...
             -1 0];
A_yaw = R_yaw * matrix_tmp;

acc_h = [u(1);u(2)];
acc_z = u(3);
    
sol = (inv(A_yaw) * acc_h)./(-g);

% 期望滚转角
expected_roll = sol(1);

% 期望俯仰角
expected_pitch = sol(2);

% 拉力
Force = mass * (g-acc_z);

%% 限幅
% angle_xian_roll = 15/180*pi;
% angle_xian_pitch = 90/180*pi;
% 
% if(exp_roll >= angle_xian_roll)
%     exp_roll = angle_xian_roll;
% elseif(exp_roll <= -angle_xian_roll)
%     exp_roll = -angle_xian_roll;
% end
% 
% if(exp_pitch >= angle_xian_pitch)
%     exp_pitch = angle_xian_pitch;
% elseif(exp_pitch <= -angle_xian_pitch)
%     exp_pitch = -angle_xian_pitch;
% end

%% 输出期望拉力与期望姿态角
sys(1) = Force;
sys(2) = expected_roll;
sys(3) = expected_pitch;

% end mdlOutputs    
