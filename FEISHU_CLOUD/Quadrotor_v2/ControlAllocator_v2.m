function [sys,x0,str,ts,simStateCompliance] = ControlAllocator_v2(t,x,u,flag,x1,x2)
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
    sys=mdlOutputs(t,x,u,x1,x2);

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

function sys=mdlOutputs(~,~,u,x1,x2)
% 输入1：机体系旋转角速度（上一时刻）
% u(1): 等效输入dn_hc
% u(2): 等效输入dn_ec
% u(3): 等效输入dn_ac
% u(4): 等效输入dn_rc

% 输出：期望的电机增量转速（当前时刻）
% sys(1): 期望的电机1增量转速 dn1 
% sys(2): 期望的电机2增量转速 dn2
% sys(3): 期望的电机3增量转速 dn3
% sys(4): 期望的电机4增量转速 dn4

%% Input
dn_hc = u(1);
dn_ec = u(2);
dn_ac = u(3);
dn_rc = u(4);

%% Allocator
dn1 = 0;
dn2 = 0;
dn3 = 0;
dn4 = 0;

% altitude control
dn1 = dn1 + dn_hc;
dn2 = dn2 + dn_hc;
dn3 = dn3 + sqrt(x1/x2)*dn_hc;
dn4 = dn4 + sqrt(x1/x2)*dn_hc;

% pitch control
if(dn_ec >= 0)
    dn1 = dn1 + dn_ec;
    dn2 = dn2 + dn_ec;
    dn3 = dn3 + 0;
    dn4 = dn4 + 0;
else
    dn1 = dn1 + 0;
    dn2 = dn2 + 0;
    dn3 = dn3 - sqrt(x1/x2)*dn_ec;
    dn4 = dn4 - sqrt(x1/x2)*dn_ec;
end

% roll control
if(dn_ac >= 0)
    dn1 = dn1 + dn_ac;
    dn2 = dn2 + 0;
    dn3 = dn3 + sqrt(x1/x2)*dn_ac;
    dn4 = dn4 + 0;
else
    dn1 = dn1 + 0;
    dn2 = dn2 - dn_ac;
    dn3 = dn3 + 0;
    dn4 = dn4 - sqrt(x1/x2)*dn_ac;
end

% yaw control
if(dn_rc >= 0)
    dn1 = dn1 + dn_rc;
    dn2 = dn2 + 0;
    dn3 = dn3 + 0;
    dn4 = dn4 + dn_rc;
else
    dn1 = dn1 + 0;
    dn2 = dn2 - sqrt(x1/x2)*dn_rc;
    dn3 = dn3 - sqrt(x1/x2)*dn_rc;
    dn4 = dn4 + 0;
end

%% Output
sys(1) = dn1;
sys(2) = dn2;
sys(3) = dn3;
sys(4) = dn4;

% end mdlOutputs    
