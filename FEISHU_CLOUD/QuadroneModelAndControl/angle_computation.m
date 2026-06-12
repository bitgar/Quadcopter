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

%%

%%
%非线性模型控制器设计


%%
%小角度线性化模型控制器设计
R_yaw = [cos(u(4)),-sin(u(4));...
         sin(u(4)),cos(u(4))];
matrix_tmp = [0 1;...
             -1 0];
A_yaw = R_yaw * matrix_tmp;

acc_h = [u(1);u(2)];

sol = -1/g * inv(A_yaw) * acc_h;
% sol = 1/g * inv(A_yaw) * acc_h;

% 滚转角
exp_roll = sol(1);
% 俯仰角
exp_pitch = sol(2);

% 拉力
Force = mass*g - mass*u(3);

%限幅
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

sys(1) = Force;
sys(2) = exp_roll;
sys(3) = exp_pitch;

%%

% %转换
% pi=3.1415926535;
% 
% fai_real=fai_exp;
% theta_real=theta_exp;
% 
% fai_real_tmp=mod(fai_real,2*pi);
% theta_real_tmp=mod(theta_real,2*pi);
% 
% if (fai_real_tmp > 0) && (fai_real_tmp < pi/2.0) 
%     fai_real = fai_real_tmp;
% elseif (fai_real_tmp >= pi/2.0) && (fai_real_tmp < pi)
%     fai_real = fai_real_tmp;%pi/2.0-0.001;
% elseif (fai_real_tmp >= pi) && (fai_real_tmp < 1.5*pi)
%     fai_real = fai_real_tmp-2*pi;%1.5*pi+0.001;
% elseif (fai_real_tmp >= 1.5*pi) && (fai_real_tmp < 2.0*pi)
%     fai_real = fai_real_tmp-2*pi;    
% end
% 
% if (theta_real_tmp > 0) && (theta_real_tmp < pi/2.0) 
%     theta_real = theta_real_tmp;
% elseif (theta_real_tmp >= pi/2.0) && (theta_real_tmp < pi)
%     theta_real = theta_real_tmp-2*pi;%pi/2.0-0.001;
% elseif (theta_real_tmp >= pi) && (theta_real_tmp < 1.5*pi)
%     theta_real = theta_real_tmp-2*pi;%1.5*pi+0.001;
% elseif (theta_real_tmp >= 1.5*pi) && (theta_real_tmp < 2.0*pi)
%     theta_real = theta_real_tmp-2*pi;    
% end

% end mdlOutputs    
