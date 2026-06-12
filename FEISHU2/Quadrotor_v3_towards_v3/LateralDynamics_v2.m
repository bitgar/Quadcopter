function [sys,x0,str,ts,simStateCompliance] = LateralDynamics_v2(t,x,input,flag,mass,g,x1,x2,y1,y2,zr,kf,km,kd,Ix,Iy,Iz,Ixz)
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
    sys=mdlOutputs(t,x,input);

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
sizes.NumOutputs     = 1;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

% ============================================================
function sys=mdlOutputs(~,~,input)
% Input
dpsi   = input(1);
dbeta  = input(2);
dphi   = input(3);
theta0 = input(4);

% d_chi
% d_chi = dpsi + ...
%         dbeta - ...
%         sin(theta0) * dphi;

% d_chi = 0.4584 * dphi;

d_chi = dpsi  + ...
        dbeta + ...
        0.4584 * dphi;

% d_chi = dpsi + ...
%         d_beta/cos(gamma0) + ...
%         last_element/cos(gamma0);

% Output
% sys(1) = sin(d_chi);
sys(1) = d_chi;

% end mdlOutputs    
% ============================================================
          
          