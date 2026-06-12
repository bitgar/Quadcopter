function [sys,x0,str,ts,simStateCompliance] = AccVelToBody(t,x,input,flag)
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
sizes.NumOutputs     = 3;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];


simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,input)
ax_a = input(1);
ay_a = input(2);
az_a = input(3);

alpha = input(4);
beta = input(5);

acc_a = [ax_a;ay_a;az_a];

C_ab = [cos(alpha)*cos(beta)    sin(beta)   sin(alpha)*cos(beta);...
       -cos(alpha)*sin(beta)    cos(beta)  -sin(alpha)*cos(beta);...
       -sin(alpha)                  0       cos(alpha)];
C_ba = C_ab';

acc_body = C_ba * acc_a;
        
sys(1) = acc_body(1);
sys(2) = acc_body(2);
sys(3) = acc_body(3);

% end mdlOutputs    
