function [sys,x0,str,ts,simStateCompliance] = VelBodyToNED(t,x,input,flag)
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
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];


simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,input)

u = input(1);
v = input(2);
w = input(3);

phi = input(4);
theta = input(5);
psi = input(6);

C_nb = [cos(phi)*cos(psi)     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(phi)*sin(psi)     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta)           sin(phi)*cos(theta)                                cos(phi)*cos(theta)];

v_ned = C_nb * [u;v;w];
        
sys(1) = v_ned(1);
sys(2) = v_ned(2);
sys(3) = v_ned(3);

% end mdlOutputs    
