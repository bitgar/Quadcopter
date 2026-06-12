function [sys,x0,str,ts,simStateCompliance] = oula_zhuan(t,x,u,flag)
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
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];


simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,u)

sys(1) = u(1);
sys(2) = u(2);
sys(3) = u(3);

%%
%Å·À­½Ç×ª»»

% pai=3.1415926535;
% 
% fai_real=u(1);
% theta_real=u(2);
% kesi_real=u(3);
% 
% fai_real_tmp=mod(fai_real,2*pai);
% theta_real_tmp=mod(theta_real,2*pai);
% kesi_real_tmp=mod(kesi_real,2*pai);
% 
% if (fai_real_tmp > 0) && (fai_real_tmp < pai/2.0) 
%     fai_real = fai_real_tmp;
% elseif (fai_real_tmp >= pai/2.0) && (fai_real_tmp < pai)
%     fai_real = fai_real_tmp;%pai/2.0-0.001;
% elseif (fai_real_tmp >= pai) && (fai_real_tmp < 1.5*pai)
%     fai_real = fai_real_tmp-2*pai;%1.5*pai+0.001;
% elseif (fai_real_tmp >= 1.5*pai) && (fai_real_tmp < 2.0*pai)
%     fai_real = fai_real_tmp-2*pai;    
% end
% 
% if (theta_real_tmp > 0) && (theta_real_tmp < pai/2.0) 
%     theta_real = theta_real_tmp;
% elseif (theta_real_tmp >= pai/2.0) && (theta_real_tmp < pai)
%     theta_real = theta_real_tmp;%pai/2.0-0.001;
% elseif (theta_real_tmp >= pai) && (theta_real_tmp < 1.5*pai)
%     theta_real = theta_real_tmp-2*pai;%1.5*pai+0.001;
% elseif (theta_real_tmp >= 1.5*pai) && (theta_real_tmp < 2.0*pai)
%     theta_real = theta_real_tmp-2*pai;    
% end
% 
% if (kesi_real_tmp > 0) && (kesi_real_tmp < pai) 
%     kesi_real = kesi_real_tmp;
% elseif (kesi_real_tmp >= pai) && (kesi_real_tmp < 2.0*pai)
%     kesi_real = kesi_real_tmp-2*pai;    
% end
% 
% sys(1) = fai_real;
% sys(2) = theta_real;
% sys(3) = kesi_real;

% end mdlOutputs    
