function [sys,x0,str,ts,simStateCompliance] = PlantDynamicsTowards(t,x,input,flag,mass,g,x1,x2,y1,y2,zr,rho,fb,kf,km,kd,Ix,Iy,Iz,Ixz)
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
    sys=mdlOutputs(t,x,input,mass,g,x1,x2,y1,y2,zr,rho,fb,kf,km,kd,Ix,Iy,Iz,Ixz);

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
sizes.NumOutputs     = 11;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,input,mass,g,x1,x2,y1,y2,zr,rho,fb,kf,km,kd,Ix,Iy,Iz,Ixz)
% description of input variable
% input(1): n1      motor 1 speed at epoch k
% input(2): n2      motor 2 speed at epoch k
% input(3): n3      motor 3 speed at epoch k
% input(4): n4      motor 4 speed at epoch k

% input(5):   u at epoch k
% input(6):   v at epoch k
% input(7):   w at epoch k

% input(8):   p at epoch k
% input(9):   q at epoch k
% input(10):  r at epoch k

% input(11): phi at epoch k
% input(12): theta at epoch k
% input(13): psi at epoch k

% Output variables
% sys(1):     a_xa at epoch k
% sys(2):     a_ya at epoch k
% sys(3):     a_za at epoch k
% sys(4):     p_dot at epoch k
% sys(5):     q_dot at epoch k
% sys(6):     r_dot at epoch k
% sys(7):     phi_dot at epoch k
% sys(8):     theta_dot at epoch k
% sys(9):     psi_dot at epoch k
% sys(10):    alpha at epoch k
% sys(11):    beta at epoch k

n1 = input(1);
n2 = input(2);
n3 = input(3);
n4 = input(4);

u = input(5);
v = input(6);
w = input(7);

p = input(8);
q = input(9);
r = input(10);

% epoch k euler angles§’
phi = input(11);
theta = input(12);
psi = input(13);

v_body = [u;v;w];
V = sqrt(u^2+v^2+w^2);
alpha = atan2(w,u);
beta = asin(v/V);

C_nb = [cos(phi)*cos(psi)     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(phi)*sin(psi)     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
       -sin(theta)            sin(phi)*cos(theta)                                cos(phi)*cos(theta)];
C_bn = C_nb';

C_ab = [cos(alpha)*cos(beta)    sin(beta)   sin(alpha)*cos(beta);...
       -cos(alpha)*sin(beta)    cos(beta)  -sin(alpha)*cos(beta);...
       -sin(alpha)                  0       cos(alpha)];

C_an = C_ab * C_bn;
C_na = C_an';

v_ned = C_nb * v_body;
V_ned = sqrt(v_ned(1)^2+v_ned(2)^2+v_ned(3)^2);

% Flight path angle
gamma = -asin(v_ned(3)/V_ned);
chi = atan2(v_ned(2),v_ned(1));
mu = asin(C_na(3,2)/cos(gamma));

Mx = kf * y1 * (n1^2 - n2^2) + kf * y2 * (n3^2 - n4^2);
My = (-kd*zr+kf*x1) * (n1^2 + n2^2) - (kd*zr+kf*x2) * x2 * (n3^2 + n4^2);
Mz = (km-kd*y1) * (n1^2 - n2^2) - (km+kd*y2) * (n3^2 - n4^2);

% sol_acceleration = [a_xa;a_ya;a_za]
sol_acceleration = [-1/mass*(kf*sin(alpha)*cos(beta)+kd*cos(alpha)*cos(beta))*(n1^2+n2^2+n3^2+n4^2)-1/(2*mass)*rho*V^2*fb-g*sin(gamma);...
                     1/mass*(kf*sin(alpha)*cos(beta)+kd*cos(alpha)*sin(beta))*(n1^2+n2^2+n3^2+n4^2)+g*cos(gamma)*sin(mu);...
                     1/mass*(-kf*cos(alpha)+kd*sin(alpha))*(n1^2+n2^2+n3^2+n4^2)+g*cos(gamma)*cos(mu)];

% epoch k acceleration in the frame of velocity
sys(1) = sol_acceleration(1);                       % a_
sys(2) = sol_acceleration(2);                       % a_
sys(3) = sol_acceleration(3);                       % a_

% rotational inertial
I = [Ix     0      -Ixz;...
     0      Iy      0;...
    -Ixz    0       Iz];

I_inv = inv(I); 
tau_total = [Mx-(Iz-Iy)*q*r+Ixz*p*q;...
             My-(Ix-Iz)*p*r-Ixz*(p^2-q^2);...
             Mz-(Iy-Ix)*p*q-Ixz*q*r];

sol_angular_acceleration = I_inv * tau_total;

sys(4) = sol_angular_acceleration(1);              % p_dot
sys(5) = sol_angular_acceleration(2);              % q_dot
sys(6) = sol_angular_acceleration(3);              % r_dot

% Matrix F, angular acceleration to euler attitude velocity
F_aa_to_av = [1     sin(phi)*tan(theta)     cos(phi)*tan(theta);...
              0     cos(phi)               -sin(phi);...
              0     sin(phi)/cos(theta)     cos(phi)/cos(theta)];

sol_attitude_velocity = F_aa_to_av * [p;q;r];

sys(7) = sol_attitude_velocity(1);                % phi_dot
sys(8) = sol_attitude_velocity(2);                % theta_dot
sys(9) = sol_attitude_velocity(3);                % psi_dot

sys(10) = alpha;                % epoch k alpha
sys(11) = beta;                 % epoch k beta

% end mdlOutputs    
