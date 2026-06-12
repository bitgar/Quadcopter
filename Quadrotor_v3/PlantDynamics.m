function [sys,x0,str,ts,simStateCompliance] = PlantDynamics(t,x,input,flag,mass,g,Ix,Iy,Iz)
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
    sys=mdlOutputs(t,x,input,mass,g,Ix,Iy,Iz);

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
sizes.NumOutputs     = 9;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

function sys=mdlOutputs(~,~,input,mass,g,Ix,Iy,Iz)

% u(1): 拉力 （当前时刻）
% u(2): 力矩 x （当前时刻）
% u(3): 力矩 y （当前时刻）
% u(4): 力矩 z  （当前时刻）

% u(5): 角速度u （机体系x轴） （当前时刻）
% u(6): 角速度v （机体系y轴） （当前时刻）
% u(7): 角速度w （机体系z轴） （当前时刻）

% u(8): 角速度p （机体系x轴） （当前时刻）
% u(9): 角速度q （机体系y轴） （当前时刻）
% u(10): 角速度r （机体系z轴） （当前时刻）

% u(11): 滚转角 （当前时刻）
% u(12): 俯仰角 （当前时刻）
% u(13): 偏航角 （当前时刻）

% sys(1): 加速度u_dot                              （机体系x）      （当前时刻）
% sys(2): 加速度v_dot                              （机体系y）      （当前时刻）
% sys(3): 加速度w_dot                              （机体系z）      （当前时刻）
% sys(4): 角加速度p_dot                            （机体坐标系x）  （当前时刻）
% sys(5): 角加速度q_dot                            （机体坐标系y）  （当前时刻）
% sys(6): 角加速度r_dot                            （机体坐标系z）  （当前时刻）
% sys(7): 滚转角速度phi_dot                        （NED坐标系x）   （当前时刻）
% sys(8): 俯仰角速度theta_dot                      （NED坐标系y）   （当前时刻）
% sys(9): 偏航角速度psi_dot                        （NED坐标系z）   （当前时刻）

u = input(5);
v = input(6);
w = input(7);

p = input(8);
q = input(9);
r = input(10);

phi = input(11);
theta = input(12);
psi = input(13);

% 机体系下的拉力
F_Lb = [0;...
        0;...
       -input(1)];
% F_Lb = [0;...
%         0;...
%        -mass*g];
a_Lb = F_Lb./mass;  

C_nb = [cos(phi)*cos(psi)     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(phi)*sin(psi)     sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta)           sin(phi)*cos(theta)                                cos(phi)*cos(theta)];

g_n = [0;...
       0;...
       g];

C_bn = C_nb';

% 机体系下的重力
a_wb = C_bn * g_n;   

a_couple = [w*q - v*r;...
            u*r - w*p;...
            v*p - u*q];
% a_couple = [0;...
%             0;...
%             0];
sol_acceleration = a_Lb + a_wb -a_couple;

% 加速度模型
sys(1) = sol_acceleration(1);                       % u_dot
sys(2) = sol_acceleration(2);                       % v_dot
sys(3) = sol_acceleration(3);                       % w_dot

% 输出角加速度（机体坐标系）
% 角加速度线性模型如下
I = [Ix 0 0;...
     0 Iy 0;...
     0 0 Iz];
I_inv = inv(I); 
tau_total = [input(2)-(Iz-Iy)*q*r;...
             input(3)-(Ix-Iz)*p*r;...
             input(4)-(Iy-Ix)*p*q];
         
% tau_total = [input(2);...
%              input(3);...
%              input(4)];

sol_angular_acceleration = I_inv * tau_total;

sys(4) = sol_angular_acceleration(1);              % p_dot
sys(5) = sol_angular_acceleration(2);              % q_dot
sys(6) = sol_angular_acceleration(3);              % r_dot

% Matrix F, angular acceleration to euler attitude velocity
F_aa_to_av = [1     sin(phi)*tan(theta)     cos(phi)*tan(theta);...
              0     cos(phi)                -sin(phi);...
              0     sin(phi)/cos(theta)     cos(phi)/cos(theta)];
% F_aa_to_av = eye(3);
sol_attitude_velocity = F_aa_to_av * [p;q;r];

sys(7) = sol_attitude_velocity(1);                % phi_dot
sys(8) = sol_attitude_velocity(2);                % theta_dot
sys(9) = sol_attitude_velocity(3);                % psi_dot


% end mdlOutputs    
