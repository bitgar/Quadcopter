close all
clear all
clc


syms p q r Mx My Mz Ix Iy Iz real

x = [p; q; r];
u = [Mx; My; Mz];

f = [ (Ix - Iz)*q*r / Ix;
      (Iz - Ix)*r*p / Iy;
      (Ix - Iy)*p*q / Iz ];

B = diag([1/Ix, 1/Iy, 1/Iz]);

xdot = f + B*u;

A_sym = jacobian(xdot, x)   % A(x)
B_sym = jacobian(xdot, u)   % should equal B

% 线性化点（平衡点）：p=q=r=0
A_lin = subs(A_sym, {p,q,r}, {0,0,0})
B_lin = subs(B_sym, {p,q,r}, {0,0,0})