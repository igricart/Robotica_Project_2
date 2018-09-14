%% Manip Project 2 

%All link lengths and offsets are measured in mm
clear L
%            theta    d                     a         alpha
L(1) = Link([0        0           0         pi/2        0  0], 'standard');
L(2) = Link([0        0           279.4e-3  0           0  0], 'standard');
L(3) = Link([0        0           0         -pi/2       0  0], 'standard');
L(4) = Link([0        228.6e-3    0         pi/2        0  0], 'standard');
L(5) = Link([0        0           0         -pi/2       0  0], 'standard');
L(6) = Link([0        0           0         0           0  0], 'standard');

manip=SerialLink(L, 'name', 'manip');
% manip
qz = [0 pi/2 -pi pi 0 0];

%max joint velocity = 3 rad/s
joint_maxVel = 3;

%Trajectories
wn = 2*pi/10;
syms t;
xd1 = 0.001.*[75*sin(wn*t) 0 75*cos(wn*t)]';
dx_xd1 = diff(xd1,t);

matlabFunction(xd1, 'File', 'xd1');
matlabFunction(dx_xd1, 'File', 'dx_xd1');

% K gain value
x_dot_zero = manip.jacob0(qz,'trans')*joint_maxVel*ones(6,1);
x_zero = transl(manip.fkine(qz))';
x_d_dot_zero = subs(dx_xd1,t,0);
x_d_zero = subs(xd1,t,0);


K_lim = (x_dot_zero - x_d_dot_zero)./(x_d_zero - x_zero);

%Control
