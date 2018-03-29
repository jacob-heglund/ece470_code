%% Question 1: Compute the derivative of the matrix exponential
%{
clear all;
clc;

S = [0.38; 0.92; -0.15; 0.63; -0.07; -0.11];
theta = 0.79;
thetadot = 0.14;

sMat = sVec2sMat(S);

sol = sMat*thetadot*expm(sMat*theta);
mat2str(sol,6)
%}
%% Question 2: Derive the body Jacobian of a robot with several joints
%{
clear all;
clc;

theta = [0.82; 0.66; 0.67];
% characterize the robot
%{
% revolute
a = [0;0;0];
q = [0;0;0];
s = [a; -vec2skew(a)*q];

% prismatic
a = [0;0;0];
s = [zeros(3,1); a];
%}
% x0 in
% y1 in
x1 = [0; -1; 0];
y1 = [1; 0; 0];
z1 = [0; 0; 1];
p = [0; -4; -4];
M = [x1 y1 z1 p; 0 0 0 1];

% revolute
a1 = [0;0;-1];%good
q1 = [0;0;-2];%good
s1 = [a1; -vec2skew(a1)*q1];

% prismatic
a2 = [0;0;-1];%good
s2 = [zeros(3,1); a2];

% revolute
a3 = [0;0;-1];%good
q3 = [0;-2;-4];%good
s3 = [a3; -vec2skew(a3)*q3];

% calculate the spatial jacobian
j1 = s1;

x1 = expm(sVec2sMat(s1)*theta(1));
j2 = matrixAdjoint(x1)*s2;

x2 = x1*expm(sVec2sMat(s2)*theta(2));
j3 = matrixAdjoint(x2)*s3;

Js = [j1 j2 j3];

T = expm(sVec2sMat(s1)*theta(1))*expm(sVec2sMat(s2)*theta(2))*expm(sVec2sMat(s3)*theta(3))*M;

Jb = matrixAdjoint(T)^-1*Js;
mat2str(Jb,6)
%}

%% Question 3: Derive the spatial twist of a 2-DOF robot
clear all;
clc;
%{
% y0 in
% x1 in
theta_1 = -0.77;
theta_2 = 0.57;
thetadot_1 = 0.81;
thetadot_2 = -0.11;

theta = [theta_1; theta_2];
thetadot = [thetadot_1; thetadot_2];

x1 = [0; 1; 0];%good
y1 = [1; 0; 0];%good
z1 = [0; 0; -1];%good
p = [-2; 0; -2];%good
M = [x1 y1 z1 p; 0 0 0 1];

% prismatic
a1 = [0;-1;0];%good
s1 = [zeros(3,1); a1];

% prismatic
a2 = [0;0;-1];%good
s2 = [zeros(3,1); a2];

% find the space jacobian
j1 = s1;

x1 = expm(sVec2sMat(s1)*theta(1));
j2 = matrixAdjoint(x1)*s2;

Js = [j1 j2];

Vs = Js*thetadot;
mat2str(Vs,6)
%}
%% Question 4: Find joint velocities that would produce a given spatial twist
clear all;
clc;

theta = [-0.53772408; 0.46866237; 0.35460082; -0.17741540; -0.17907633; -0.07825295];
Vs = [0.24691683; 0.92399213; -1.34774167; -7.30567390; 0.91070971; -1.20281404];

%{
% revolute
a = [0;0;0];
q = [0;0;0];
s = [a; -vec2skew(a)*q];

% prismatic
a = [0;0;0];
s = [zeros(3,1); a];
%}
% x0 out
% y1 in
x1 = [0; 1; 0];%good
y1 = [-1; 0; 0];%good
z1 = [0; 0; 1];%good
p = [0; 8; 2];%good
M = [x1 y1 z1 p; 0 0 0 1];

% prismatic
a1 = [0;0;1];%good
s1 = [zeros(3,1); a1];

% revolute
a2 = [0;0;1];%good
q2 = [0;2;2];%good
s2 = [a2; -vec2skew(a2)*q2];

% revolute
a3 = [0;1;0];%good
q3 = [0;2;4];%good
s3 = [a3; -vec2skew(a3)*q3];

% revolute
a4 = [1;0;0];%good
q4 = [0;4;4];%good
s4 = [a4; -vec2skew(a4)*q4];

% prismatic
a5 = [0;1;0];%good
s5 = [zeros(3,1); a5];

% revolute
a6 = [0;0;1];%good
q6 = [0;6;2];%good
s6 = [a6; -vec2skew(a6)*q6];

% calculate the spatial jacobian
j1 = s1;

x1 = expm(sVec2sMat(s1)*theta(1));
j2 = matrixAdjoint(x1)*s2;

x2 = x1*expm(sVec2sMat(s2)*theta(2));
j3 = matrixAdjoint(x2)*s3;

x3 = x2*expm(sVec2sMat(s3)*theta(3));
j4 = matrixAdjoint(x3)*s4;

x4 = x3*expm(sVec2sMat(s4)*theta(4));
j5 = matrixAdjoint(x4)*s5;

x5 = x4*expm(sVec2sMat(s5)*theta(5));
j6 = matrixAdjoint(x5)*s6;

Js = [j1 j2 j3 j4 j5 j6];

T = expm(sVec2sMat(s1)*theta(1))*expm(sVec2sMat(s2)*theta(2))*expm(sVec2sMat(s3)*theta(3))*expm(sVec2sMat(s4)*theta(4))*expm(sVec2sMat(s5)*theta(5))*expm(sVec2sMat(s6)*theta(6))*M;


% Vs = Js*thetadot\
thetadot = inv(Js)*Vs;
mat2str(thetadot,6)




















