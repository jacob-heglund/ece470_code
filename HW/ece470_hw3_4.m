%% HW3.4.1. Find joint velocities that would produce a given spatial twist
%{
clear all;
clc;
% specify the robot joints
%{
% prismatic joint
a = [0; 0; 0];
s = [zeros(3,1); a];

% revolute joint
a = [0; 0; 0];
q = [0; 0; 0];
s = [a; -axis2skew(a)*q];
%}
% z0 out
% z1 out

% revolute joint
a1 = [0; 1; 0];
q1 = [-2; 0; 0];
s1 = [a1; -axis2skew(a1)*q1];

% prismatic joint
a2 = [0; 0; 1];
s2 = [zeros(3,1); a2];

% revolute joint
a3 = [-1; 0; 0];
q3 = [-4; 4; 0];
s3 = [a3; -axis2skew(a3)*q3];

% revolute joint
a4 = [0; 0; 1];
q4 = [-6; 4; 0];
s4 = [a4; -axis2skew(a4)*q4];

% revolute joint
a5 = [0; 0; 1];
q5 = [-4; 2; 0];
s5 = [a5; -axis2skew(a5)*q5];

% revolute joint
a6 = [0; -1; 0];
q6 = [-6; 0; 0];
s6 = [a6; -axis2skew(a6)*q6];

% specific rotation
theta = [-0.80394991; -0.74563028; -0.56606165; 0.24069864; -0.95428911; -0.57208097];
V = [0.44934710; -0.09862944; 0.34752083; 3.42721644; -1.21372916; -3.99067455];

% calculate the spatial Jacobian
j1 = s1;

x1 = expm(vec2mat(s1)*theta(1));
j2 = matrixAdjoint(x1)*s2;

x2 = x1*expm(vec2mat(s2)*theta(2));
j3 = matrixAdjoint(x2)*s3;

x3 = x2*expm(vec2mat(s3)*theta(3));
j4 = matrixAdjoint(x3)*s4;

x4 = x3*expm(vec2mat(s4)*theta(4));
j5 = matrixAdjoint(x4)*s5;

x5 = x4*expm(vec2mat(s5)*theta(5));
j6 = matrixAdjoint(x5)*s6;

J = [j1 j2 j3 j4 j5 j6];
thetadot = inv(J)*V;

mat2str(thetadot, 6)
%}
%% HW3.4.2. Find joint velocities that would produce a given body twist
%{
clear all;
clc;
% specify the robot joints
%{
% prismatic joint
a = [0; 0; 0];
s = [zeros(3,1); a];

% revolute joint
a = [0; 0; 0];
q = [0; 0; 0];
s = [a; -axis2skew(a)*q];
%}
% x0 in
% z1 out
x1 = [0; 1; 0];
y1 = [0; 0; -1];
z1 = [-1; 0; 0];
p = [0; 0; -2];
M = [x1 y1 z1 p; 0 0 0 1];

% revolute joint
a1 = [0; 1; 0]; %good
q1 = [0; -2; 2]; %good
s1 = [a1; -axis2skew(a1)*q1];

% revolute joint
a2 = [0; 0; 1]; %good
q2 = [0; -2; 4]; %good
s2 = [a2; -axis2skew(a2)*q2];

% prismatic joint
a3 = [0; -1; 0]; %good
s3 = [zeros(3,1); a3];

% revolute joint
a4 = [0; -1; 0]; %good
q4 = [0; -4; 6]; %good
s4 = [a4; -axis2skew(a4)*q4];

% revolute joint
a5 = [0; 1; 0]; %good
q5 = [0; -4; 2]; %good
s5 = [a5; -axis2skew(a5)*q5];

% revolute joint
a6 = [0; 1; 0]; %good
q6 = [0; -2; -2]; %good
s6 = [a6; -axis2skew(a6)*q6];

% specific rotation
theta = [-0.47336761; 0.56364557; 0.50753197; 0.35139113; 0.50714681; -0.90677830];
Vb = [0.98182295; 0.29264846; 0.28183144; 0.27402646; -1.12904814; 0.00866865];

% calculate the spatial Jacobian
j1 = s1;

x1 = expm(vec2mat(s1)*theta(1));
j2 = matrixAdjoint(x1)*s2;

x2 = x1*expm(vec2mat(s2)*theta(2));
j3 = matrixAdjoint(x2)*s3;

x3 = x2*expm(vec2mat(s3)*theta(3));
j4 = matrixAdjoint(x3)*s4;

x4 = x3*expm(vec2mat(s4)*theta(4));
j5 = matrixAdjoint(x4)*s5;

x5 = x4*expm(vec2mat(s5)*theta(5));
j6 = matrixAdjoint(x5)*s6;

Js = [j1 j2 j3 j4 j5 j6];

T = expm(vec2mat(s1)*theta(1))*expm(vec2mat(s2)*theta(2))*expm(vec2mat(s3)*theta(3))*expm(vec2mat(s4)*theta(4))*expm(vec2mat(s5)*theta(5))*expm(vec2mat(s6)*theta(6))*M;
Vs = matrixAdjoint(T)*Vb;
Jb = (matrixAdjoint(T)^-1)*Js;

thetadot1 = inv(Js)*Vs
thetadot2 = inv(Jb)*Vb

mat2str(thetadot1, 6)
mat2str(thetadot2, 6)
%}

