%% HW3.4.1. Find joint velocities that would produce a given spatial twist
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
% revolute joint
a1 = [0; 0; 1];
q1 = [0; 2; 0];
s1 = [a1; -axis2skew(a1)*q1];

% revolute joint
a2 = [-1; 0; 0];
q2 = [0; 4; 0];
s2 = [a2; -axis2skew(a2)*q2];

% revolute joint
a3 = [0; 0; 1];
q3 = [-2; 4; 0];
s3 = [a3; -axis2skew(a3)*q3];

% revolute joint
a4 = [0; 1; 0];
q4 = [-2; 6; 0];
s4 = [a4; -axis2skew(a4)*q4];

% prismatic joint
a5 = [0; -1; 0];
s5 = [zeros(3,1); a5];

% revolute joint
a6 = [0; -1; 0];
q6 = [-4; 4; 0];
s6 = [a6; -axis2skew(a6)*q6];

% specific rotation
theta = [0.34796985; 0.67377304; 0.03854045; 0.80432731; -0.09498409; 0.55065031];
V = [-0.56823857; 0.20965901; 0.00241019; 1.01018188; 0.59194061; 0.32831872];

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

s = [s1 s2 s3 s4 s5]
J = [j1 j2 j3 j4 j5 j6];
thetadot = pinv(J)*V;

mat2str(thetadot, 6);


right = [0.0537314551844; 0.374722176083; -0.772905127968; 0.979514125789; 0.18792986482; -0.703887170208];































