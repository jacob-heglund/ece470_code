%% HW2.3.1. Derive the forward kinematics of a 1-DOF robot
%{
clear all;
clc;
%{
%for a body screw
M_1in0 = [0 -1 0 0; 0 0 1 2; -1 0 0 -2; 0 0 0 1];
a_in1 = [0; -1; 0];
q_in1 = [0; 0; 0];

B = [a_in1; -axis2skew(a_in1)*q_in1];

mat2str(M_1in0, 6)
mat2str(B,6)
%}

%for a spatial screw
M = [0 -1 0 0; 0 0 1 2; -1 0 0 -2; 0 0 0 1];
a = [0; 0; -1];
q = [2; 0; 0];
%revolute
%S = [a; -axis2skew(a)*q];
%mat2str(M)
%mat2str(S)

%prismatic
S = [zeros(3,1); a];
mat2str(M)
mat2str(S)
%}
%% HW2.3.2. Derive the forward kinematics of a robot with several joints

clear all;
clc;
%{
% prismatic joint
a = [0; 0; 0];
s = [zeros(3,1); a];

% revolute joint
a = [0; 0; 0];
q = [0; 0; 0];
s = [a; -axis2skew(a)*q];
%}

M = [0 0 1 0; 1 0 0 8; 0 1 0 -2; 0 0 0 1];

% revolute joint
a1 = [1; 0; 0];
q1 = [-6; 2; 0];
s1 = [a1; -axis2skew(a1)*q1];

% revolute joint
a2 = [0; -1; 0];
q2 = [-2; 2; 0];
s2 = [a2; -axis2skew(a2)*q2];

% revolute joint
a3 = [0; 1; 0];
q3 = [0; -2; 0];
s3 = [a3; -axis2skew(a3)*q3];

S = [s1 s2 s3];
mat2str(M)
mat2str(S)

%% HW2.3.3. Find the pose of the tool frame for a robot with one joint
%{
clear all;
clc;
theta = -0.19;
M = [0 0 1 -2; 0 -1 0 -2; 1 0 0 0; 0 0 0 1];
a = [0; 0; 1];
q = [-2; 0; 0];
S = [a; -axis2skew(a)*q];

T = expm(vec2mat(S)*theta)*M;
mat2str(T, 6)
%}
%end of line




































