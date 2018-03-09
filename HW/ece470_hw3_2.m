%% HW3.2.1. Derive the space Jacobian of a robot with several joints
%{
clear all;
clc;
%{
% prismatic joint
a = [0; 0; 0];
S = [zeros(3,1); a];

% revolute joint
a = [0; 0; 0];
q = [0; 0; 0];
S = [a; -axis2skew(a)*q];
%}

% 1 revolute joint
a1 = [0; 0; 1];
q1 = [2; 0; 0];
S1 = [a1; -axis2skew(a1)*q1];

% 2 revolute joint
a2 = [1; 0; 0];
q2 = [2; -2; 0];
S2 = [a2; -axis2skew(a2)*q2];

% 3 prismatic joint
a3 = [0; 0; 1];
S3 = [zeros(3,1); a3];

theta = [0.31; -0.68; 0.60];
J1 = S1;

x1 = expm(vec2mat(S1)*theta(1));
J2 = matrixAdjoint(x1)*S2;

x2 = x1*expm(vec2mat(S2)*theta(2));
J3 = matrixAdjoint(x2)*S3;

J = [J1 J2 J3];
mat2str(J, 6)
%}

%% HW3.2.2. Derive the spatial twist of a robot with several joints
%{
clear all;
clc;
%{
% prismatic joint
a = [0; 1; 0];
s = [zeros(3,1); a];

% revolute joint
a = [-1; 0; 0];
q = [-2; 0; 2];
s = [a; -axis2skew(a)*q];
%}

% 1 prismatic joint
a1 = [-1; 0; 0];
S1 = [zeros(3,1); a1];

% 2 prismatic joint
a2 = [0; 0; 1];
S2 = [zeros(3,1); a2];

% 3 prismatic joint
a3 = [-1; 0; 0];
S3 = [zeros(3,1); a3];

theta = [0.32; -0.47; 0.40];
thetadot = [0.56; 0.55; -0.50];

J1 = S1;

x1 = expm(vec2mat(S1)*theta(1));
J2 = matrixAdjoint(x1)*S2;

x2 = x1*expm(vec2mat(S2)*theta(2));
J3 = matrixAdjoint(x2)*S3;

J = [J1 J2 J3];

V = J*thetadot;
mat2str(V,6)
%}

%% HW3.2.3. Derive the body Jacobian of a robot with several joints
%{
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

% 1 revolute joint
a1 = [1; 0; 0];
q1 = [0; 2; 0];
s1 = [a1; -axis2skew(a1)*q1];

% 2 revolute joint
a2 = [0; 1; 0];
q2 = [2; 2; 0];
s2 = [a2; -axis2skew(a2)*q2];

% 3 prismatic joint
a3 = [-1; 0; 0];
s3 = [zeros(3,1); a3];

% rotation of each joint
theta = [0.30; 0.86; 0.39];

% finding the columns of the space jacobian
% calculate the spatial Jacobian
j1 = s1;

x1 = expm(vec2mat(s1)*theta(1));
j2 = matrixAdjoint(x1)*s2;

x2 = x1*expm(vec2mat(s2)*theta(2));
j3 = matrixAdjoint(x2)*s3;

J_in0 = [j1 j2 j3];

% initial pose in frame 0
x1 = [-1; 0; 0; 0];
y1 = [0; -1; 0; 0];
z1 = [0; 0; 1; 0];
p = [0; 4; 0; 1];
M = [x1 y1 z1 p];

% pose after the joints move
T_0in1 = expm(vec2mat(s1)*theta(1))*expm(vec2mat(s2)*theta(2))*expm(vec2mat(s3)*theta(3))*M;

X = matrixAdjoint((T_0in1^-1));

% transform from the space jacobian to the body jacobian
J_in1 = X*J_in0
mat2str(J_in1, 6)
%}

%% HW3.2.4. Derive the body twist of a robot with several joints

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

% 1 revolute joint
a1 = [0; 0; -1];
q1 = [-2; 0; 0];
s1 = [a1; -axis2skew(a1)*q1];

% 2 revolute joint
a2 = [0; 0; -1];
q2 = [-2; -2; 0];
s2 = [a2; -axis2skew(a2)*q2];

% 3 revolute joint
a3 = [-1; 0; 0];
q3 = [-2; -4; 0];
s3 = [a3; -axis2skew(a3)*q3];

% 4 revolute joint
a4 = [-1; 0; 0];
q4 = [-4; -4; 0];
s4 = [a4; -axis2skew(a4)*q4];

% 5 revolute joint
a5 = [-1; 0; 0];
q5 = [-6; -4; 0];
s5 = [a5; -axis2skew(a5)*q5];

% 6 revolute joint
a6 = [-1; 0; 0];
q6 = [-4; -8; 0];
s6 = [a6; -axis2skew(a6)*q6];

% rotation of each joint
theta = [-0.16; -0.54; 0.37; 0.57; -0.96; 0.51];
thetadot = [-0.47; -0.65; -0.48; -0.99; 0.17; -0.34];

% finding the columns of the space jacobian
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

J_in0 = [j1 j2 j3 j4 j5 j6];

% initial pose in frame 0
x1 = [0; 0; 1; 0];
y1 = [0; -1; 0; 0];
z1 = [-1; 0; 0; 0];
p = [-8; -6; 0; 1];
M = [x1 y1 z1 p];

% find the spatial twist
V_in0 = J_in0*thetadot;

% transform spatial twist to body twist
% pose after the joints move
T_0in1 = expm(vec2mat(s1)*theta(1))*expm(vec2mat(s2)*theta(2))*expm(vec2mat(s3)*theta(3))*expm(vec2mat(s4)*theta(4))*expm(vec2mat(s5)*theta(5))*expm(vec2mat(s6)*theta(6))*M;

F = matrixAdjoint(T_0in1);

V_in1 = F^-1*V_in0;
mat2str(V_in1,6)























