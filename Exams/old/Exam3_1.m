%% Question 1: Compute the derivative of the matrix exponential
%{
clear all;
clc;

S = [0.40; 0.59; 0.35; -0.93; -0.17; 0.27];
theta = -0.92;
thetadot = 0.94;

solution = vec2twist(S)*thetadot*expm(vec2twist(S)*theta);

mat2str(solution,6)
%}


%% Question 3: Derive the body Jacobian of a robot with several joints
clear all;
clc;

%{
% prismatic
a = [0; 0; 0];
s = [zeros(3,1); a];

% revolute
a = [0;0;0];
q = [0;0;0];
s = [a; -vec2skew(a)*q];
%}

% prismatic
a1 = [-1; 0; 0];
s1 = [zeros(3,1); a1];

% revolute
a2 = [0;0;1];
q2 = [-8;0;2];
s2 = [a2; -vec2skew(a2)*q2];

% revolute
a3 = [0;0;1];
q3 = [-8;0;4];
s3 = [a3; -vec2skew(a3)*q3];


x1 = [1; 0; 0];
y1 = [0; 0; 1];
z1 = [0; -1; 0];
p = [-6; 0; 4];
M = [x1 y1 z1 p; 0 0 0 1];
theta = [-0.90; 0.09; -0.58];

% finding columns of the spatial jacobian
j1 = s1;

j1 = s1;


T1 = expm(vec2twist(s1)*theta(1));
j2 = matrixAdjoint(T1)*s2;

T2 = T1*expm(vec2twist(s2)*theta(2));
j3 = matrixAdjoint(T2)*s3;

J = [j1 j2 j3];

T_1in0 = expm(vec2twist(s1)*theta(1))*expm(vec2twist(s2)*theta(2))*expm(vec2twist(s3)*theta(3))*M;
J_body = matrixAdjoint(T_1in0)*J;

mat2str(J_body, 4)



%% Question 4: Derive the spatial twist of a 2-DOF robot
%{
clear all;
clc;

x1 = [0; -1; 0];
y1 = [0; 0; 1];
z1 = [-1; 0; 0];
p = [2; 0; 4];

M = [x1 y1 z1 p; 0 0 0 1];
%{
% prismatic
a = [0; 0; 0];
s = [zeros(3,1); a];

% revolute
a = [0;0;0];
q = [0;0;0];
s = [a; -vec2skew(a)*q];
%}

% prismatic
a1 = [0; -1; 0];
s1 = [zeros(3,1); a1];

% prismatic
a2 = [0; -1; 0];
s2 = [zeros(3,1); a2];

theta_1 = -0.75;
theta_2 = 0.22;
thetadot_1 = -0.49;
thetadot_2 = 0.55;
thetadot = [thetadot_1; thetadot_2];
% finding the columns of spatial Jacobian

j1 = s1;

T1 = expm(vec2twist(s1)*theta_1);
j2 = matrixAdjoint(T1)*s2;

J = [j1 j2];
V = J*thetadot;
mat2str(V)
%}













