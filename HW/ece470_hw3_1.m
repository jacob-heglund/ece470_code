%% HW3.1.1. Compute an infinite sum
%{
clear all;
clc;

M = [-3.00 -10.00 -3.00; -9.00 6.00 4.00; 1.00 2.00 4.00];
mat2str(expm(M), 6)
%}

%% HW3.1.2. Compute the derivative of the matrix exponential
%{
clear all;
clc;
S = [0.63; -0.33; -0.94; -0.09; 0.82; -0.52];
theta = -0.52;
thetadot = -0.15;

S_mat = vec2mat(S);

sol = S_mat*expm(S_mat*theta)*thetadot;

mat2str(sol, 6)
%}

%% HW3.1.4. Derive the space Jacobian of a 1-DOF robot
%{
clear all;
clc;

theta_1 = -0.58;

% prismatic joint
%a = [0; 1; 0];
%S = [zeros(3,1); a];

% revolute joint
a = [-1; 0; 0];
q = [-2; 0; 2];
S = [a; -axis2skew(a)*q];

mat2str(S)
%}
%% HW3.1.5. Derive the space Jacobian of a 2-DOF robot
%{
clear all;
clc;

theta_1 = -0.04;
theta_2 = -0.09;
%{
% prismatic joint
a = [0; 1; 0];
S = [zeros(3,1); a];

% revolute joint
a = [-1; 0; 0];
q = [-2; 0; 2];
S = [a; -axis2skew(a)*q];
%}

% prismatic joint
a1 = [0; 1; 0];
S1 = [zeros(3,1); a1];

% revolute joint
a2 = [1; 0; 0];
q2 = [0; 2; 0];
S2 = [a2; -axis2skew(a2)*q2];

X = expm(vec2mat(S1)*theta_1);

J = [S1 matrixAdjoint(X)*S2];
mat2str(J, 6)
%}
%% HW3.1.6. Derive the spatial twist of a 2-DOF robot
clear all;
clc;

x1 = [0; 0; -1];
y1 = [-1; 0; 0];
z1 = [0; 1; 0];
p = [-2; 0; 0];
M = [x1 y1 z1 p; 0 0 0 1];
%{
% prismatic
a = [0; 0; 0];
s = [zeros(3,1); a];

% revolute
a = [0; 0; 0];
q = [0; 0; 0];
s = [a; -vec2skew(a)*q];
%}
% revolute
a1 = [-1; 0; 0];
q1 = [0; -2; 0];
s1 = [a1; -vec2skew(a1)*q1];

% revolute
a2 = [-1; 0; 0];
q2 = [-2; -2; 0];
s2 = [a2; -vec2skew(a2)*q2];

theta_1 = 0.26;
theta_2 = -0.99;
thetadot_1 = -0.40;
thetadot_2 = 0.25;

% columns of the space jacobian
j1 = s1;

T1 = expm(vec2twist(s1)*theta_1);
j2 = matrixAdjoint(T1)*s2;

J = [j1 j2];
V = J*[thetadot_1; thetadot_2];
mat2str(V);


%% HW3.1.7. Derive the spatial twist of a 1-DOF robot
%{
clear all;
clc;
theta_1 = 0.62;
thetadot_1 = 0.28;

% prismatic joint
a = [1; 0; 0];
S = [zeros(3,1); a];

% revolute joint
%a = [1; 0; 0];
%q = [2; 0; 0];
%S = [a; -axis2skew(a)*q];

V = S*thetadot_1;
mat2str(V)
%}

























