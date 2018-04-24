%% Question 1: Find spatial twist given body twist
%{
clear all;
clc;

V_01in1 = [0.84628167; 0.59566026; 0.79063262; -0.12497523; -0.75427376; 0.33328375];
T = [0.91172524 0.40210928 -0.08405480 -0.52947425; 0.21989449 -0.65053606 -0.72694515 0.73354192; -0.34699207 0.64429106 -0.68153176 0.38543261; 0.00000000 0.00000000 0.00000000 1.00000000];

%find V_01in0
V_01in0 = matAdjoint(T)*V_01in1;

mat2str(V_01in0)
%}
%% Question 2: Find the matrix representation of a twist
%{
clear all;
clc;
V = [-4; 5; 1; -5; 2; 1];

V_mat = twist2mat(V);
mat2str(V_mat)
%}
%% Question 3: Find the pose of the tool frame for a robot with several joints
%{
clear all;
clc;

theta = [-0.30 -0.48 0.23];
M = [1 0 0 0; 0 0 1 2; 0 -1 0 -6; 0 0 0 1];

%revolute
a1 = [1; 0; 0];
q1 = [0; 0; -2];
S1 = [a1; -vec2skew(a1)*q1];

%revolute
a2 = [1; 0; 0];
q2 = [0; 0; -4];
S2 = [a2; -vec2skew(a2)*q2];

%revolute
a3 = [0; 0; -1];
q3 = [0; 2; -4];
S3 = [a3; -vec2skew(a3)*q3];

T_1in0 = expm(twist2mat(S1)*theta(1))*expm(twist2mat(S2)*theta(2))*expm(twist2mat(S3)*theta(3))*M;

mat2str(T_1in0, 6)
%}

%% Question 4: Derive the forward kinematics of a robot with several joints
%{
clear all;
clc;

M = [-1 0 0 -2; 0 1 0 8; 0 0 -1 0; 0 0 0 1];

%revolute
a1 = [0; 0; 1];
q1 = [2; 2; 0];
S1 = [a1; -vec2skew(a1)*q1];

%revolute
a2 = [0; 1; 0];
q2 = [0; 2; 0];
S2 = [a2; -vec2skew(a2)*q2];

%revolute
a3 = [0; 1; 0];
q3 = [0; 4; 0];
S3 = [a3; -vec2skew(a3)*q3];

%revolute
a4 = [1; 0; 0];
q4 = [0; 6; 0];
S4 = [a4; -vec2skew(a4)*q4];

%revolute
a5 = [-1; 0; 0];
q5 = [0; 8; 0];
S5 = [a5; -vec2skew(a5)*q5];

S = [S1 S2 S3 S4 S5];
mat2str(M)
mat2str(S)
%}

%% Question 5: Find the body pose as a function of one joint variable
clear all;
clc;
M = [-sqrt(3)/2 0 -1/2 -6; 1/2 0 -sqrt(3)/2 1; 0 -1 0 0; 0 0 0 1];

%revolute
a_in0 = [-sqrt(2)/2; sqrt(2)/2; 0];
q_in0 = [-7; -5; 0];

a_in1 = (inv(M)*[a_in0; 0])(1:3);
q_in1 = (inv(M)*[q_in0; 1])(1:3);


B = [a_in1; -vec2skew(a_in1)*q_in1];
mat2str(M)
mat2str(B)



%% Question 6: Derive the forward kinematics of a 1-DOF robot
%{
clear all;
clc;

M = [0 0 1 2; 0 1 0 0; -1 0 0 2; 0 0 0 1];

% body prismatic
a1 = [0; 0; 1];
B1 = [zeros(3,1); a1];
mat2str(M)
mat2str(B1)
%}


%% Question 7: Find the adjoint of a Homogeneous transformation matrix
%{
clear all;
clc;

T = [-0.02142874 0.77767265 0.62830411 0.49687943; 0.23716562 -0.60655587 0.75884283 0.85992026; 0.97123287 0.16527318 -0.17143949 0.09561473; 0.00000000 0.00000000 0.00000000 1.00000000];
mat2str(matAdjoint(T), 4)
%}
















