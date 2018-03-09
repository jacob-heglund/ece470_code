%% HW2.2.1. Find the body pose as a function of one joint variable
clear all;
clc;

%for a body screw
%{
M_1in0 = [sqrt(2)/2 sqrt(2)/2 0 -4; 0 0 1 0; sqrt(2)/2 -sqrt(2)/2 0 -3; 0 0 0 1];

a_in0 = [sqrt(3)/2; 0; 1/2];
q_in0 = [-4; 0; -12];

M_0in1 = inv(M_1in0);

a_in1 = (M_0in1*[a_in0; 0]);
q_in1 = (M_0in1*[q_in0; 1]);

a_in1 = a_in1(1:3);
q_in1 = q_in1(1:3);


% revolute
B = [a_in1; -axis2skew(a_in1)*q_in1];

%prismatic
%B = [zeros(3,1); a_in1];

mat2str(M_1in0, 6)
mat2str(B,6)
%}

%for a spatial screw
M_1in0 = [1/2 sqrt(3)/2 0 -4; 0 0 -1 0; -sqrt(3)/2 1/2 0 3; 0 0 0 1];
a_in0 = [0; 0; -1];
q_in0 = [-2; 0; -4];

% revolute
S = [a_in0; -axis2skew(a_in0)*q_in0];

%prismatic
%S = [zeros(3,1); a_in0];

mat2str(M_1in0, 6)
mat2str(S, 6)

















































