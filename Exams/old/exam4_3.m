%% Question 1: Find the spatial twist that would produce a given change in pose
%{
clear all;
clc;

T_1in0 = [0.90262339 -0.20026549 0.38100492 -0.33789640; 0.42566676 0.54665056 -0.72109706 -0.13739165; -0.06386569 0.81306020 0.57866595 -1.08211295; 0.00000000 0.00000000 0.00000000 1.00000000];
T_2in0 = [0.99955082 0.01802180 0.02394504 0.16599944; -0.01768162 0.99974078 -0.01434343 -1.01449078; -0.02419733 0.01391360 0.99961037 -0.99591123; 0.00000000 0.00000000 0.00000000 1.00000000];

V_mat = logm(T_2in0*T_1in0^-1);
V_vec = unskew4(V_mat);

mat2str(V_vec)
%}
%% Question 2: Find a set of joint variables that would produce a given tool pose for a 6-DOF robot
%{
clear all;
clc;

% characterize the robot by finding initial pose and spatial screw axis
% z0 in
% z1 in
x1 = [0;-1;0];
y1 = [1;0;0];
z1 = [0;0;1];
p = [6;6;0];

M = [x1 y1 z1 p; 0 0 0 1];

%{
% prismatic
a = [0;0;0];
s = [zeros(3,1); a];

% revolute
a = [0;0;0];
q = [0;0;0];
s = [a; -skew3(a)*q];
%}
% prismatic
a1 = [0;1;0];
s1 = [zeros(3,1); a1];

% revolute
a2 = [1;0;0];
q2 = [0;2;0];
s2 = [a2; -skew3(a2)*q2];

% revolute
a3 = [0;0;-1];
q3 = [0;4;0];
s3 = [a3; -skew3(a3)*q3];

% revolute
a4 = [0;1;0];
q4 = [2;0;0];
s4 = [a4; -skew3(a4)*q4];

% revolute
a5 = [0;1;0];
q5 = [4;0;0];
s5 = [a5; -skew3(a5)*q5];

% prismatic
a6 = [1;0;0];
s6 = [zeros(3,1); a6];


% the desired final pose
T_1in0 = [0.23010121 0.52242732 0.82105002 3.82597163; -0.88715390 -0.23418519 0.39763708 5.97306348; 0.40001423 -0.81989450 0.40958714 -5.06634983; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = 5*rand(6,1);

done = 0;
tol = .0001;
i = 0;
while (done == 0)
    % find the current spatial jacobian
    j1 = s1;
    
    x1 = expm(skew4(s1)*theta(1));
    j2 = adjoint(x1)*s2;
    
    x2 = x1*expm(skew4(s2)*theta(2));
    j3 = adjoint(x2)*s3;
    
    x3 = x2*expm(skew4(s3)*theta(3));
    j4 = adjoint(x3)*s4;
    
    x4 = x3*expm(skew4(s4)*theta(4));
    j5 = adjoint(x4)*s5;
    
    x5 = x4*expm(skew4(s5)*theta(5));
    j6 = adjoint(x5)*s6;
    
    J = [j1 j2 j3 j4 j5 j6];
    
    % calculate current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*expm(skew4(s4)*theta(4))*expm(skew4(s5)*theta(5))*expm(skew4(s6)*theta(6))*M;
    
    % calculate change in pose for one timestep
    dT = T_1in0*T_curr^-1;
    
    % calculate the spatial twist to achieve this change in pose using logm
    V_mat = logm(dT);
    V = unskew4(V_mat);
    if norm(V) < tol
        done = 1;
    else
        dTheta = pinv(J)*V;
        theta = theta+ dTheta;
        i = i+1
    end
end

mat2str(theta)
%}
%% Question 3: Find a set of joint variables that would produce a given tool pose for a robot with more than 6 joints
%{
clear all;
clc;

% characterize the robot by initial pose and 
M = [0.00000000 1.00000000 0.00000000 2.00000000; 0.00000000 0.00000000 -1.00000000 -6.00000000; -1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [1.00000000 -1.00000000 0.00000000 -1.00000000 0.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000 1.00000000; 0.00000000 0.00000000 -1.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 -1.00000000 -1.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 6.00000000 8.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 -4.00000000 -4.00000000 0.00000000 0.00000000 1.00000000 0.00000000 0.00000000; 2.00000000 -4.00000000 2.00000000 -6.00000000 0.00000000 0.00000000 10.00000000 -1.00000000 0.00000000 -1.00000000 6.00000000];
s1 = S(:,1);
s2 = S(:,2);
s3 = S(:,3);
s4 = S(:,4);
s5 = S(:,5);
s6 = S(:,6);
s7 = S(:,7);
s8 = S(:,8);
s9 = S(:,9);
s10 = S(:,10);
s11 = S(:,11);


% the desired final pose
T_1in0 = [-0.74484899 0.66619161 0.03726558 -2.81781241; 0.01527965 0.07286671 -0.99722464 -6.56175586; -0.66705810 -0.74221236 -0.06445387 -5.02962666; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = 5*rand(11,1);

done = 0;
tol = .0001;
i = 0;
while (done == 0)
    % find the current spatial jacobian
    j1 = s1;
    
    x1 = expm(skew4(s1)*theta(1));
    j2 = adjoint(x1)*s2;
    
    x2 = x1*expm(skew4(s2)*theta(2));
    j3 = adjoint(x2)*s3;
    
    x3 = x2*expm(skew4(s3)*theta(3));
    j4 = adjoint(x3)*s4;
    
    x4 = x3*expm(skew4(s4)*theta(4));
    j5 = adjoint(x4)*s5;
    
    x5 = x4*expm(skew4(s5)*theta(5));
    j6 = adjoint(x5)*s6;
    
    x6 = x5*expm(skew4(s6)*theta(6));
    j7 = adjoint(x6)*s7;
    
    x7 = x6*expm(skew4(s7)*theta(7));
    j8 = adjoint(x7)*s8;
    
    x8 = x7*expm(skew4(s8)*theta(8));
    j9 = adjoint(x8)*s9;
    
    x9 = x8*expm(skew4(s9)*theta(9));
    j10 = adjoint(x9)*s10;
    
    x10 = x9*expm(skew4(s10)*theta(10));
    j11 = adjoint(x10)*s11;
      
    
    J = [j1 j2 j3 j4 j5 j6 j7 j8 j9 j10 j11];
    
    % calculate current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*expm(skew4(s4)*theta(4))*expm(skew4(s5)*theta(5))*expm(skew4(s6)*theta(6))*expm(skew4(s7)*theta(7))*expm(skew4(s8)*theta(8))*expm(skew4(s9)*theta(9))*expm(skew4(s10)*theta(10))*expm(skew4(s11)*theta(11))*M;
    
    % calculate change in pose for one timestep
    dT = T_1in0*T_curr^-1;
    
    % calculate the spatial twist to achieve this change in pose using logm
    V_mat = logm(dT);
    V = unskew4(V_mat);
    if norm(V) < tol
        done = 1;
    else
        dTheta = pinv(J)*V;
        theta = theta+ dTheta;
        i = i+1
    end
end

mat2str(theta)
%}
%% Question 4: Find a set of joint variables that would produce a given tool pose for a robot with fewer than 6 joints

clear all;
clc;

% characterize the robot by initial pose and 
M = [0.00000000 0.00000000 1.00000000 -4.00000000; 0.00000000 1.00000000 0.00000000 0.00000000; -1.00000000 0.00000000 0.00000000 -2.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [0.00000000 0.00000000 0.00000000; -1.00000000 -1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000; 2.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000; 2.00000000 2.00000000 -1.00000000];
s1 = S(:,1);
s2 = S(:,2);
s3 = S(:,3);

% the desired final pose
T_1in0 = [0.99866667 0.00000000 0.05162254 1.00497068; 0.00000000 1.00000000 0.00000000 0.00000000; -0.05162254 0.00000000 0.99866667 -1.83884328; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = 5*rand(3,1);

done = 0;
tol = .0001;
i = 0;
while (done == 0)
    % find the current spatial jacobian
    j1 = s1;
    
    x1 = expm(skew4(s1)*theta(1));
    j2 = adjoint(x1)*s2;
    
    x2 = x1*expm(skew4(s2)*theta(2));
    j3 = adjoint(x2)*s3;
    
      
    
    J = [j1 j2 j3];
    
    % calculate current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*M;
    
    % calculate change in pose for one timestep
    dT = T_1in0*T_curr^-1;
    
    % calculate the spatial twist to achieve this change in pose using logm
    V_mat = logm(dT);
    V = unskew4(V_mat);
    if norm(V) < tol
        done = 1;
    else
        dTheta = pinv(J)*V;
        theta = theta+ dTheta;
        i = i+1
    end
end

mat2str(theta)










































