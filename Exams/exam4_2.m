%% Question 1: Find a set of joint variables that would produce a given tool pose for a robot with more than 6 joints

%% characterize the robot
M = [-1.00000000 0.00000000 0.00000000 -10.00000000; 0.00000000 0.00000000 -1.00000000 -6.00000000; 0.00000000 -1.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [-1.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 -1.00000000 1.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 1.00000000 0.00000000; -2.00000000 1.00000000 6.00000000 4.00000000 -6.00000000 0.00000000 1.00000000];
s1 = S(:,1);
s2 = S(:,2);
s3 = S(:,3);
s4 = S(:,4);
s5 = S(:,5);
s6 = S(:,6);
s7 = S(:,7);

% desired final pose
T_1in0 = [-0.87844205 0.47784889 0.00000000 -9.72719940; -0.23720209 -0.43605478 -0.86809642 -6.18217291; -0.41481891 -0.76257240 0.49639561 0.59434636; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = 5*rand(7,1);

done = 0;
tol = .0001;
i = 0;
while (done == 0)
    % find current space jacobian
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
    
    J = [j1 j2 j3 j4 j5 j6 j7];
    
    %% calculate current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*expm(skew4(s4)*theta(4))*expm(skew4(s5)*theta(5))*expm(skew4(s6)*theta(6))*expm(skew4(s7)*theta(7))*M;
    
    % calculate change in pose for one timestep
    dT = T_1in0*inv(T_curr);
    
    % calculate the spatial twist to achive this change in pose
    V = unskew4(logm(dT));
    
    if norm(V) < tol
        done = 1;
    else
        dtheta = pinv(J)*V;
        theta = theta + dtheta
        i = i+1
    end
end

mat2str(theta)



%% Question 2: Find a set of joint variables that would produce a given tool pose for a 6-DOF robot
%{
clear all;
clc;
%% characterize the robot
% y0 out
% z1 in
x1 = [1;0;0];
y1 = [0;0;1];
z1 = [0; -1; 0];
p = [-4;0;2];
% initial pose
M = [x1 y1 z1 p; 0 0 0 1];

% characterize the robot
%{
% prismatic
a = [0;0;0];
s = [zeros(3,1); a];

% revolute
a = [0;0;0];
q = [0;0;0];
s = [a; -skew3(a)*q];
%}

% revolute
a1 = [0;1;0];
q1 = [-2;0;0];
s1 = [a1; -skew3(a1)*q1];

% prismatic
a2 = [0;1;0];
s2 = [zeros(3,1); a2];

% revolute
a3 = [0;0;1];
q3 = [-4;0;0];
s3 = [a3; -skew3(a3)*q3];

% prismatic
a4 = [-1;0;0];
s4 = [zeros(3,1); a4];

% revolute
a5 = [0;1;0];
q5 = [-6;0;-2];
s5 = [a5; -skew3(a5)*q5];

% revolute
a6 = [1;0;0];
q6 = [0;0;0];
s6 = [a6; -skew3(a6)*q6];

S = [s1 s2 s3 s4 s5 s6];

% desired final pose
T_1in0 = [0.81726325 -0.04535736 -0.57447670 -2.91308115; -0.57506991 -0.00003672 -0.81810427 0.29299900; 0.03708595 0.99897082 -0.02611366 0.66676010; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = 5*rand(6,1);

done = 0;
tol = .0001;
i = 0;
while (done == 0)
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
    
    %% calculate current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*expm(skew4(s4)*theta(4))*expm(skew4(s5)*theta(5))*expm(skew4(s6)*theta(6))*M;
    
    % calculate change in pose for one timestep
    dT = T_1in0*inv(T_curr);
    
    % calculate the spatial twist to achive this change in pose
    V = unskew4(logm(dT));
    
    if norm(V) < tol
        done = 1;
    else
        dtheta = pinv(J)*V;
        theta = theta + dtheta
        i = i+1
    end
end

mat2str(theta)
%}
%% Question 3: Find the spatial twist that would produce a given change in pose
%{
clear all;
clc;
T_1in0 = [0.45563270 0.62275085 -0.63606621 0.28712580; -0.61774969 0.73568278 0.27777000 0.10925905; 0.64092446 0.26636861 0.71990527 -0.52694507; 0.00000000 0.00000000 0.00000000 1.00000000];
T_2in0 = [0.55839037 0.82957689 -0.00153975 -0.40729622; -0.78994643 0.53228000 0.30440539 -0.49928053; 0.25334726 -0.16876071 0.95254133 -0.04181366; 0.00000000 0.00000000 0.00000000 1.00000000];

V_mat = logm(T_2in0*T_1in0^-1);

V_vec = unskew4(V_mat);
mat2str(V_vec)
%}
%% Question 4: Find a set of joint variables that would produce a given tool pose for a robot with fewer than 6 joints
%{
%% characterize the robot
M = [0.00000000 1.00000000 0.00000000 -6.00000000; 1.00000000 0.00000000 0.00000000 8.00000000; 0.00000000 0.00000000 -1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [0.00000000 0.00000000 -1.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; -1.00000000 -1.00000000 6.00000000 -1.00000000 -1.00000000];

s1 = S(:,1);
s2 = S(:,2);
s3 = S(:,3);
s4 = S(:,4);
s5 = S(:,5);


% desired final pose
T_1in0 = [0.00000000 1.00000000 0.00000000 -6.00000000; 0.74283588 0.00000000 -0.66947357 7.25785827; -0.66947357 0.00000000 -0.74283588 -1.60513137; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = 5*rand(5,1);

done = 0;
tol = .0001;
i = 0;
while (done == 0)
    j1 = s1;
    
    x1 = expm(skew4(s1)*theta(1));
    j2 = adjoint(x1)*s2;
    
    x2 = x1*expm(skew4(s2)*theta(2));
    j3 = adjoint(x2)*s3;
    
    x3 = x2*expm(skew4(s3)*theta(3));
    j4 = adjoint(x3)*s4;
    
    x4 = x3*expm(skew4(s4)*theta(4));
    j5 = adjoint(x4)*s5;
        
    J = [j1 j2 j3 j4 j5];
    
    %% calculate current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*expm(skew4(s4)*theta(4))*expm(skew4(s5)*theta(5))*M;
    
    % calculate change in pose for one timestep
    dT = T_1in0*inv(T_curr);
    
    % calculate the spatial twist to achive this change in pose
    V = unskew4(logm(dT));
    
    if norm(V) < tol
        done = 1;
    else
        dtheta = pinv(J)*V;
        theta = theta + dtheta
        i = i+1
    end
end

mat2str(theta)
%}














































