%% HW4.2.1. Find a set of joint variables that would produce a given tool pose for a robot with fewer than 6 joints
%{
clear all;
clc;
%% parameters of the robot
M = [0.00000000 1.00000000 0.00000000 0.00000000; -1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 1.00000000 -8.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [-1.00000000 0.00000000 -1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 -1.00000000; 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 -1.00000000 0.00000000 -8.00000000; -2.00000000 0.00000000 6.00000000 0.00000000; 0.00000000 0.00000000 2.00000000 0.00000000];

s1 = S(1:6, 1);
s2 = S(1:6, 2);
s3 = S(1:6, 3);
s4 = S(1:6, 4);

% the desired final pose of the robot is
T_1in0 = [0.00000000 0.97944211 -0.20172545 0.41820604; -0.53948774 -0.16985151 -0.82468385 6.51937614; -0.84199346 0.10882841 0.52839701 -6.50929080; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = -5+(5+5)*rand(4,1);
done = 0;
tol = .0001;
i=0;
while (done == 0)
    % calculate space Jacobian
    j1 = s1;

    x1 = expm(vec2mat(s1)*theta(1));
    j2 = matrixAdjoint(x1)*s2;

    x2 = x1*expm(vec2mat(s2)*theta(2));
    j3 = matrixAdjoint(x2)*s3;

    x3 = x2*expm(vec2mat(s3)*theta(3));
    j4 = matrixAdjoint(x3)*s4;

    Js = [j1 j2 j3 j4];

    % calculate the current pose of the tool frame using theta
    T_curr = expm(vec2mat(s1)*theta(1))*expm(vec2mat(s2)*theta(2))*expm(vec2mat(s3)*theta(3))*expm(vec2mat(s4)*theta(4))*M;

    % calculate the change in pose for one timestep
    dT = T_1in0*inv(T_curr);
    
    % calculate the spatial twist to achieve this change in pose
    V = mat2twist(logm(dT));
    
    if norm(V) < tol
        done = 1;

    else
        dtheta = pinv(Js)*V;    
        theta = theta+dtheta;
        i=i+1;
    end

end
T_curr
T_1in0
mat2str(theta,8)
%}
%% HW4.2.2. Find a set of joint variables that would produce a given tool pose for a robot with more than 6 joints

clear all;
clc;
%% parameters of the robot
M = [0.00000000 -1.00000000 0.00000000 0.00000000; 1.00000000 0.00000000 0.00000000 2.00000000; 0.00000000 0.00000000 1.00000000 2.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [1.00000000 0.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 1.00000000 1.00000000 1.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 -6.00000000 -4.00000000 -2.00000000; 0.00000000 0.00000000 1.00000000 2.00000000 0.00000000 0.00000000 0.00000000; 2.00000000 1.00000000 0.00000000 2.00000000 0.00000000 0.00000000 0.00000000];

s1 = S(1:6, 1);
s2 = S(1:6, 2);
s3 = S(1:6, 3);
s4 = S(1:6, 4);
s5 = S(1:6, 5);
s6 = S(1:6, 6);
s7 = S(1:6, 7);

% the desired final pose of the robot is
T_1in0 = [0.00000000 -0.03904114 0.99923760 -3.29413132; 0.19629864 0.97979659 0.03828157 1.99559693; -0.98054416 0.19614898 0.00766372 -3.52316598; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = -5+(5+5)*rand(7,1);
done = 0;
tol = .0001;
i=0;
while (done == 0)
    % calculate space Jacobian
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
    
    x6 = x5*expm(vec2mat(s6)*theta(6));
    j7 = matrixAdjoint(x6)*s7;
       
    Js = [j1 j2 j3 j4 j5 j6 j7];

    % calculate the current pose of the tool frame using theta
    T_curr = expm(vec2mat(s1)*theta(1))*expm(vec2mat(s2)*theta(2))*expm(vec2mat(s3)*theta(3))*expm(vec2mat(s4)*theta(4))*expm(vec2mat(s5)*theta(5))*expm(vec2mat(s6)*theta(6))*expm(vec2mat(s7)*theta(7))*M;

    % calculate the change in pose for one timestep
    dT = T_1in0*inv(T_curr);
    
    % calculate the spatial twist to achieve this change in pose
    V = mat2twist(logm(dT));
    
    if norm(V) < tol
        done = 1;

    else
        dtheta = pinv(Js)*V;    
        theta = theta+dtheta;
        i=i+1;
    end

end
T_curr
T_1in0
mat2str(theta,8)




