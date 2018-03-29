%% HW4.1.5. Find a set of joint variables that would produce a given tool pose for a 6-DOF robot
clear all;
clc;
%% parameters of the robot
M = [0.00000000 0.00000000 -1.00000000 0.00000000; -1.00000000 0.00000000 0.00000000 6.00000000; 0.00000000 1.00000000 0.00000000 -6.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [0.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 1.00000000 0.00000000; -1.00000000 -1.00000000 0.00000000 -1.00000000 0.00000000 1.00000000; 0.00000000 -2.00000000 0.00000000 -4.00000000 8.00000000 6.00000000; 0.00000000 0.00000000 -6.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 -2.00000000 0.00000000 0.00000000 0.00000000];

s1 = S(1:6, 1);
s2 = S(1:6, 2);
s3 = S(1:6, 3);
s4 = S(1:6, 4);
s5 = S(1:6, 5);
s6 = S(1:6, 6);

% the desired final pose of the robot is
T_1in0 = [-0.46901061 -0.85019775 -0.23915024 3.20952960; -0.65313577 0.15161194 0.74190800 3.19703630; -0.59451048 0.50416030 -0.62640217 -3.39726155; 0.00000000 0.00000000 0.00000000 1.00000000];

% guess an initial theta
theta = -5+(5+5)*rand(6,1)
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
    Js = [j1 j2 j3 j4 j5 j6];

    % calculate the current pose of the tool frame using theta
    T_curr = expm(vec2mat(s1)*theta(1))*expm(vec2mat(s2)*theta(2))*expm(vec2mat(s3)*theta(3))*expm(vec2mat(s4)*theta(4))*expm(vec2mat(s5)*theta(5))*expm(vec2mat(s6)*theta(6))*M;

    % calculate the change in pose for one timestep
    dT = T_1in0*inv(T_curr);
    
    % calculate the spatial twist to achieve this change in pose
    V = mat2twist(logm(dT));
    
    if norm(V) < tol
        done = 1;

    else
        dtheta = inv(Js)*V;    
        theta = theta+dtheta;
        i=i+1;
    end

end
T_curr
T_1in0
mat2str(theta,8)

