%% Question 4: Find a set of joint variables that would produce a given tool pose for a robot with fewer than 6 joints
clear all;
clc;

% characterize the robot by initial pose, joint screw axes, and desired
% final pose

T_1in0 = [1.00000000 0.00000000 0.00000000 0.24420662; 0.00000000 -0.82865755 -0.55975590 -2.06219638; 0.00000000 0.55975590 -0.82865755 -3.05285681; 0.00000000 0.00000000 0.00000000 1.00000000];
M = [1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 -1.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 -1.00000000 -4.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [-1.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000; 0.00000000 1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 -1.00000000];
s1 = S(:,1);
s2 = S(:,2);
s3 = S(:,3);

% guess initial theta
theta = 3*randn(3,1);
done = 0;
i = 0;
tol = .0001;
while done == 0
    % find current spatial jacobian
    j1 = s1;
    
    x1 = expm(skew4(s1)*theta(1));
    j2 = adjoint(x1)*s2;
    
    x2 = x1*expm(skew4(s2)*theta(2));
    j3 = adjoint(x2)*s3;
    J = [j1 j2 j3];
    
    % find current pose
    T_curr = expm(skew4(s1)*theta(1))*expm(skew4(s2)*theta(2))*expm(skew4(s3)*theta(3))*M;
        
    % find change in pose for one timestep
    dT = T_1in0*T_curr^-1;
    
    % calculate the spatial twist
    V_mat = logm(dT);
    
    V = unskew4(V_mat);
    
    if norm(V) < tol
        done = 1;
        norm(V)
    else
        dTheta = pinv(J)*V;
        theta = theta+dTheta;
        i = i+1
    end
end

mat2str(theta)























































































