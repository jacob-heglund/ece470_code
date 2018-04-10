%% Question 1: Find a set of joint variables that would produce a given tool pose for a robot with fewer than 6 joints
clear all;
clc;

% characterize the robot with initial pose, screw axes, and final desired
% pose
T_1in0 = [-0.42098729 0.74639888 -0.51542061 -6.91880083; 0.61682592 0.65219198 0.44064884 -1.10370550; 0.66505299 -0.13241724 -0.73496271 1.81896647; 0.00000000 0.00000000 0.00000000 1.00000000];
M = [0.00000000 1.00000000 0.00000000 -4.00000000; 1.00000000 0.00000000 0.00000000 2.00000000; 0.00000000 0.00000000 -1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
S = [0.00000000 0.00000000 -1.00000000 1.00000000; -1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 -1.00000000 0.00000000 0.00000000; 0.00000000 4.00000000 0.00000000 0.00000000; 0.00000000 -2.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 -2.00000000 0.00000000];

% guess an initial theta
theta = 2+2*randn(4,1);
done = 0;
tol = .001;
i = 0;

% start the loop
while (done == 0)
    % find current spatial jacobian
    j1 = S(:,1);
    
    x1 = expm(skew4(S(:,1))*theta(1));
    j2 = adjoint(x1)*S(:,2);
    
    x2 = x1*expm(skew4(S(:,2))*theta(2));
    j3 = adjoint(x2)*S(:,3);
    
    x3 = x2*expm(skew4(S(:,3))*theta(3));
    j4 = adjoint(x3)*S(:,4);
    
    J = [j1 j2 j3 j4];
    
    % find the current pose
    T_curr = expm(skew4(S(:,1))*theta(1))*expm(skew4(S(:,2))*theta(2))*expm(skew4(S(:,3))*theta(3))*expm(skew4(S(:,4))*theta(4))*M;
    
    % find the change in pose required to take a step toward the desired
    % pose
    dT = T_1in0*T_curr^-1;
    
    % find the associated spatial twist
    V_mat = logm(dT);
    V = unskew4(V_mat);
    
    if norm(V) < tol
        done = 1;
    else
        %dTheta = pinv(J)*V;
        dTheta = inv(J'*J)*J'*V;
        theta = theta+dTheta;
        i = i+1
    end
end

norm(V)
i
mat2str(theta)



%% Question 3: Find the spatial twist that would produce a given change in pose
%{
clear all;
clc;


T_1in0 = [0.80963199 -0.58664062 -0.01867659 0.83051296; 0.56947018 0.79284425 -0.21702929 -0.80912008; 0.14212582 0.16507809 0.97598641 0.44595899; 0.00000000 0.00000000 0.00000000 1.00000000];
T_2in0 = [0.99721820 0.05215277 0.05325359 1.44802110; 0.03012430 0.37151503 -0.92793810 -0.53843526; -0.06817905 0.92696099 0.36891048 1.43172343; 0.00000000 0.00000000 0.00000000 1.00000000];

dT = T_2in0*T_1in0^-1;

Vmat = logm(dT);

V = unskew4(Vmat);

mat2str(V,6)
%}













































































