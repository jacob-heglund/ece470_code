%% Question 1: Detect collision at several different configurations (2 joints)

clear all;
clc;

S = [0.00 0.00; 0.00 0.00; 0.00 0.00; 0.00 0.00; 1.00 0.00; 0.00 1.00];
M = [0.00 0.00 -1.00 0.00; 0.00 -1.00 0.00 2.00; -1.00 0.00 0.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 2.00 2.00 0.00; 0.00 0.00 2.00 2.00; 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [-7.75 -9.72 -0.69 -2.50 2.00 6.66 -8.55 4.16 -4.30 -8.68 8.64 8.21 -3.03 0.83 8.45 1.95 -4.51; 3.09 7.92 3.46 -6.16 -1.04 5.83 6.26 -1.85 1.91 5.30 -0.58 -7.97 2.50 4.89 5.89 -5.83 8.09; -6.88 8.87 -8.36 7.81 9.52 2.54 7.03 8.77 2.19 5.61 -9.35 -3.42 -3.54 3.98 0.04 -0.49 -1.06];
r_obstacle = [4.31 4.90 1.61 3.20 1.84 1.06 2.48 4.53 3.53 1.01 2.86 0.79 1.99 0.89 4.35 3.25 4.01];
theta = [-1.63 0.54 2.22 1.01 1.44 -1.85 2.34 1.95 -0.27 0.43 1.19 1.78 -1.60; 2.21 2.05 -0.38 1.15 1.94 2.19 -1.36 -0.52 1.19 1.91 -0.36 1.04 1.50];

numConfigs = size(theta,2);
numSpheres = size(r_robot,2);
numObstacles = size(p_obstacle,2);

collVec = zeros(1, numConfigs);

for i = 1:numConfigs
    collFlag = 0;
    pRobotFinal = moveSpheres(S, p_robot, theta(:,i));
    
    % check self collisions
    for j = 1:numSpheres
        for k = 1:numSpheres
            if (j==k);
                continue
            else
                collFlag = checkCollSphere(pRobotFinal(:,j), pRobotFinal(:,k), r_robot(j), r_robot(k));
                if collFlag == 1
                    collVec(i) = 1;
                    break
                end
            end
        end
    end
    % check obstacle collisions
    for j = 1:numSpheres
        for k = 1:numObstacles
            collFlag = checkCollSphere(pRobotFinal(:,j), p_obstacle(:,k), r_robot(j), r_obstacle(k));
            if collFlag == 1
                collVec(i) = 1;
                break
            end
        end
    end
end
mat2str(collVec)

%% Question 2: Detect collision along several straight-line paths (2 joints)
%{
clear all;
clc;
S = [0.00 0.00; -1.00 1.00; 0.00 0.00; 0.00 0.00; 0.00 0.00; 2.00 0.00];
M = [0.00 1.00 0.00 2.00; 1.00 0.00 0.00 -2.00; 0.00 0.00 -1.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 -2.00 0.00 2.00; 0.00 -2.00 -2.00 -2.00; 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [2.84 -4.07 2.13 -1.68 4.93 2.09 3.08 -2.17 4.61; 1.55 2.83 1.84 3.89 4.04 3.09 0.05 4.52 0.16; -3.53 4.17 -4.96 -0.26 -1.12 -3.33 4.84 1.41 -1.80];
r_obstacle = [0.85 3.56 2.71 2.96 2.21 1.42 0.60 1.04 2.77];
theta_start = [-0.58 1.04 1.60 -2.81 -1.53 -0.81 3.09 1.94 -0.61; -0.97 1.89 -1.62 -0.02 1.98 -0.23 1.13 -0.57 2.14];
theta_goal = [1.78 1.08 -1.17 -0.74 -0.53 1.86 -2.87 -1.07 0.91; -1.60 -1.53 0.83 0.93 0.35 -1.09 -2.16 -0.69 -1.51];

numConfigs = size(theta_start,2);
numSpheres = size(r_robot,2);
numObstacles = size(p_obstacle,2);

collVec = zeros(1, numConfigs);
sVec = zeros(1,numConfigs);

for i = 1:numConfigs
    thetaA = theta_start(:,i);
    thetaB = theta_goal(:,i);
    sVec(i) = checkCollLine(thetaA, thetaB, S, p_robot, p_obstacle, r_robot, r_obstacle);
end

mat2str(sVec)
%}
%% Question 3: Place bounding volumes for a given set of joint variables
%{
clear all;
clc;

S = [0.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000 0.00000000 0.00000000; 1.00000000 0.00000000 0.00000000 0.00000000 1.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 8.00000000 0.00000000; 2.00000000 1.00000000 0.00000000 0.00000000 4.00000000 0.00000000; 0.00000000 0.00000000 -4.00000000 0.00000000 0.00000000 1.00000000];
M = [0.00000000 0.00000000 1.00000000 -6.00000000; 0.00000000 -1.00000000 0.00000000 6.00000000; 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
theta = [-0.01748606; -0.89622607; 0.31055393; -0.59421302; -2.61470744; 2.91551340];

p = zeros(3, size(theta,2));
% characterize the robot
p(:,1) = [0; 0; 0];
p(:,2) = [-2; 0; 0];
p(:,3) = [-2; 2; 0];
p(:,4) = [-2; 4; 0];
p(:,5) = [0; 6; 0];
p(:,6) = [-4; 8; 0];
p(:,7) = [-6; 8; 0];
p(:,8) = [-6; 6; 0];

pFinal = moveSpheres(S,p,theta);

mat2str(pFinal, 6)
%}
%% Question 4: Plan a collision-free path from start to goal (4 joints)
%{
clear all;
clc;

S = [0.00 0.00 0.00 0.00; -1.00 0.00 0.00 -1.00; 0.00 -1.00 -1.00 0.00; -2.00 2.00 2.00 -6.00; 0.00 0.00 0.00 0.00; 0.00 0.00 0.00 0.00];
M = [-1.00 0.00 0.00 0.00; 0.00 -1.00 0.00 -2.00; 0.00 0.00 1.00 -6.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 0.00 0.00 0.00 0.00 0.00; 0.00 0.00 -2.00 -2.00 0.00 -2.00; 0.00 -2.00 -2.00 -4.00 -6.00 -6.00];
r_robot = [0.90 0.90 0.90 0.90 0.90 0.90];
p_obstacle = [-5.46 4.61 4.33 -1.08 -2.52 0.29 3.80 4.79 -2.77 3.19 -5.05 -4.93 5.54 -3.53 2.79 1.00 -5.17 -3.60 5.08 5.50 -3.70 4.49 2.75 4.97 -4.83 3.12 -0.61 -1.63 4.88 2.43 0.62 -3.42 1.19 -2.53 1.10 -0.02 5.74 -1.60 5.82 1.64; 0.32 -1.34 0.77 3.96 4.19 -3.93 -5.86 -0.63 0.64 2.07 2.75 4.69 -4.41 2.48 -3.36 -2.61 -2.13 3.70 -3.99 -0.20 5.23 2.39 0.76 5.31 -0.45 2.28 3.36 2.67 -5.83 3.19 -0.77 1.33 4.81 3.48 5.77 -4.05 3.70 -4.05 -5.70 -4.64; -4.30 -0.93 -1.14 1.59 -2.82 3.77 4.00 -1.26 -4.45 -1.76 -1.43 1.41 -4.82 4.91 5.85 1.20 2.01 -0.50 -1.44 2.09 3.35 4.61 -3.56 1.81 -4.42 -2.28 -5.64 5.26 -4.46 1.52 4.00 0.88 2.39 -3.71 2.53 -1.79 0.31 5.09 5.71 3.19];
r_obstacle = [0.77 0.39 0.75 0.67 0.79 0.62 0.29 0.64 0.34 0.27 0.48 0.50 0.39 0.34 0.29 0.30 0.44 0.42 0.66 0.54 0.33 0.49 0.48 0.27 0.67 0.72 0.71 0.31 0.24 0.42 0.66 0.76 0.68 0.48 0.68 0.42 0.73 0.43 0.57 0.47];
theta_start = [-2.50; -0.49; -3.14; 0.77];
theta_goal = [0.11; 2.38; 0.75; -2.36];



numConfigs = size(theta_start,2);
numSpheres = size(r_robot,2);
numObstacles = size(p_obstacle,2);
thetaTemp = zeros(size(theta_start));
done = 0;
while (done == 0)
    % sample thetaTemp from thetaFree
    for i = 1:size(theta_start,1)
        thetaTemp(i) = -3.14 + (3.14+3.14)*rand(1,1);
    end
    % check for collisions along the straight-line path from the start to
    % the sampled point    
    first = checkCollLine(theta_start, thetaTemp, S, p_robot, p_obstacle, r_robot, r_obstacle);
    second = checkCollLine(theta_goal, thetaTemp, S, p_robot, p_obstacle, r_robot, r_obstacle);
    if (~first && ~second)
        done = 1;
    end
end

answer = [theta_start thetaTemp theta_goal]
%}
%% Question 5: Are two spheres in collision (n times)
%{
clear all;
clc;

p = [0.0947 0.8083 2.0556 1.8179 1.2265 -3.5644 -4.9120 -2.4085 -2.1504 3.1341 -2.1314 -4.6598 -2.0806 -2.8740 1.3754 3.0807 0.2857 1.8032; -3.6395 -0.1830 -3.2165 -0.6103 -2.9144 -3.6025 -4.7132 3.2190 0.6563 2.7456 -3.5308 -1.8279 -2.6596 2.3383 -4.8989 4.1894 1.8874 -0.2407; 2.3903 2.4946 -4.9857 -2.7940 -3.0487 3.6127 -0.0713 -3.8394 2.5926 -1.5325 3.0841 4.2668 2.1407 -2.6926 -4.3210 2.5899 -2.7540 3.7441];
r = [1.4632 2.1886 1.9343 1.4914 1.5233 2.1769 1.6275 1.8734 1.8814 2.8544 1.0507 2.0692 1.4964 1.4640 2.9540 2.5453 1.5567 2.6662];
q = [-2.5445; 0.5711; 4.6478];
s = 2.3657;

collVec = zeros(1, size(p,2));
for i = 1:size(p,2);
    c = checkCollSphere(p(:,i), q, r(i), s);
    collVec(i) = c;
end

mat2str(collVec)
%}






