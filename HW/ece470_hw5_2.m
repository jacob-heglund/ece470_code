%% HW5.2.1. Detect collision at a given configuration (2 joints)
%{
clear all;
clc;

% characterize the robot
S = [-1.00 0.00; 0.00 0.00; 0.00 -1.00; 0.00 0.00; 0.00 -2.00; 2.00 0.00];
M = [0.00 0.00 -1.00 -2.00; -1.00 0.00 0.00 -2.00; 0.00 1.00 0.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 -2.00 -2.00 -2.00; 0.00 2.00 0.00 -2.00; 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [-5.64 5.49 5.81 9.42 -5.22 -6.99 0.26 5.64 -8.76 2.74; 5.27 -5.75 -6.18 9.57 4.54 -5.55 -1.36 5.78 -1.06 -8.72; -2.05 4.37 -5.73 8.38 -6.05 9.02 -7.71 1.84 7.42 -3.67];
r_obstacle = [3.77 2.17 2.44 1.40 1.74 1.82 2.14 0.97 4.30 1.60];
theta = [-0.88; 1.15];

% the total number of spheres on the robot
numSpheresRobot = size(p_robot, 2);

numObstacles = size(p_obstacle, 2);

numConfigs = size(theta, 2);
% augment the positions of the robot with a row of ones
aug = ones(1, numSpheresRobot);
pAugInit = [p_robot; aug];

coll = zeros(1, numConfigs);

for i = 1:numConfigs;
    % the first two spheres aren't moved by any joints
    pAugFinal(:,1) = pAugInit(:,1);
    pAugFinal(:,2) = pAugInit(:,2);

    T = 1;
    for j = 1:size(S, 2)
        % move the center of each sphere by only the joints that affect it
        trans = expm(skew4(S(:,j))*theta(j));
        T = T*trans;
        pAugFinal(:, j+2) = T * pAugInit(:, j+2);
    end

    % now we have the centers of the spheres at the position described by theta
    pFinal = pAugFinal(1:3, :);
    
    % shows if each sphere is in collision for a particular configuration
    selfColl = zeros(numSpheresRobot, numSpheresRobot);
    obsColl = zeros(numObstacles, numSpheresRobot);

    for j = 1:numSpheresRobot
        % check if a sphere is colliding with any of the others on the robot, except itself
        for k = 1:numSpheresRobot
            if (j == k)
                continue
            else     
                p1 = pFinal(:,j);
                p2 = pFinal(:,k); 
                
                x1 = norm(p1-p2);
                x2 = r_robot(j) + r_robot(k);
                if (x1 <= x2)
                    selfColl(j,k) = 1;
                else
                    continue
                end
            end
        end
        % check if a sphere is colliding with an obstacle
        for m = 1:numObstacles
            x3 = norm(pFinal(:,j) - p_obstacle(:,m));
            x4 = r_robot(j) + r_obstacle(m);        
            if (x3 <= x4)
                obsColl(j,m) = 1;
            else
                continue
            end
        end
    end    
    
    % check if any of the rows in selfColl and obsColl show collisions
    selfIntermediate = any(selfColl);
    obsIntermediate = any(obsColl);

    % get if there were any collisions in this configuration
    selfBool = any(selfIntermediate);
    obsBool = any(obsIntermediate);
    
    % if there were either self or obstacle collisions, make a note of that
    if selfBool | obsBool
        coll(i) = 1;
    end
end
coll
%}
%% HW5.2.2. Detect collision at several different configurations (2 joints)
%{
clear all;
clc;

% characterize the robot
S = [0.00 -1.00; 0.00 0.00; 1.00 0.00; 2.00 0.00; 0.00 -2.00; 0.00 0.00];
M = [0.00 0.00 1.00 0.00; 0.00 -1.00 0.00 -2.00; 1.00 0.00 0.00 2.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 0.00 0.00 0.00; 0.00 2.00 0.00 -2.00; 0.00 2.00 2.00 2.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [-4.41 -9.62 -3.52 5.29 8.49 -7.33 4.53 -6.84 8.08 -7.50 -9.36 -2.90 1.08 -4.70 2.67; 7.12 -2.41 -1.38 0.91 4.44 1.23 1.47 -1.47 2.47 5.04 -6.77 6.83 4.03 6.42 9.11; 4.82 -5.02 -3.93 -7.62 5.44 9.86 -1.21 -9.84 -2.73 8.23 7.84 9.86 -7.83 3.44 1.96];
r_obstacle = [1.76 2.00 0.72 1.00 4.79 0.52 2.18 1.92 2.27 4.73 2.67 0.61 3.39 3.32 4.72];
theta = [-2.21 -1.71 -2.32 2.19 -1.59 -0.48 2.05 1.53 -1.93 2.38 0.45 -1.19 2.88 -2.62 2.88 0.48 -0.10 0.73 3.00 -2.53; 1.56 1.71 2.06 3.12 1.71 -2.22 -0.95 2.75 -0.85 -1.52 -2.25 -2.94 1.16 -2.07 2.88 -1.92 1.23 -3.12 0.74 -2.40];

% the total number of spheres on the robot
numSpheresRobot = size(p_robot, 2);

numObstacles = size(p_obstacle, 2);

numConfigs = size(theta, 2);

% augment the positions of the robot with a row of ones
aug = ones(1, numSpheresRobot);
pAugInit = [p_robot; aug];

coll = zeros(1, numConfigs);

for i = 1:numConfigs;
    thetaLocal = theta(:,i);
    % the first two spheres aren't moved by any joints
    pAugFinal(:,1) = pAugInit(:,1);
    pAugFinal(:,2) = pAugInit(:,2);

    T = 1;
    for j = 1:size(S, 2)
        % move the center of each sphere by only the joints that affect it
        
        trans = expm(skew4(S(:,j))*thetaLocal(j));
        T = T*trans;
        pAugFinal(:, j+2) = T * pAugInit(:, j+2);
    end

    % now we have the centers of the spheres at the position described by theta
    pFinal = pAugFinal(1:3, :);
    
    % shows if each sphere is in collision for a particular configuration
    selfColl = zeros(numSpheresRobot, numSpheresRobot);
    obsColl = zeros(numObstacles, numSpheresRobot);

    for j = 1:numSpheresRobot
        % check if a sphere is colliding with any of the others on the robot, except itself
        for k = 1:numSpheresRobot
            if (j == k)
                continue
            else     
                p1 = pFinal(:,j);
                p2 = pFinal(:,k); 
                
                x1 = norm(p1-p2);
                x2 = r_robot(j) + r_robot(k);
                if (x1 <= x2)
                    selfColl(j,k) = 1;
                else
                    continue
                end
            end
        end
        % check if a sphere is colliding with an obstacle
        for m = 1:numObstacles
            x3 = norm(pFinal(:,j) - p_obstacle(:,m));
            x4 = r_robot(j) + r_obstacle(m);        
            if (x3 <= x4)
                obsColl(j,m) = 1;
            else
                continue
            end
        end
    end    
    
    % check if any of the rows in selfColl and obsColl show collisions
    selfIntermediate = any(selfColl);
    obsIntermediate = any(obsColl);

    % get if there were any collisions in this configuration
    selfBool = any(selfIntermediate);
    obsBool = any(obsIntermediate);
    
    % if there were either self or obstacle collisions, make a note of that
    if selfBool | obsBool
        coll(i) = 1;
    end
end
mat2str(coll)
%}
%% HW5.2.3. Detect collision along a straight-line path (2 joints)
%{
clear all;
clc;

% characterize the robot
S = [0.00 0.00; 0.00 1.00; -1.00 0.00; 0.00 2.00; 2.00 0.00; 0.00 2.00];
M = [0.00 -1.00 0.00 0.00; 1.00 0.00 0.00 0.00; 0.00 0.00 1.00 -2.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 2.00 2.00 0.00; 0.00 0.00 0.00 0.00; 0.00 0.00 -2.00 -2.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [-2.53 -3.32 3.30 2.02 -4.10 0.79 4.58 -1.76 1.48 -3.42 -2.28 -0.73 3.24 -3.32 1.40 0.58; -2.63 3.12 3.11 -2.86 -0.96 -2.96 1.83 2.88 4.88 2.35 0.23 3.45 2.44 -3.62 3.86 -4.02; -2.67 -3.91 0.01 3.81 3.86 0.23 1.88 -1.81 2.61 -4.03 -3.14 1.67 3.76 -2.13 -1.57 0.35];
r_obstacle = [1.33 2.03 0.86 3.10 0.99 0.91 1.57 1.75 3.80 2.05 2.39 1.90 1.45 2.11 0.81 1.64];
theta_a = [-0.81; -2.98];
theta_b = [1.41; -1.62];

%resolution of our search for collisions along a straight-line path
n = 100;
s = linspace(0,1,n);

theta = zeros(2,n);
for i = 1:n
    theta(:,i) = (1-s(i))*theta_a + s(i)*theta_b;
end

% the total number of spheres on the robot
numSpheresRobot = size(p_robot, 2);

numObstacles = size(p_obstacle, 2);

numConfigs = size(theta, 2);

% augment the positions of the robot with a row of ones
aug = ones(1, numSpheresRobot);
pAugInit = [p_robot; aug];

coll = zeros(1, numConfigs);

for i = 1:numConfigs;
    thetaLocal = theta(:,i);
    % the first two spheres aren't moved by any joints
    pAugFinal(:,1) = pAugInit(:,1);
    pAugFinal(:,2) = pAugInit(:,2);

    T = 1;
    for j = 1:size(S, 2)
        % move the center of each sphere by only the joints that affect it
        
        trans = expm(skew4(S(:,j))*thetaLocal(j));
        T = T*trans;
        pAugFinal(:, j+2) = T * pAugInit(:, j+2);
    end

    % now we have the centers of the spheres at the position described by theta
    pFinal = pAugFinal(1:3, :);
    
    % shows if each sphere is in collision for a particular configuration
    selfColl = zeros(numSpheresRobot, numSpheresRobot);
    obsColl = zeros(numObstacles, numSpheresRobot);

    for j = 1:numSpheresRobot
        % check if a sphere is colliding with any of the others on the robot, except itself
        for k = 1:numSpheresRobot
            if (j == k)
                continue
            else     
                p1 = pFinal(:,j);
                p2 = pFinal(:,k); 
                
                x1 = norm(p1-p2);
                x2 = r_robot(j) + r_robot(k);
                if (x1 <= x2)
                    selfColl(j,k) = 1;
                else
                    continue
                end
            end
        end
        % check if a sphere is colliding with an obstacle
        for m = 1:numObstacles
            x3 = norm(pFinal(:,j) - p_obstacle(:,m));
            x4 = r_robot(j) + r_obstacle(m);        
            if (x3 <= x4)
                obsColl(j,m) = 1;
            else
                continue
            end
        end
    end    
    
    % check if any of the rows in selfColl and obsColl show collisions
    selfIntermediate = any(selfColl);
    obsIntermediate = any(obsColl);

    % get if there were any collisions in this configuration
    selfBool = any(selfIntermediate);
    obsBool = any(obsIntermediate);
    
    % if there were either self or obstacle collisions, make a note of that
    if selfBool | obsBool
        coll(i) = 1;
    end
end

%find the values of s that cause a collision
indices = find(coll == 1);
s(indices)

%}
%% HW5.2.4. Detect collision along several straight-line paths (2 joints)
%{
clear all;
clc;
% characterize the robot
S = [0.00 0.00; -1.00 0.00; 0.00 0.00; 0.00 0.00; 0.00 0.00; 0.00 -1.00];
M = [0.00 1.00 0.00 -2.00; 1.00 0.00 0.00 0.00; 0.00 0.00 -1.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 0.00 -2.00 -2.00; 0.00 -2.00 -2.00 0.00; 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [-1.10 -0.46 -4.88 -3.77 -1.07 -4.98 -0.94 1.07 -4.45 -4.58 -2.92; -4.73 -2.49 -1.75 -0.69 -4.16 -2.22 3.02 3.91 0.40 4.51 1.05; -3.45 3.35 -1.13 4.61 -4.19 -2.17 -2.60 1.57 2.36 3.39 -3.40];
r_obstacle = [1.03 0.54 2.07 0.82 2.49 1.31 1.59 1.74 1.22 3.44 2.12];
theta_start = [2.49 -1.64 -1.58 0.53 0.73 -0.37 -0.72 -2.93 -2.67; -1.94 0.31 1.54 -1.79 2.36 -2.25 -2.52 -3.00 0.23];
theta_goal = [-2.58 2.85 2.46 -1.36 0.02 -2.55 0.70 -0.31 -1.97; -1.98 0.60 1.53 -0.71 -2.04 0.92 -0.81 -2.37 1.04];


sColl = zeros(1, size(theta_start,2));

for z = 1:size(theta_start,2);
    theta_a = theta_start(:,z);
    theta_b = theta_goal(:,z);
    %resolution of our search for collisions along a straight-line path
    n = 100;
    s = linspace(0,1,n);

    theta = zeros(2,n);
    for i = 1:n
        theta(:,i) = (1-s(i))*theta_a + s(i)*theta_b;
    end

    % the total number of spheres on the robot
    numSpheresRobot = size(p_robot, 2);

    numObstacles = size(p_obstacle, 2);

    numConfigs = size(theta, 2);

    % augment the positions of the robot with a row of ones
    aug = ones(1, numSpheresRobot);
    pAugInit = [p_robot; aug];

    coll = zeros(1, numConfigs);

    for i = 1:numConfigs;
        thetaLocal = theta(:,i);
        % the first two spheres aren't moved by any joints
        pAugFinal(:,1) = pAugInit(:,1);
        pAugFinal(:,2) = pAugInit(:,2);

        T = 1;
        for j = 1:size(S, 2)
            % move the center of each sphere by only the joints that affect it

            trans = expm(skew4(S(:,j))*thetaLocal(j));
            T = T*trans;
            pAugFinal(:, j+2) = T * pAugInit(:, j+2);
        end

        % now we have the centers of the spheres at the position described by theta
        pFinal = pAugFinal(1:3, :);

        % shows if each sphere is in collision for a particular configuration
        selfColl = zeros(numSpheresRobot, numSpheresRobot);
        obsColl = zeros(numObstacles, numSpheresRobot);

        for j = 1:numSpheresRobot
            % check if a sphere is colliding with any of the others on the robot, except itself
            for k = 1:numSpheresRobot
                if (j == k)
                    continue
                else     
                    p1 = pFinal(:,j);
                    p2 = pFinal(:,k); 

                    x1 = norm(p1-p2);
                    x2 = r_robot(j) + r_robot(k);
                    if (x1 <= x2)
                        selfColl(j,k) = 1;
                    else
                        continue
                    end
                end
            end
            % check if a sphere is colliding with an obstacle
            for m = 1:numObstacles
                x3 = norm(pFinal(:,j) - p_obstacle(:,m));
                x4 = r_robot(j) + r_obstacle(m);        
                if (x3 <= x4)
                    obsColl(j,m) = 1;
                else
                    continue
                end
            end
        end    

        % check if any of the rows in selfColl and obsColl show collisions
        selfIntermediate = any(selfColl);
        obsIntermediate = any(obsColl);

        % get if there were any collisions in this configuration
        selfBool = any(selfIntermediate);
        obsBool = any(obsIntermediate);

        % if there were either self or obstacle collisions, make a note of that
        if selfBool | obsBool
            coll(i) = 1;
        end
    end

    %find the values of s that cause a collision
    indices = find(coll == 1);
    answer = s(indices);
    if (size(answer,2) ~= 0)
        sColl(z) = answer(1);    
    end
end


mat2str(sColl)
%}
%% HW5.2.8. Detect collision along several straight-line paths
%{
clear all;
clc;
% characterize the robot
S = [0.00 0.00 0.00 0.00 0.00; 0.00 0.00 -1.00 0.00 1.00; 1.00 0.00 0.00 0.00 0.00; 2.00 0.00 0.00 -1.00 0.00; 0.00 0.00 0.00 0.00 0.00; 0.00 1.00 4.00 0.00 -6.00];
M = [0.00 -1.00 0.00 -6.00; 0.00 0.00 1.00 0.00; -1.00 0.00 0.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 0.00 -4.00 -4.00 -4.00 -6.00 -6.00; 0.00 2.00 2.00 0.00 -2.00 -2.00 0.00; 0.00 0.00 0.00 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90 0.90 0.90 0.90];
p_obstacle = [4.43 -2.37 3.76 3.29 4.84; -3.28 -1.02 -2.27 -1.46 -4.63; 0.34 2.14 -0.02 3.50 2.62];
r_obstacle = [2.17 0.56 1.19 3.04 4.93];
theta_start = [-2.78 -1.27 -0.27 -1.86 -0.39 -1.32 0.09 -1.64; -2.67 -1.35 0.83 2.86 1.40 -1.44 -0.60 2.48; -1.13 -1.35 -0.05 -2.48 1.89 -2.29 1.10 0.84; 1.69 0.23 2.36 1.51 -0.02 -0.10 1.66 2.95; 3.11 0.41 1.01 3.04 -2.33 2.82 1.94 1.26];
theta_goal = [-2.03 -2.65 -2.29 2.91 -1.89 -2.16 -1.02 -2.03; 3.09 -2.48 -2.32 -0.94 -1.47 3.04 -0.80 0.93; 1.31 -1.53 -0.99 0.33 0.00 -0.92 0.52 0.25; 1.89 0.09 3.13 1.25 1.70 2.40 1.52 0.17; 2.31 1.72 -1.11 -0.06 0.49 1.88 -0.61 -2.49];

sColl = zeros(1, size(theta_start,2));

for z = 1:size(theta_start,2);
    theta_a = theta_start(:,z);
    theta_b = theta_goal(:,z);
    %resolution of our search for collisions along a straight-line path
    n = 100;
    s = linspace(0,1,n);

    theta = zeros(size(theta_start,1),n);
    for i = 1:n
        theta(:,i) = (1-s(i))*theta_a + s(i)*theta_b;
    end

    % the total number of spheres on the robot
    numSpheresRobot = size(p_robot, 2);

    numObstacles = size(p_obstacle, 2);

    numConfigs = size(theta, 2);

    % augment the positions of the robot with a row of ones
    aug = ones(1, numSpheresRobot);
    pAugInit = [p_robot; aug];

    coll = zeros(1, numConfigs);

    for i = 1:numConfigs;
        thetaLocal = theta(:,i);
        % the first two spheres aren't moved by any joints
        pAugFinal(:,1) = pAugInit(:,1);
        pAugFinal(:,2) = pAugInit(:,2);

        T = 1;
        for j = 1:size(S, 2)
            % move the center of each sphere by only the joints that affect it

            trans = expm(skew4(S(:,j))*thetaLocal(j));
            T = T*trans;
            pAugFinal(:, j+2) = T * pAugInit(:, j+2);
        end

        % now we have the centers of the spheres at the position described by theta
        pFinal = pAugFinal(1:3, :);

        % shows if each sphere is in collision for a particular configuration
        selfColl = zeros(numSpheresRobot, numSpheresRobot);
        obsColl = zeros(numObstacles, numSpheresRobot);

        for j = 1:numSpheresRobot
            % check if a sphere is colliding with any of the others on the robot, except itself
            for k = 1:numSpheresRobot
                if (j == k)
                    continue
                else     
                    p1 = pFinal(:,j);
                    p2 = pFinal(:,k); 

                    x1 = norm(p1-p2);
                    x2 = r_robot(j) + r_robot(k);
                    if (x1 <= x2)
                        selfColl(j,k) = 1;
                    else
                        continue
                    end
                end
            end
            % check if a sphere is colliding with an obstacle
            for m = 1:numObstacles
                x3 = norm(pFinal(:,j) - p_obstacle(:,m));
                x4 = r_robot(j) + r_obstacle(m);        
                if (x3 <= x4)
                    obsColl(j,m) = 1;
                else
                    continue
                end
            end
        end    

        % check if any of the rows in selfColl and obsColl show collisions
        selfIntermediate = any(selfColl);
        obsIntermediate = any(obsColl);

        % get if there were any collisions in this configuration
        selfBool = any(selfIntermediate);
        obsBool = any(obsIntermediate);

        % if there were either self or obstacle collisions, make a note of that
        if selfBool | obsBool
            coll(i) = 1;
        end
    end

    %find the values of s that cause a collision
    indices = find(coll == 1);
    answer = s(indices);
    if (size(answer,2) ~= 0)
        sColl(z) = answer(1);    
    end
end


mat2str(sColl)

sColl = zeros(1, size(theta_start,2));

%}





































