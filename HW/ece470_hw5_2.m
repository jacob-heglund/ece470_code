%% HW5.2.1. Detect collision at a given configuration (2 joints)
%{
clear all;
clc;

% characterize the robot
S = [-1.00 0.00; 0.00 1.00; 0.00 0.00; 0.00 0.00; 0.00 0.00; 0.00 -2.00];
M = [0.00 0.00 -1.00 -4.00; 0.00 -1.00 0.00 2.00; -1.00 0.00 0.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 -2.00 -2.00 -4.00; 0.00 0.00 2.00 2.00; 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [9.94 4.10 5.57 -6.05 2.88 4.65 2.44 -9.29 5.43 4.75 0.67 3.57 7.23 6.98 -4.81 -7.33 1.06 -4.51 4.37 -9.31 -5.17 -2.85 6.19 -7.71; 1.54 7.78 -1.88 6.37 -9.27 3.06 -9.37 -4.77 -0.66 0.17 2.91 -4.36 2.46 -4.25 -5.22 1.86 -3.07 3.78 6.70 -9.76 -8.22 7.72 -4.90 -8.53; -2.14 7.50 9.19 1.42 -2.93 1.18 3.98 -0.10 6.29 -5.97 5.71 -3.93 -5.35 7.49 -6.10 -8.85 6.03 -5.00 -4.72 -3.19 -7.33 6.03 4.10 1.95];
r_obstacle = [3.70 3.45 2.96 1.52 3.55 4.27 0.90 2.56 1.41 1.53 4.24 3.40 3.09 2.14 0.58 3.61 1.19 1.66 1.19 0.90 4.73 1.23 1.56 1.42];
theta = [1.79; -1.86];

% the total number of spheres on the robot
numSpheresRobot = size(p_robot)(2);
numObstacles = size(p_obstacle)(2);

% augment the positions of the robot with a row of ones
aug = ones(1, numSpheresRobot);
pAugInit = [p_robot; aug];

coll = zeros(1, size(theta)(2));

for i = 1:size(theta)(2)
    % the first two spheres aren't moved by any joints
    pAugFinal(:,1) = pAugInit(:,1);
    pAugFinal(:,2) = pAugInit(:,2);

    T = 1;
    for j = 1:size(S)(2)
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
    %str = strcat("Configuration ", mat2str(i), "\n");
    %fprintf(str)
    %selfColl
    %obsColl
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
clear all;
clc;

% characterize the robot
S = [-1.00 0.00; 0.00 0.00; 0.00 0.00; 0.00 0.00; 0.00 -1.00; 0.00 0.00];
M = [1.00 0.00 0.00 -2.00; 0.00 0.00 -1.00 -4.00; 0.00 1.00 0.00 0.00; 0.00 0.00 0.00 1.00];
p_robot = [0.00 -2.00 -2.00 -2.00; 0.00 0.00 -2.00 -4.00; 0.00 0.00 0.00 0.00];
r_robot = [0.90 0.90 0.90 0.90];
p_obstacle = [-2.05 -0.78 3.97 5.86 3.06 -9.87 -0.96 -5.39 6.70 -8.66 -6.19; -6.55 1.54 -9.13 0.48 0.42 8.71 -1.35 4.54 9.47 9.59 8.28; -6.92 4.67 -6.58 3.55 -5.99 -4.92 8.69 -7.66 2.63 5.39 3.75];
r_obstacle = [4.55 1.69 3.47 4.57 3.96 2.02 1.87 3.72 3.92 4.35 4.74];
theta = [-2.68 -2.96 0.55 2.70 -0.93 -3.13 -0.95 -2.73 0.84 0.05 -2.57 -1.26 -2.77; 1.95 0.37 1.45 -0.69 0.74 2.76 -0.56 0.60 1.66 -0.91 1.75 2.21 2.52];

% the total number of spheres on the robot
numSpheresRobot = size(p_robot)(2);
numObstacles = size(p_obstacle)(2);

% augment the positions of the robot with a row of ones
aug = ones(1, numSpheresRobot);
pAugInit = [p_robot; aug];

coll = zeros(1, size(theta)(2));

for i = 1:size(theta)(2)
    % the first two spheres aren't moved by any joints
    pAugFinal(:,1) = pAugInit(:,1);
    pAugFinal(:,2) = pAugInit(:,2);

    T = 1;
    for j = 1:size(S)(2)
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
    %str = strcat("Configuration ", mat2str(i), "\n");
    %fprintf(str)
    %selfColl
    %obsColl
    % check if any of the rows in selfColl and obsColl show collisions
    selfIntermediate = any(selfColl);
    obsIntermediate = any(obsColl);

    % get if there were any collisions in this configuration
    selfBool = any(selfIntermediate);
    obsBool = any(obsIntermediate);
    
    %i
    %selfBool
    %obsBool

    % if there were either self or obstacle collisions in this orientation, 
    % make a note of it in coll
    if selfBool | obsBool
        coll(i) = 1;
    end
end

mat2str(coll)
















