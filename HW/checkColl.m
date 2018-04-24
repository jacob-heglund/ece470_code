% this function takes in a single configuration and checks if there is self or object collision
function collFlag = checkColl(theta, S, p_robot, p_obstacle, r_robot, r_obstacle)

collFlag = 0;

% robot and environment initialization
numSpheresRobot = size(p_robot, 2);
numObstacles = size(p_obstacle, 2);

% augment the positions of the robot with a row of ones
aug = ones(1, numSpheresRobot);
pAugInit = [p_robot; aug];

% move the joints by the appropriate joint variable amount
% the first two spheres aren't moved by any joints
pAugFinal(:,1) = pAugInit(:,1);
pAugFinal(:,2) = pAugInit(:,2);

% move the center of each sphere by only the joints that affect it
T = 1;
for j = 1:size(S, 2)
    trans = expm(skew4(S(:,j))*theta(j));
    T = T*trans;
    pAugFinal(:, j+2) = T * pAugInit(:, j+2);
end

% now we have the centers of the spheres at the position described by theta
pFinal = pAugFinal(1:3, :);

% shows if each sphere is in collision for a particular configuration
selfColl = zeros(numSpheresRobot, numSpheresRobot);
obsColl = zeros(numObstacles, numSpheresRobot);

% check if a sphere is colliding with any of the others on the robot, except itself
% also check if a sphere is colliding with an obstacle
for j = 1:numSpheresRobot    
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
if (selfBool | obsBool)
    collFlag = 1;
end















