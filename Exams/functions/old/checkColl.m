% checks collisions for a single configuration.
% return values: 0 - no collision
%                1 - collision
 
function returnValue = checkColl(p_robot, p_obstacle, r_robot, r_obstacle, S, theta)
returnValue = 0;

numJoints = size(p_robot,2);
numObstacles = size(p_obstacle,2);

pRobotFinal = moveSpheres(p_robot,S,theta);

for j = 1:numJoints
    % check self collision
    for k = 1:numJoints
        if (j==k)
            continue
        else
            collFlag = checkCollSphere(pRobotFinal(:,j), pRobotFinal(:,k), r_robot(j), r_robot(k));
            if collFlag == 1
                returnValue = 1;
            end
        end
    end

    % check obstacle collision
    for k = 1:numObstacles
        collFlag = checkCollSphere(pRobotFinal(:,j), p_obstacle(:,k), r_robot(j), r_obstacle(k));
        if collFlag == 1
            returnValue = 1;
        end
    end
end














