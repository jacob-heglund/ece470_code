function returnValue = checkCollRobot(p_robot, p_obstacle, r_robot, r_obstacle, S, theta)
returnValue = 0;

numJoints = size(p_robot,2);
numObstacles = size(p_obstacle,2);

pRobotFinal = moveSpheres(p_robot,S,theta);
for i = 1:numJoints
    % check self collision
    for j = 1:numJoints
        if i==j
            continue
        else
            collFlag = checkCollSpheres(pRobotFinal(:,i), pRobotFinal(:,j), r_robot(i), r_robot(j));
            if collFlag == 1
                returnValue = 1;
            end
        end
    end
    % check obstacle collision
    for j = 1:numObstacles
            collFlag = checkCollSpheres(pRobotFinal(:,i), p_obstacle(:,j), r_robot(i), r_obstacle(j));
            if collFlag == 1
                returnValue = 1;
            end
    end
end





























