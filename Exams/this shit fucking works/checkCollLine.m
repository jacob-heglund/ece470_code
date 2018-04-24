% given an initial and final theta, check at points along the line
% parameterized by (1-s)*theta_start + s*theta_end
% if there is no collision, returns 0
% if there is collision, returns one of the values of s that cause
% collision
function return_value = checkCollLine(thetaA, thetaB, S, p_robot, p_obstacle, r_robot, r_obstacle)
n = 100;
s = linspace(0,1,n);

return_value = 0;

numSpheres = size(r_robot,2);
numObstacles = size(p_obstacle,2);

theta = zeros(numSpheres-2, n);
for i = 1:n
    theta(:,i) = (1-s(i))*thetaA + s(i)*thetaB;
end

numConfigs = n;
for i = 1:numConfigs
    pRobotFinal = moveSpheres(S, p_robot, theta(:,i));
    
    % check self collisions
    for j = 1:numSpheres
        for k = 1:numSpheres
            if (j==k);
                continue
            else
                collFlag = checkCollSphere(pRobotFinal(:,j), pRobotFinal(:,k), r_robot(j), r_robot(k));
                if collFlag == 1
                    %return_value = s(i);
                    return_value = 1;
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
                %return_value = s(i);
                return_value = 1;
                break
            end
        end
    end
end
end























