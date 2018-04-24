% this function checks if there is a collision along a straight line between two points
function collFlagLine = checkCollLine(thetaA, thetaB, S, p_robot, p_obstacle, r_robot, r_obstacle)
% collFlagLine = 0 -> no collisions, = 1-> collision along the line
collFlagLine = 0;

% set resolution in our search for collisions along a straight-line path
n = 100;
s = linspace(0,1,n);

theta = zeros(2,n);
% generate configurations along the straight-line path 
for i = 1:n
    theta(:,i) = (1-s(i))*thetaA + s(i)*thetaB;
end

% check for collision along the straight-line path
for i = i:n
    thetaLocal = theta(:,i);
    collFlagLine = checkColl(thetaLocal, S, p_robot, p_obstacle, r_robot, r_obstacle);
    if (collFlagLine == 1)
        break
    end
end



