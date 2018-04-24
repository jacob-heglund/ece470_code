function returnValue = checkCollLine(thetaA, thetaB, p_robot, p_obstacle, r_robot, r_obstacle, S)
returnValue = 0;

numJoints = size(p_robot,2);

n = 100;
s = linspace(0,1,n);
theta = zeros(numJoints-2, n);

for i = 1:n
    theta(:,i) = thetaA*(1-s(i)) + thetaB*s(i);
end

for i = 1:n
    thetaLocal = theta(:,i);
    collFlag = checkCollRobot(p_robot, p_obstacle, r_robot, r_obstacle, S, thetaLocal);
    if collFlag == 1
        returnValue = s(i);
        break
    end
   
end

end