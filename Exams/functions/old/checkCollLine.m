% function checks for collisions along a line between thetaA and thetaB,
% returns the s value that causes the collision if there is one

function sColl = checkCollLine(thetaA, thetaB,p_robot, p_obstacle, r_robot, r_obstacle, S)
sColl = 0;

n = 100;
s = linspace(0,1,n);

for i = 1:n
    theta(:,i) = (1-s(i))*thetaA + s(i)*thetaB;
end

for i = 1:n
    collFlag = checkColl(p_robot,p_obstacle,r_robot,r_obstacle,S,theta(:,i));
    if collFlag == 1
        sColl = s(i);
    end
end
end