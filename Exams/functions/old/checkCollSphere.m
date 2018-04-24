% function checkCollSphere - checks if two spheres are in collision
% return values - 0: no collision
%                 1: collision
function c = checkCollSphere(p1, p2, r1, r2)
c = 0;
dist  = norm(p1-p2,2);
r = r1+r2;

if (dist <= r)
    c = 1;
end
end