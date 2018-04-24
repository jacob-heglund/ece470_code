% checks for collision between two spheres given the position of their
% center and their radii
function c = checkCollSphere(p1, p2, r1, r2)
c = 0;
dist = norm(p1 - p2, 2);
r = r1+r2;
if (dist <= r)
    c = 1;
end
end


