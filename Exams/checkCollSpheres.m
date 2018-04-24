function returnValue = checkCollSpheres(p1, p2, r1, r2)
returnValue = 0;
dist = norm(p1-p2, 2);
r = r1+r2;
if (dist <= r)
    returnValue = 1;
end