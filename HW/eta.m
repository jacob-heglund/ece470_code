function output = eta(r)
numPts = size(r, 2);
for i = 1:numPts
    e1 = r(1, i) / r(3, i);
    e2  = r(2, i) / r(3, i);
    e3 = 1;
    output(:,i) = [e1; e2; e3];
end
end

