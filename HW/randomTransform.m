function M = randomTransform()
% generates a random homogenous transformation matrix with columns 1-3 unit vectors
v1 = rand(3,1);
v2 = rand(3,1);
v3 = rand(3,1);
p = 3*rand(3,1);

% normalize the vectors
v1 = v1./sqrt(v1'*v1);
v2 = v2./sqrt(v2'*v2);
v3 = v2./sqrt(v3'*v3);
aug = [0 0 0 1];
M = [v1 v2 v3 p; aug];
