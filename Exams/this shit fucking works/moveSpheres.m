% this function takes the initial position of centers of spheres and moves them
% to their final config given by theta and S
function pFinal = moveSpheres(S,p,theta)

aug = ones(1, size(p,2));
pAugInit = [p; aug];

% move the centers of the spheres
pAugFinal(:,1) = pAugInit(:,1);
pAugFinal(:,2) = pAugInit(:,2);

T = 1;
for i = 1:(size(p,2) - 2)
    trans = expm(skew4(S(:,i))*theta(i));
    T = T*trans;
    pAugFinal(:, i+2) = T*pAugInit(:,i+2);
end
pFinal = pAugFinal(1:3, :);













