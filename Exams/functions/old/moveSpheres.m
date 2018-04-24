% function that returns the final position of the centers of spheres given
% by p, S, and M moved by distance theta.  Note, this only works for a
% single configuration, so pass in a nx1 theta
function pFinal = moveSpheres(pInit, S, theta)

numJoints = size(pInit,2);
aug = ones(1, size(pInit,2));
pAugInit = [pInit; aug];

pAugFinal(:,1) = pAugInit(:,1);
pAugFinal(:,2) = pAugInit(:,2);

T = 1;
for i = 1:(numJoints - 2)
    trans = expm(skew4(S(:,i))*theta(i));
    T = T*trans;
    pAugFinal(:,i+2) = T*pAugInit(:,i+2);
end
pFinal = pAugFinal(1:3, :);
end
