%% HW6.2.1. Find the tag pose with minimum projection error (no noise)
clear all;
clc;

w = 0.15000000;
K = [207.84609691 0.00000000 160.00000000; 0.00000000 207.84609691 120.00000000; 0.00000000 0.00000000 1.00000000];
q = [12.65518707 42.44406887 132.81774316 69.51676409; -17.84341152 -147.69517717 111.34976124 110.23306759];
augMarker = ones(1, size(q,2));
qMarker = [q; augMarker];

% standard projection matrix
pi0 = [1 0 0 0; 0 1 0 0; 0 0 1 0];

% find points of the fiducial marker in the camera frame
% place the camera at a distance of .5 away from the marker 
zCam = .5;

p1 = [-w/2; -w/2; zCam];
p2 = [w/2; -w/2; zCam];
p3 = [w/2; w/2; zCam];
p4 = [-w/2; w/2; zCam];
aug = [1 1 1 1];

pCam = [p1 p2 p3 p4; aug];

% generate random initial camera pose
v1 = rand(3,1);
v1 = [v1/norm(v1); 0];
v2 = rand(3,1);
v2 = [v2/norm(v2); 0];
v3 = rand(3,1);
v3 = [v3/norm(v3); 0];
p = [2; 2; zCam; 1];
T = [v1 v2 v3 p];

% find initial view of the fiducial marker
qCam = eta(K*pi0*T*pCam);

numData = size(qCam, 2);

% stuff for the Marquardt method
deltaMat = eye(6);
mu = .1;
I = eye(6);

loopCount = 0;
tolerance = .1;
done = 0;
while (done == 0)    
    b_i = zeros(numData, 1);
    % the two summations in the linear least squares problem
    term1 = zeros(6);
    term2 = zeros(6,1);

    % for each datapoint, find the quantity b = qMarker - f(T)
    for i = 1:numData
        % find the derivative for this datapoint using chain rule J_i with dimension numData x 6
        J_i = zeros(3, 6);
        
        for j = 1:6
            delta = deltaMat(:,j);
            % find d(eta) / dr, size 3 x 3
            deriv1 = etaPrime(K * pi0 * T * pCam(:,i));
            
            % find d(stuff) du, size 
            deriv2 = K * pi0 * skew4(delta) * T * pCam(:,i);
            
            J_i(:,j) = deriv1 * deriv2;
        end
        b_i = qMarker(:,i) - eta(K*pi0*T*pCam(:,i));
        
        term1 = term1 + (J_i' * J_i);
        term2 = term2 + J_i' * b_i;  
    end
    % update u and M
    term1 = term1 + mu * eye(6);
    u = inv(term1) * term2;
    
    % some end condition
    if norm(u) < tolerance
        done = 1;
    end
    
    T = expm(skew4(u))*T;
                
    
loopCount = loopCount + 1
if loopCount == 500
    norm(u)
    return
    
end
end

err = norm(q_in0 - term1*q_in1, inf);
mat2str(term1,7);











