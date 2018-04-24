% takes two point clouds as input and aligns them
% utilizes Levenburg - Marquardt root finding algorithm
% return values: T_1in0 - the transformation matrix that best aligns the
%                two point clouds
function T_1in0 = cloudAlign(q0, q1)
aligned = 0;

numPoints = size(q1,2);
% augment the points so we can use transformation matrices
augQ = ones(1, numPoints);
q0aug = [q0; augQ];
q1aug = [q1; augQ];

% choose an initial orientation for point cloud 1
I = eye(3);
augM = [0 0 0 1];
p = 5*rand(3,1);
M = [I p; augM];

% choose a small pertubation for our initial u
u = .1*rand(6,1);

unitVecs = eye(6,1);

tol = .0001;
while (aligned == 0)
    b = zeros(4, numPoints);
    J = zeros(4, numPoints);
    
    for i = 1:numPoints
        % find the b terms
        b(:,i) = q0aug(:,i) - expm(skew4(u))*M*q1aug(:,i);
        
        % find the J terms
        for j = 1:6
            J(:,j) = skew4(unitVecs(:,j))*M*q1aug(:,i);
        end
    end
    
    % update u given the values we just found
    uNext = 0;
    for i = i:numPoints
        % a regularization term so the matrix is invertible
        reg = mu*eye(6);
        updateTerm = ((J(:,i)'*J(:,i) + reg)^-1)*J(:,i)'*b(:,i);
        uNext = uNext + updateTerm;
    end
    
    % update for the next iteration
    u = uNext;
    M = expm(skew4(u))*M;
    
    % the end condition for the while loop
    if norm(u) < tol
        aligned = 1;
        T_1in0 = expm(skew4(u))*M;
    end
    
end
end

