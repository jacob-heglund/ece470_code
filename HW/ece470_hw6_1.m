%% HW6.1.1. Find the transformation matrix that would align two point clouds

clear all;
clc;

q_in1 = [-2.30292215 -0.53807967 -0.74953608 1.48114245 -0.85565124 -0.45743467 1.90910358 0.53428153 -0.69152384 0.74542195 -0.99574392 0.19321111 0.98939379 -0.67169174; -1.01065681 -0.34595221 -0.03908132 0.18380410 -0.78631927 -0.33322844 -0.32039888 0.83899395 1.69277096 -1.44674514 -1.30836703 0.14318142 -1.17124068 0.64909524; -0.60186798 -0.73908885 -0.29130914 -0.07916521 -0.89866438 -1.30792825 1.18982007 -1.78774657 0.21529934 -0.17885952 -0.38023725 -0.42218045 0.76548813 -0.17094930; 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000];
q_in0 = [3.43385340 1.72815366 1.66809848 -0.45441282 2.19165842 1.85877131 -1.15291321 0.82223217 0.94786276 0.69270185 2.27486325 0.82620585 0.05679382 1.36248528; -0.00086365 0.24787304 -0.26827844 0.39905430 0.53821898 0.49556148 0.49013605 0.14070579 -1.86072895 1.46456768 0.70224248 0.02868325 0.96878775 -0.84620884; -1.61591200 -2.39311800 -2.12973379 -2.47646489 -2.24785349 -2.89494788 -1.23087412 -4.07286809 -2.58216260 -1.60541747 -1.52183712 -2.50676143 -0.99138107 -2.38805577; 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000];

% declare initial M
M = eye(4);
numData = size(q_in0, 2);
deltaMat = eye(6);
mu = .1;
I = eye(6);

loopCount = 0;
tolerance = .00000000000001;
done = 0;
while (done == 0)
    %b = zeros(4, numData);    
    b_i = zeros(4, 1);
    % the two summations in the linear least squares problem
    term1 = zeros(6);
    term2 = zeros(6,1);

    % for each datapoint, find the quantity b = q_in0 - M*q_in1
    for i = 1:numData
        %b(:,i) = q_in0(:,i) - M*q_in1(:,i);
        b_i = q_in0(:,i) - M*q_in1(:,i);
        
        % find the derivative for this datapoint J_i with dimension (i, 6)
        J_i = zeros(4, 6);
        for j = 1:6
            delta = deltaMat(:,j);
            deriv = skew4(delta)* M *q_in1(:,i);
            J_i(:,j) = deriv;
        end
        term1 = term1 + J_i'*J_i + mu*I;
        term2 = term2 + J_i'*b_i;  
    end
    
    % update u and M
    u = term1^-1 * term2;
    M = expm(skew4(u))*M;
                
    % some end condition
    if norm(u) < tolerance
        done = 1;
    end
loopCount = loopCount + 1
end

err = norm(q_in0 - M*q_in1, inf)
mat2str(M,7)


%% HW6.1.2. Find the transformation matrix that would best align two point clouds
%{
clear all;
clc;

q_in1 = [-0.59811655 -1.70852373 0.83304277 0.00130773 0.68698800 -0.12600659 1.66027611 0.55799538 2.00555965 -0.87623047 -1.88453358 1.71556546 -1.03312739; -0.31001211 -0.18348355 -0.38316043 -1.41791896 0.51727083 1.67379640 -0.33437300 -0.18150058 -0.40523633 1.08491786 1.47207393 -0.28637501 1.40216278; 1.06480023 -0.24772810 -0.69639203 1.39351780 0.12642265 0.79941609 -0.51866177 -0.80948573 -0.54404170 -1.90110145 0.51296853 -0.76723167 0.17023355; 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000];
q_in0 = [-0.83719421 -1.64429038 1.66573695 -1.19363592 0.97777617 -0.17097730 3.02675616 1.28089487 2.16152455 1.23388300 -1.64818698 1.78785498 -0.52512844; -2.18479099 -3.03256458 -2.20446372 -2.12369619 -2.81008935 -4.43089768 -2.28760371 -3.49300731 -2.35044165 -3.69809080 -5.37988727 -3.27451867 -4.55032681; 0.65006735 1.27099787 1.37790570 1.19145912 0.93824877 0.03472952 1.72165712 1.95901158 0.71120825 3.02945662 1.22809296 0.86574905 1.17011646; 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000];

% declare initial M
M = eye(4);
numData = size(q_in0, 2);
deltaMat = eye(6);
mu = .1;
I = eye(6);

loopCount = 0;
tolerance = .00000000000001;
done = 0;
while (done == 0)
    %b = zeros(4, numData);    
    b_i = zeros(4, 1);
    % the two summations in the linear least squares problem
    term1 = zeros(6);
    term2 = zeros(6,1);

    % for each datapoint, find the quantity b = q_in0 - M*q_in1
    for i = 1:numData
        %b(:,i) = q_in0(:,i) - M*q_in1(:,i);
        b_i = q_in0(:,i) - M*q_in1(:,i);
        
        % find the derivative for this datapoint J_i with dimension (i, 6)
        J_i = zeros(4, 6);
        for j = 1:6
            delta = deltaMat(:,j);
            deriv = skew4(delta)* M *q_in1(:,i);
            J_i(:,j) = deriv;
        end
        term1 = term1 + J_i'*J_i + mu*I;
        term2 = term2 + J_i'*b_i;  
    end
    
    % update u and M
    u = term1^-1 * term2;
    M = expm(skew4(u))*M;
                
    % some end condition
    if norm(u) < tolerance
        done = 1;
    end
loopCount = loopCount + 1
end

err = norm(q_in0 - M*q_in1, 2)
mat2str(M,7)
%}
%% HW6.1.3. Find the transformation matrix that would align two point clouds (unknown correspondences)
%{
clear all;
clc;

q_in1 = [0.07985991 0.90328607 -1.32839515 -2.14345821 0.56857753 0.74393381 -0.81406299 1.67366153; 1.12760926 0.82917518 -0.45791512 0.06246639 -0.23600800 -0.51753994 -0.63781593 -0.64000403; -0.66013134 -1.84674753 -0.86346799 0.10434538 -0.25103307 -0.90254432 0.66820584 1.78857206; 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000];
q_in0 = [0.33171870 -1.75047938 -1.61833412 0.09931332 2.33008929 -0.45361091 -0.28941570 -0.31264843; 0.52915759 -0.12238794 -0.09212434 0.54099100 -0.21174384 1.67682841 1.98158054 -0.49601784; -0.15731992 1.47709606 0.18238858 -0.94098750 0.66265890 0.33468308 -1.07251507 0.96446057; 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000 1.00000000];

% step 1 - declare initial M - the homogenous transform that takes q_in1 -> q_in0 (or at least as close as we can get it)
M = randomTransform();

numModel = size(q_in0, 2);
numData = size(q_in1, 2);
deltaMat = eye(6);
mu = .1;
I = eye(6);

loopCount = 0;
%tolerance = .0000000000000001;
tolerance = .01;

done = 0;
while (done == 0)
    b_i = zeros(4, 1);    
    % the two summations in the linear least squares problem
    term1 = zeros(6);
    term2 = zeros(6,1);
        
    % for each datapoint, find the quantity b = q_in0 - M*q_in1
    for i = 1:numData
        % for known correspondance
        %b_i = q_in0(:,i) - M*q_in1(:,i);
        
        % for unknown correspondance, find the closest model point to our data point
        dist = zeros(1,numModel);
        for j = 1:numModel
            dist(j) = norm(q_in0(:,j) - q_in1(:,i));            
        end
        
        [val, idx] = min(dist);
                
        b_i = q_in0(:,idx) - M*q_in1(:,i);
        % an array for checking the error.  each entry should have a 1-to-1 correspondance to the entries in 
        % q_in1
        q_in0_temp(:,i) = q_in0(:,idx);
        
        % find the derivative for this datapoint J_i with dimension (i, 6)
        J_i = zeros(4, 6);
        for j = 1:6
            delta = deltaMat(:,j);
            deriv = skew4(delta)* M *q_in1(:,i);
            J_i(:,j) = deriv;
        end
        term1 = term1 + J_i'*J_i + mu*I;
        term2 = term2 + J_i'*b_i;  
    end
    
    % update u and M
    u = term1^-1 * term2;
    M = expm(skew4(u))*M;
    
    %end condition - make sure there's zero error between the aligned points
    q_in1_temp = zeros(size(q_in1));
    error = 0;
    
    for i = 1:numData
       q_in1_temp(:,i) = M*q_in1(:,i);
       error = error + norm(q_in1_temp(:,i) - q_in0_temp(:,i), 2);
    end
    
    if error < tolerance
       done = 1; 
    end
    
    
    % end condition - check if the centroids are the same
    %{
    avgX0 = mean(q_in0(1,:));
    avgY0 = mean(q_in0(2,:));
    avgZ0 = mean(q_in0(3,:));
    
    centroid0 = [avgX0; avgY0; avgZ0];
    
    q_in1_temp = zeros(size(q_in1));
    for i = 1:numData
        q_in1_temp(:,i) = M*q_in1(:,i);
    end
    
    avgX1 = mean(q_in1_temp(1,:));
    avgY1 = mean(q_in1_temp(2,:));
    avgZ1 = mean(q_in1_temp(3,:));

    centroid1 = [avgX1; avgY1; avgZ1];
    if centroid0 == centroid0
        done = 1;
    end
    %}
    
    % end condition - check if the norm of u is small
    %{
    if norm(u) < tolerance
        done = 1;
    end
    %}
loopCount = loopCount + 1
error
end

mat2str(M,7)


%}




















