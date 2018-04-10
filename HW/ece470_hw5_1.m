%% HW5.1.1. Are two spheres in collision
%{
clear all;
close all;
clc;
sides = 30;
num_spheres = 2;

p_1 = [2.9582; -2.8459; 4.6247];
p_2 = [1.4697; -0.7578; 4.9718];
r_1 = 1.6931;
r_2 = 0.9871;

coll = 0;
dist = sqrt((p_1(1) - p_2(1))^2 + (p_1(2) - p_2(2))^2 + (p_1(3) - p_2(3))^2);
if dist < r_1 + r_2
    coll = 1;
end

coll
%}

%% HW5.1.2. Are two spheres in collision (n times)
%{
clear all;
close all;
clc;

p = [0.4504 -0.6821 -4.6806 -0.0858 -3.5389 4.2371 -4.3190 0.2032 -3.2352 0.3121 2.9801 1.0264 -4.3716 -0.4646; 0.1461 -3.9892 -3.5946 -0.7882 2.2870 2.1076 2.7305 -3.0042 -0.6808 1.3132 1.3583 1.9311 -2.8810 0.4227; 1.8891 4.2836 -3.9585 3.3412 -1.2797 -2.0924 2.5428 -0.6937 0.5635 0.7037 -0.4009 -2.3728 -0.8934 -1.1963];
r = [1.0314 2.0305 1.7723 2.7808 1.5751 2.5295 2.7903 1.6149 1.6477 1.4015 2.7284 2.3226 2.9371 1.6611];
q = [-2.9053; 1.0769; -0.7542];
s = 1.7896;


numSpheres = size(p);
numSpheres = numSpheres(2);
coll = zeros(1, numSpheres);

for i = 1:numSpheres
    dist = sqrt((p(1,i) - q(1))^2 + (p(2,i) - q(2))^2 + (p(3,i) - q(3))^2);
    if (dist <= r(i)+s)
        %fprintf("in collision\n")
        coll(1,i) = 1;
    else
        %fprintf("not in collision\n")
        coll(1,i) = 0;
    end
end

mat2str(coll)
%}

%% HW5.1.3. Place bounding volumes for a given set of joint variables
%{
clear all;
clc;

num_spheres = 8;

p = zeros(3, num_spheres);

%{
p(:,1) = [0;0;0];
p(:,2) = [0;0;0];
p(:,3) = [0;0;0];
p(:,4) = [0;0;0];
p(:,5) = [0;0;0];
p(:,6) = [0;0;0];
p(:,7) = [0;0;0];
p(:,8) = [0;0;0];
%}

% the center of each sphere starts at the points:
p(:,1) = [0;0;0];
p(:,2) = [0;4;0];
p(:,3) = [4;4;0];
p(:,4) = [6;2;0];
p(:,5) = [6;0;0];
p(:,6) = [6;-2;0];
p(:,7) = [6;-4;0];
p(:,8) = [8;-4;0];

% characterize the robot
S = [0.00000000 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 -1.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 1.00000000 1.00000000; 1.00000000 0.00000000 0.00000000 0.00000000 -2.00000000 -4.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 -6.00000000 -6.00000000; 0.00000000 -4.00000000 -2.00000000 1.00000000 0.00000000 0.00000000];
M = [0.00000000 0.00000000 1.00000000 8.00000000; 0.00000000 1.00000000 0.00000000 -4.00000000; -1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
theta = [2.52622176; -0.63669277; 2.86852085; -1.11911211; 2.39608025; 1.18927962];

z = ones(1, num_spheres);
pAugInit = [p;z];
T = 1;

% the first two spheres aren't moved by any joints
pAugFinal(:,1) = pAugInit(:,1);
pAugFinal(:,2) = pAugInit(:,2);

T = 1;


for i = 1:size(S)(1)
    trans = expm(skew4(S(:,i))*theta(i));
    T = T*trans;
    
    pAugFinal(:,i+2) = T * pAugInit(:, i+2);
end

pFinal = pAugFinal(1:3, :)

sol = mat2str(pFinal, 7);
%}

%% HW5.1.4. Detect self-collision at a given configuration
%{
clear all;
clc;

num_spheres = 8;

p = zeros(3, num_spheres);

%{
p(:,1) = [0;0;0];
p(:,2) = [0;0;0];
p(:,3) = [0;0;0];
p(:,4) = [0;0;0];
p(:,5) = [0;0;0];
p(:,6) = [0;0;0];
p(:,7) = [0;0;0];
p(:,8) = [0;0;0];
%}

% the center of each sphere starts at the points:
p(:,1) = [0;0;0];
p(:,2) = [0;2;0];
p(:,3) = [-2;2;0];
p(:,4) = [-4;0;0];
p(:,5) = [-4;2;0];
p(:,6) = [-6;0;0];
p(:,7) = [-6;-2;0];
p(:,8) = [-6;-4;0];


% characterize the robot
S = [0.00000000 0.00000000 -1.00000000 0.00000000 0.00000000 0.00000000; 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 1.00000000; 0.00000000 0.00000000 0.00000000 -1.00000000 0.00000000 -2.00000000; 0.00000000 0.00000000 0.00000000 0.00000000 -1.00000000 6.00000000; 0.00000000 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000];
M = [0.00000000 0.00000000 1.00000000 -6.00000000; 0.00000000 -1.00000000 0.00000000 -4.00000000; 1.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
theta = [1.33105147; -0.69380704; -1.40115613; -0.22242667; -0.17613492; 0.37377594];
r = 0.90000000;

r_vec = r*ones(1, num_spheres);

z = ones(1, num_spheres);
pAugInit = [p;z];
T = 1;

% the first two spheres aren't moved by any joints
pAugFinal(:,1) = pAugInit(:,1);
pAugFinal(:,2) = pAugInit(:,2);

T = 1;
for i = 1:size(S)(1)
    % move the center of each sphere by only the joints that affect it
    trans = expm(skew4(S(:,i))*theta(i));
    T = T*trans;
    pAugFinal(:,i+2) = T * pAugInit(:, i+2);
end

% now we have the centers of the spheres at the position described by theta
pFinal = pAugFinal(1:3, :);
coll = zeros(1,num_spheres*num_spheres);

for i = 1:num_spheres
    for j = 1:num_spheres
        % check if each sphere is colliding with any of the others, except itself
        if (i == j)
            continue
        else       
            x1 = norm(pFinal(:,i) - pFinal(:,j));
            x2 = r_vec(i)+r_vec(j);
            
            if (x1 <= x2)
                fprintf("in collision\n")
                coll(1,i) = 1;
                i
                j
            else
                %fprintf("not in collision\n")
                coll(1,i) = 0;
            end
        end
    end
end
%}

%% HW5.1.5. Detect self-collision at several different configurations
%{
% characterize the robot
S = [0.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 1.00000000 1.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 0.00000000 -1.00000000 1.00000000 1.00000000; 0.00000000 -2.00000000 0.00000000 0.00000000 0.00000000 0.00000000; 0.00000000 0.00000000 1.00000000 -2.00000000 4.00000000 4.00000000; 2.00000000 0.00000000 0.00000000 0.00000000 0.00000000 0.00000000];
M = [1.00000000 0.00000000 0.00000000 -4.00000000; 0.00000000 0.00000000 -1.00000000 0.00000000; 0.00000000 1.00000000 0.00000000 4.00000000; 0.00000000 0.00000000 0.00000000 1.00000000];
theta = [-1.86656724 2.37204890 -2.65068972 -1.78802579 -2.64508962 1.57653731 0.34588892 1.44334181 -1.46921664 -1.55042625 2.19139051 -2.89465290 -1.49634377 1.74599670; -2.39095292 3.01261682 1.89224508 -0.35975103 0.05198427 2.79280039 -1.26467397 0.57543858 0.25366371 -1.31127027 0.75411255 -1.69250238 2.91835849 0.70982798; -2.48146547 -0.54441168 1.15622108 -0.56549203 0.56510479 -0.29028194 -1.70623095 -2.52234627 -1.77616310 2.64326627 -0.60661122 -3.07920794 -1.19437352 -0.91739892; -0.05661234 2.06352922 -0.07015501 2.46702545 0.52403565 -1.66732273 -1.63690228 2.58302460 0.30004158 1.45829865 -0.67702913 -1.31471926 0.10427285 -0.29002820; 3.05178329 0.25996384 -0.17146022 1.03930606 -1.61755830 -2.78908706 0.80034466 1.21299685 1.50454628 -0.53473494 -2.04407028 -1.14713093 -2.03301958 -0.96662815; -2.13896085 2.43007170 -1.60994950 2.00028424 2.75515220 -0.75695204 2.28460225 0.27085229 -1.48738680 -0.90017561 -0.57132698 -2.03912889 -1.13761887 -1.84273953];
r = 0.90000000;

num_spheres = 8;
r_vec = r*ones(1, num_spheres);


% find the starting position of the center of each sphere
%p = zeros(3, num_spheres);

%{
p(:,1) = [0;0;0];
p(:,2) = [0;0;0];
p(:,3) = [0;0;0];
p(:,4) = [0;0;0];
p(:,5) = [0;0;0];
p(:,6) = [0;0;0];
p(:,7) = [0;0;0];
p(:,8) = [0;0;0];
%}

p(:,1) = [0;0;0];
p(:,2) = [2;0;0];
p(:,3) = [0;0;2];
p(:,4) = [-2;0;2];
p(:,5) = [-2;0;0];
p(:,6) = [-4;0;0];
p(:,7) = [-4;0;2];
p(:,8) = [-4;0;4];


% augment the points so we can use a transformation matrix
z = ones(1, num_spheres);
pAugInit = [p;z];
T = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start the loop to take the robot to each position described by the columns of theta
% and check for collision

% for each position, this will show if there are any collisions
coll = zeros(1,size(theta)(2));

for k = 1:size(theta)(2)
collFlag = 0;
thetaLocal = theta(:,k);

% move the joints by using the transformation matrix
% the first two spheres aren't moved by any joints
pAugFinal(:,1) = pAugInit(:,1);
pAugFinal(:,2) = pAugInit(:,2);

T = 1;
for i = 1:size(S)(1)
    % move the center of each sphere by only the joints that affect it
    trans = expm(skew4(S(:,i))*thetaLocal(i));
    T = T*trans;
    pAugFinal(:,i+2) = T * pAugInit(:, i+2);
end

% now we have the centers of the spheres at the position described by theta
pFinal = pAugFinal(1:3, :);

for i = 1:num_spheres
    for j = 1:num_spheres
        % check if each sphere is colliding with any of the others, except itself
        if (i == j)
            continue
        else       
            x1 = norm(pFinal(:,i) - pFinal(:,j));
            x2 = r_vec(i)+r_vec(j);
            %x1
            %x2
            if (x1 <= x2)
                %fprintf("in collision\n")
                collFlag = 1;
            else
                %fprintf("not in collision\n")
            end
        end
    end
end
%collFlag
%k
if collFlag == 1
    coll(k) = 1;
end

end

c = mat2str(coll)
%}




















