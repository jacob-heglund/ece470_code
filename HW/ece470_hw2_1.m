%% HW2.1.1. Find the pose that is produced by a given twist after 1 second
%{
clear all;
clc;

M = [0.92664982 0.03179372 -0.37457879 -1.47107926; 0.37294898 0.04739379 0.92664064 2.99602796; 0.04721406 -0.99837017 0.03206001 1.97223030; 0.00000000 0.00000000 0.00000000 1.00000000];
V_b = [0.06745537; -0.89021386; -0.80092742; -0.54270421; 0.46089711; 0.48532398];

%for a body twist
V = twist_vec2mat(V_b);
T_1in0 = M*expm(V);

%for a spatial twist
%V = twist_vec2mat(V_s)
%T_1in0 = expm(V)*M;

mat2str(T_1in0, 6)
%}
%% HW2.1.2. Find the pose that is produced by a given screw
%{
clear all;
clc;

M = [-0.19451833 -0.22800119 -0.95403253 -2.22264000; -0.28006095 0.94503422 -0.16874890 -1.49015505; 0.94006834 0.23436251 -0.24768071 2.18791521; 0.00000000 0.00000000 0.00000000 1.00000000];
B = [-0.11460731; 0.72945439; -0.67436004; -0.60880413; -0.28176054; -0.45111490];
theta = 1.08636341;

%use for a body screw motion
%B_mat = vec2mat(B);
%T_1in0 = M*expm(B_mat*theta);

%use for a spatial screw motion
%S_mat = twist_vec2mat(S);
%T_1in0 = expm(S_mat*theta)*M;

mat2str(T_1in0, 6)
%}
%% HW2.1.3. Find the screw that would produce the given motion
%{
clear all;
clc;

M = [0.52945094 -0.21320987 0.82111098 0.92348589; 0.84656626 0.07021911 -0.52763135 0.82937532; 0.05483853 0.97447977 0.21767386 -1.95161396; 0.00000000 0.00000000 0.00000000 1.00000000];
theta = 0.63667188;
a_in0 = [-0.12924360; 0.18811208; 0.97360667];

S = [0; 0; 0; a_in0];

mat2str(S)
%}
%% HW2.1.4. Find the matrix representation of a twist
%{
clear all;
clc;
V = [5; 1; 8; 6; 10; 3];
v = vec2mat(V);
mat2str(v, 6)
%}

%% HW2.1.5. Find a twist given its matrix representation
%{
clear all;
clc;

V_mat = [0 -4 6 1; 4 0 -2 -4; -6 2 0 5; 0 0 0 0];

v = mat2vec(V_mat);
mat2str(v)
%}
%% HW2.1.6. Find the adjoint of a Homogeneous transformation matrix
%{
clear all;
clc;
T = [0.46017911 0.81343949 -0.35574061 -0.42267781; 0.84168627 -0.27222105 0.46632598 0.55590379; 0.28248789 -0.51401546 -0.80993129 0.36448191; 0.00000000 0.00000000 0.00000000 1.00000000];

R = T(1:3, 1:3);
p = [T(1:3, 4)];
p_mat = axis2skew(p);
Z = zeros(3);
adj_T = [R Z; p_mat*R R];

mat2str(adj_T, 6)
%}
%% HW2.1.7. Find spatial twist given body twist
%{
clear all;
clc;

V_01in1 = [-0.79522114; 0.41069857; -0.00328018; 0.41447237; 0.52362975; 0.77867807];
T_1in0 = [-0.00505507 -0.01752049 0.99983373 0.34523799; -0.96659366 0.25631338 -0.00039553 0.43237301; -0.25626383 -0.96643494 -0.01823087 -0.34524593; 0.00000000 0.00000000 0.00000000 1.00000000];

T_adj = mat2adj(T_1in0);
V_01in0 = T_adj*V_01in1;
mat2str(V_01in0, 6)
%}

%% HW2.1.8. Find body twist given spatial twist 
%{
clear all;
clc;

V_01in0 = [-0.86585894; 0.46085447; 0.16304392; -0.09579015; -0.08664216; 0.31675393];
T_1in0 = [0.72605881 -0.68759403 -0.00728342 0.56201717; 0.60951329 0.64844133 -0.45608924 -0.36614827; 0.31832711 0.32670827 0.88990424 0.33901765; 0.00000000 0.00000000 0.00000000 1.00000000];

T_adj = mat2adj(T_1in0);

V_01in1 = inv(T_adj)*V_01in0;

mat2str(V_01in1)
%}

%end of line
