#1.3.1, 1.3.2
#{
theta = -2.55631536;
axis = [0.87887900; -0.44956431; -0.15957326];

A = axis2skew(axis);
I = eye(3);

R_1in0 = I+sin(theta)*A+(1-cos(theta))*A^2;
fprintf(mat2str(R_1in0, 7));
fprintf('\n');
#}
#1.3.3
#{
R = [0.85591005 0.46724300 -0.22158964; -0.44388781 0.88365562 0.14871572; 0.26529531 -0.02892633 0.96373319];
theta = acos(.5*(trace(R)-1));
A = 1/(2*sin(theta))*(R-R');
axis = skew2axis(A);

fprintf(mat2str(axis));
nl();
nl();
fprintf(mat2str(theta));
nl();
#}
#{
#1.3.4, 1.3.6
R_1in0 = [0.05419471 0.77903937 -0.62462836; 0.95113770 -0.23070505 -0.20521271; -0.30397370 -0.58298614 -0.75347671];
R_2in0 = [-0.93053925 0.16947806 0.32461346; 0.16640778 0.98534689 -0.03741593; -0.32619804 0.01920121 -0.94510642];

R = R_2in0*R_1in0';
theta = acos(.5*(trace(R)-1));
A = 1/(2*sin(theta))*(R-R');
axis_in0 = skew2axis(A);
axis_in2 = R_2in0'*axis_in0;

fprintf(mat2str(axis_in2))
nl();
fprintf(mat2str(theta))
nl();
#}
#1.3.14 Euler Angle Rotations about x,y,z axes by angles theta1, theta2, theta3
X = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
Y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Z = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];

#XZX
theta1 = 1.46;
theta2 = 1.70;
theta3 = -2.40;

R = euler_Z(theta1)*euler_X(theta2)*euler_Y(theta3);
fprintf(mat2str(R));
nl();

















