function B = matAdjoint(A)
R = A(1:3, 1:3);
p = A(1:3,4);
Z = zeros(3);
B = [R Z; vec2skew(p)*R R]; 
end