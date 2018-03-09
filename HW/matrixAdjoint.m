function B = matrixAdjoint(A)
 R = A(1:3, 1:3);
 p = A(1:3, 4);
 B = [R zeros(3); axis2skew(p)*R R];
 end