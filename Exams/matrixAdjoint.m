function T_adj = matrixAdjoint(T)
R = T(1:3, 1:3);
p = T(1:3, 4);
Z = zeros(3);
T_adj = [R Z; vec2skew(p)*R R];          
          
          
end