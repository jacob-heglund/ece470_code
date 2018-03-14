function V = vec2mat(v);
    %takes vector of the form [w (angular velocity), v (linear velocity)]
    %where w and v are 3-dimensional vectors outputs a matrix
    %that is a concatination of a skew symmetric matrix and linear velocity
          V = [0    -v(3)   v(2)   v(4); 
               v(3)  0     -v(1)   v(5)
              -v(2)  v(1)   0      v(6)
               0     0      0      0];
end          