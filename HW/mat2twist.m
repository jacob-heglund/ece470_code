function v = mat2twist(V);
    %takes a matrix and turns it into a 6 dimensional vector (twist or screw axis)
          v = [V(3,2); V(1,3); V(2,1); V(1,4); V(2,4); V(3,4)];  
end