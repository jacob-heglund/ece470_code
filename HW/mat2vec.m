function v = mat2vec(V);
    %takes a matrix and turns it into a 6 dimensional vector
          v = [V(3,2); V(1,3); V(2,1); V(1,4); V(2,4); V(3,4)];  
end