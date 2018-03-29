function S = vec2twist(s)
          w = s(1:3);
          v = s(4:6);
          R = vec2skew(w);
          
          S = [R v; 0 0 0 0];
          end