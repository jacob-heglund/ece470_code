function B = twist2mat(a)
          w = a(1:3);
          v = a(4:6);
          B = [vec2skew(w) v; 0 0 0 0];
end         