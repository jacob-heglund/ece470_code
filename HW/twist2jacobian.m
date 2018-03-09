%TODO: everything, this is broken as is
function J = twist2jacobian(s, theta)
          j1 = s(:,1);          
          J = [j1];
          if size(s)(2) > 1
            for i = 2:size(s)(2)
                
                co1 = s(:,i-1);
                x = expm(vec2mat(co1)*theta(i-1));
                
                jNew = matrixAdjoint(x)*s(:,i);
            
                J = [J jNew];
            
            end   
          end
          