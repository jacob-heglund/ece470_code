function A = axis2skew(axis)
 A = [0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
 end 