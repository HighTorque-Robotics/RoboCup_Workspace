function [R] = rotateX(rx)
% https://en.wikipedia.org/wiki/Rotation_matrix
R  = [         1         0         0         0;
               0   cos(rx)  -sin(rx)         0;
               0   sin(rx)   cos(rx)         0;
               0         0         0         1;];
end
