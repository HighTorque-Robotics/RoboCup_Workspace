function [R] = rotateY(ry)
% https://en.wikipedia.org/wiki/Rotation_matrix
R  = [   cos(ry)         0   sin(ry)         0;
               0         1         0         0;
        -sin(ry)         0   cos(ry)         0;
               0         0         0         1;];
end
