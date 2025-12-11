function [T] = dtranslate(dx, dy, dz)
T = [1 0 0 dx
     0 1 0 dy
     0 0 1 dz
     0 0 0 1];
end
