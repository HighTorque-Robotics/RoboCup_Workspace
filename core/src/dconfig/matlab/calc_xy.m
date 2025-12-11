function [xy] = calc_xy(cameraMatrix, extrinsic, u, v)


M = cameraMatrix * extrinsic;

im = inv(M);

C = im(3,:);

s = -C(4) / (u * C(1) + v * C(2) + C(3));

foo = [s * u; s * v; s; 1];

x = im(1,:) * foo;
y = im(2,:) * foo;

[xy] = [x;y];

end
