% Created on: June 2, 2017
%    Author: Wenxing Mei <mwx36mwx@gmail.com>
function [PointImage] = projection(Parameters, pitch, yaw, x, y) % xreal, yreal
% Project a point from world onto image plane
camMatrix = evalin('base', 'camMatrix');
extrinsic = calc_extrinsic(Parameters, pitch, yaw);


PointImage = camMatrix * extrinsic * [x;y;0;1];

% disp(PointImage);
PointImage = [PointImage(1) / PointImage(3);
              PointImage(2) / PointImage(3);];
end
