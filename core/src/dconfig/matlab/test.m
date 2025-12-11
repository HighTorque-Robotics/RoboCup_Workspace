fx = 360.591090231311;
fy = 360.4918824799427;
cx = 624.7131585594641;
cy = 496.0890520277582;

% -15 0 417 393 450 130
% -60 45 540 610 10 -10
% 0 60 560 491 10 10



CameraMatrix = [fx 0 cx 0;
                0 fy cy 0;
                0  0  1 0;
                0  0  0 1]

% para0 = evalin('base', 'para0');
para0 = [
        5.4525;
        0.7083;
        49.5929;
        -0.0060;
        -0.3331;
        0.0072;
        2.4806;
        -0.6095;
        6.0255;
        -0.0002;
        0.0261;
        -0.0313;
        0.9839;
        1.1817;
        -0.0056
        0.0633];

pitch = 60;
yaw = 0;

pitch = pitch / 180.0 * pi;
yaw = yaw / 180.0 * pi;

ex = calc_extrinsic(para0, pitch, yaw)

A = CameraMatrix * ex

x = 10;
y = 10;


fuck = A * [x;y;0;1];

u = fuck(1) / fuck(3)
v = fuck(2) / fuck(3)



[xy] = calc_xy(CameraMatrix, ex, 560, 491)

%fuu = inv(foo) * fuck;

% u = 746;
% v = 533;
% 
% bar = inv(foo)
% 
% C = bar(3,:);
% 
% s = -C(4) / (u * C(1) + v * C(2) + C(3));
% 
% x = bar(1,:) * [s*u;s*v;s;1];
% y = bar(2,:) * [s*u;s*v;s;1];
% disp(x);
% disp(y);
