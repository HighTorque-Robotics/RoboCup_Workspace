function [extrinsic] = calc_extrinsic(Parameters, pitchRad, yawRad)

% world to plat, R means rotate
Xw2p = Parameters(1);
Yw2p = Parameters(2);
Zw2p = Parameters(3);

RXw2p = Parameters(4);
RYw2p = Parameters(5);
RZw2p = Parameters(6); % plat to camera
Xp2c = Parameters(7);
Yp2c = Parameters(8);
Zp2c = Parameters(9);

RXp2c = Parameters(10);
RYp2c = Parameters(11);
RZp2c = Parameters(12);

scaleYaw = Parameters(13);
scalePitch = Parameters(14);
biasYaw = Parameters(15);
biasPitch = Parameters(16);

pitch = (pitchRad + biasPitch) * scalePitch;
yaw = (yawRad + biasYaw) * scaleYaw;

% fprintf('pitch: %f, yaw: %f\n', pitch, yaw);

w2p = rotateZ(RZw2p) ...
    * rotateY(RYw2p) ...
    * rotateX(RXw2p) ...
    * dtranslate(-Xw2p, -Yw2p, -Zw2p);

p2c = rotateZ(RZp2c) ...
    * rotateY(RYp2c) ...
    * rotateX(RXp2c) ...
    * dtranslate(-Xp2c, -Yp2c, -Zp2c) ...
    * rotateY(-pitch) ...
    * rotateZ(-yaw);

extrinsic = [ 0 -1  0  0;
              0  0 -1  0;
              1  0  0  0;
              0  0  0  1;] * p2c * w2p;

end
