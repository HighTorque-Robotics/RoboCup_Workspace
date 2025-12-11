clear;
clc;
close all;



A = load('~/dancer-camera/bot5_ext.txt');
intrinsic = load('~/dancer-camera/bot5_in.txt');

% intrinsic = [360.7369784388869 360.6518377419487 625.1801051372124 466.7405779251932]; % bot1 2019/04/02
% intrinsic = [363.5620076330466 363.5194666723943 638.7477442613776 479.4042697060596]; % bot2 2018/06/17
% intrinsic = [359.9923885279374 359.8709434581638 573.5747484214887 424.6316344676307]; % bot4 2018/06/16
% intrinsic = [361.0845745220181 360.8742770020000 579.2496411994034 433.3033638629275]; % bot5 2018/06/13
% intrinsic = [362.7887946814019 362.9034344274426 565.4855662445698 419.926389392531]; % bot5 2018/06/17
% intrinsic = [366.6406824362387 366.2481176260064 584.7534259589613 417.8286362064586]; % bot6 2018/06/17
           
fx = intrinsic(1);
fy = intrinsic(2);
cx = intrinsic(3);
cy = intrinsic(4);

camMatrix = [fx 0 cx 0;
                0 fy cy 0;
                0  0  1 0;
                0  0  0 1];

% Xw2p
% Yw2p
% Zw2p

% RXw2p
% RYw2p
% RZw2p

% Xp2c
% Yp2c
% Zp2c

% RXp2c
% RYp2c
% RZp2c

% scaleYaw
% scalePitch
% biasYaw
% biasPitch

%           x       y       z       rx        ry      rz        x        y       z      rx       ry       rz     s    s     b     b
para0  = [  0;      0;     60;       0;       0;       0;       0;       0;      10;      0;       0;       0;    1;   1;    0;    0];    
lb     = [ -20;     -10;     40;    -0.2;    -0.4;    -0.2;      -10;      -10;      0;   -0.1;    -0.1;    -0.1;  0.8; 0.8;  -10 / 180 * pi;    -10 / 180 * pi];
ub     = [  20;      10;     70;     0.2;     0.4;     0.2;       10;       10;      20;   0.1;     0.1;     0.1;  1.3; 1.3;    10 / 180 * pi;    10 / 180 * pi];

% lb     = [ -0;     -0;     52;    -0.0;    -0.1;    -0.0;      -1;      -1;      6;      0;    -0.1;       0;    1;   1;    0;    0];
% ub     = [  0;      0;     54;     0.0;     0.1;     0.0;       1;       1;      8;      0;     0.1;       0;    1;   1;    0;    0];

% TODO(MWX): How to test correctness of projection function

uv = projection(para0, 45 / 180 * pi, 0, 450, 130);

disp(uv);
% uv = projection(para0, 0, 0, 450, -130);

% disp(uv);

options = optimset('Display','iter-detailed','Algorithm','interior-point','FunValCheck','on',...
    'TolFun',10^-6,'LargeScale','off','TolX',10^-6,'MaxFunEvals',10^6,...
   'MaxIter',10000);

[respara, reserror, exitflag, output] = fmincon(@errorfunc, para0, [], [], [], [], lb, ub, [], options);
% disp(respara);
for i = 1 : size(respara, 1)
    if (i == size(respara, 1))
        disp(num2str(respara(i,1)));
    else 
        disp([num2str(respara(i,1)) ',']);
    end
end

%% Plot
hold on;
for i = 1 : size(A, 1)
    plot(A(i, 5), A(i, 6), 'b*')
end


for i = 1 : size(A, 1)
    pitch = A(i, 2) / 180 * pi;
    yaw = A(i, 1) / 180 * pi;
    u = A(i, 3);
    v = A(i, 4);
    [xy] = calc_xy(camMatrix, calc_extrinsic(respara, pitch, yaw), u, v);
    plot(xy(1), xy(2), 'r*');
end
