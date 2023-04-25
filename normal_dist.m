clc; clear; close all;

% 2023-03-29 HBY
% generate normal distibuted measurement data for calibration
% 1. given joint trajectories
% 2. compute mean and std of each joint
% 3. generate randome joint angles based on the statistics
% 4. check if end effector is within bounding box and certain range
% of angle variation

%% 0. Define Robot

% nominal DH
DH_nominal = ...
    [135 0 0 0 0;
    0 -pi/2 0 -pi/2 0;
    0 0 135 0 0;
    120 0 38 -pi/2 0;
    0 0 0 pi/2 0;
    70 pi 0 -pi/2 0]; % d theta a alpha

% DH following convention: theta,d,a,alpha
ndh = DH_nominal(:,[2 1 3 4]);

% define robot
for ii = 1:6
    nL(ii) = Revolute('d', ndh(ii,2), 'a', ndh(ii,3), 'alpha', ndh(ii,4),'offset',ndh(ii,1),'modified');
end

% tcp information
TCP = [-5.853503192	0.09048161127	146.1547325	173.9416558	85.61107535	5.6322697]; % x y z Rx Ry Rz
T_tool = XYZWPR2SE3(TCP); % SE(3)
% T_tool = eye(4,4);

nrobot = SerialLink(nL,'tool',T_tool,'name',"Final Robot");

%% 1. Generate Normal Distributed Data

mu = [12.682249, 38.749025, 24.033056, -89.276325, -96.694194, 59.585503];
joint15 = 1080*2;
joint6 = 1080*2;
sigma = [joint15 joint15 joint15 joint15 joint15 joint6];
sigma = diag(sigma);

% check the average configuration
figure()
nrobot.plot(mu*pi/180)
Z_avg = nrobot.fkine(mu*pi/180).a;

% generate data based on normal distribution
N = 20000; % number of generated thetas
n = 200; % number of thetas we want after pose check
thetas = mvnrnd(mu, sigma, N);
thetas = thetas * pi/180; % conversion from degree to radian

% check joint limit
ul = [175 90 70 170 115 180]* pi/180;
ll = [-175 -70 -135 -170 -115 -180]* pi/180;
% thetas = (ul+ll)/2 + 2*(rand(N,6)-0.5).*(ul-ll)/2;
%% 2. Check Position and Orientation Constraints

% define box and orientation constraints
box_c = nrobot.fkine(mu*pi/180).t;
box_l = 160/2;
box = [box_c - box_l, box_c + box_l];
angle = pi/2;

% check the constraints
idx = [];
points = [];
for ii = 1:N
    q = thetas(ii,:);
    T = nrobot.fkine(q);
    P = T.t; % position
    Z = T.a; % z direction of end effector
    points = [points P];
%     if q(6) > 180
%         q(6) = q(6)-360;
%     elseif q(6) < -180
%         q(6) = q(6) + 360;
%     end
    if all(q <= ul) && all(q >= ll) % check joint limit
        if all(P >= box(:,1)) && all(P <= box(:,2)) % check position
            if subspace(Z, Z_avg) <= angle % check orientation
                idx = [idx ii];
            end
        end
    end
end
disp(length(idx))


idx = randsample(idx,n);
points = points(:,idx);
thetas = thetas(idx,:)*180/pi; % from radian to degree
disp(size(thetas)) % check the number of data

% save the result
writematrix(thetas,'uniform_dist_data_0deg_160cube.txt','Delimiter','tab')

%% Figure Results

figure()
plot3(points(1,:),points(2,:),points(3,:),'.');
grid on
xlabel('x');ylabel('y');zlabel('z');
axis equal
xlim(box(1,:))
ylim(box(2,:))
zlim(box(3,:))
title('Cartesian Space Distribution (TCP)')


figure()
for ii = 1:6
    subplot(2,3,ii)
    histogram(thetas(:,ii),10)
    xlabel('angle [deg]')
end
sgtitle('Joint Space Distribution')

figure()
nrobot.plot(thetas*pi/180)