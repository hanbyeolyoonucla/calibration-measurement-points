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
TCP = [-0.4590164281	-0.9164464634	161.3819234	168.2432879	85.64040621	11.58103583]; % x y z Rx Ry Rz
T_tool = XYZWPR2SE3(TCP); % SE(3)
% T_tool = eye(4,4);

nrobot = SerialLink(nL,'tool',T_tool,'name',"Final Robot");

%% 1. Generate Normal Distributed Data

mu = [4.333911931	13.46457946	39.44606267	-89.63181963	-88.44431986	76.71547582];
sigma = [90 90 90 90 90 1080];
sigma = diag(sigma);

% check the average configuration
figure()
nrobot.plot(mu*pi/180)
Z_avg = nrobot.fkine(mu*pi/180).a;

% generate data based on normal distribution
N = 1000; % number of generated thetas
n = 80; % number of thetas we want after pose check
thetas = mvnrnd(mu, sigma, N);
thetas = thetas * pi/180; % conversion from degree to radian

%% 2. Check Position and Orientation Constraints

% define box and orientation constraints
box_c = [122 241 189]';
box_l = 80/2;
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
    if all(P >= box(:,1)) && all(P <= box(:,2)) % check position
        if subspace(Z, Z_avg) <= angle % check orientation
            idx = [idx ii];
        end
    end
end
disp(length(idx))


idx = randsample(idx,n);
points = points(:,idx);
thetas = thetas(idx,:)*180/pi; % from radian to degree
disp(size(thetas)) % check the number of data

% save the result
writematrix(thetas,'normal_dist_data.txt','Delimiter','tab')

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