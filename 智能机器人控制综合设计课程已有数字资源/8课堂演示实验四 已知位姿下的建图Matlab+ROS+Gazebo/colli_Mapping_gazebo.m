%% 地图配置
% map = binaryOccupancyMap(11,11,10);
clear;
map = occupancyMap(11,11,20);
mapFigure = figure('Name','Unknown Map');
map.GridLocationInWorld = [-1 -1];
%% 参数
sampleTime = 0.5;             % 采样时间 s
r = rateControl(1/sampleTime);
%% ROS通信
cmdPub = rospublisher("/cmd_vel");
odomSub = rossubscriber("/odom");
scanSub = rossubscriber("/scan");
cmdMsg = rosmessage(cmdPub);
odomMsg = receive(odomSub);
scanMsg = receive(scanSub);
maxrange = 5;
initPose = readPose(odomMsg);
currentScan = lidarScan(scanMsg);
insertRay(map,initPose,currentScan,maxrange);
show(map);
title('Mapping')
pause(2);
%% 路径绘制
% poseList(:) = [];
poseList(1,:) = [initPose(1) initPose(2)];
idx = 1;
%% 跟踪器
% path = [initPose(1) initPose(2);-6 -2;-7 1;-5 -1.5;-2.5 -1.5;0 1;-2 -2;-1 -3;-3.5 -4.5;0 -6];
path = [initPose(1) initPose(2);1 2;1 7;2 5;3.5 5;7 7;6 4;7 3;4 2;7 0.5];
%path = [initPose(1) initPose(2);4 6; 6.5 12.5; 4 22; 12 14; 22 22; 16 12; 20 10; 14 6; 22 3];
robotGoal = path(end,:);
robotInitialLocation = path(1,:);
controller = controllerPurePursuit;%路径跟踪器
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3; %恒定线速度   0.2
controller.MaxAngularVelocity = 2;   %最大角速度    0.5
controller.LookaheadDistance = 0.5;   %探测距离         2
goalRadius = 0.1;%目标半径
distanceToGoal = norm(robotInitialLocation(1:2) - robotGoal(:)); %初始位置与目标点的距离
while( distanceToGoal > goalRadius )
    odomMsg = receive(odomSub);
    scanMsg = receive(scanSub);
    pose = readPose(odomMsg);
    currentScan = lidarScan(scanMsg);
    insertRay(map,pose,currentScan,maxrange);
    show(map);
    title('Mapping')
    robotCurrentPose = pose';
    [v,omega] = controller(robotCurrentPose);
    cmdMsg.Linear.X = v;
    cmdMsg.Angular.Z = -omega;                    % 此处取负原因暂时不明
    send(cmdPub,cmdMsg); 
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));    %求解距离
    idx = idx + 1;
    poseList(idx,:) = [robotCurrentPose(1) robotCurrentPose(2)]; 
	plotTrvec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrvec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'View', '2D', 'FrameSize', 0.3, 'Parent',mapFigure.CurrentAxes);
    waitfor(r);
end
cmdMsg.Linear.X = 0;
cmdMsg.Angular.Z = 0;
send(cmdPub,cmdMsg);
mapFigure;
hold on
plot(poseList(:,1),poseList(:,2),'b-','DisplayName','机器人路径');

   




