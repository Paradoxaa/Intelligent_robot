%%设置激光传感器模型和TurtleBot运动模型
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
%%设置里程表运动模型的噪声分布
%%motionModel = odometryMotionModel;
%motionModel.Noise = [0.2 0.2 0.2 0.2];
%%用现有的噪声参数显示粒子的分布。每个粒子都是预测姿势的假设
% showNoiseDistribution(motionModel, ...
%             'OdometryPoseChange', [0.5 0.1 0.25], ...
%             'NumSamples', 1000);
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;
% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'/base_link','/base_scan');
sensorTransform = getTransform(tftree,'/base_link', '/base_scan');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
%接收传感器测量值并发送速度命令
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
%创建ROS发布器以将速度命令发送到turtlebot
[velPub,velMsg] = ...
    rospublisher('/cmd_vel','geometry_msgs/Twist');
%初始化AMCL对象
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
%在amcl对象中分配MotionModel和SensorModel属性。
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
%仅当机器人的运动超过UpdateThresholds（更新阈值）时，粒子过滤器才会更新粒子，该阈值定义了[x，y，yaw]的最小位移以触发过滤器更新。这样可以防止由于传感器噪声而导致过于频繁的更新。在amcl.ResamplingInterval过滤器更新后，会进行粒子重采样。使用较大的数字会导致粒子耗竭变慢，但代价是粒子会聚速度变慢。
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
%使用初始姿势估计为定位配置AMCL对象
% amcl.ParticleLimits = [500 5000];
% amcl.GlobalLocalization = false;
%无初始位姿的先验信息
amcl.GlobalLocalization = true;
amcl.ParticleLimits = [500 50000];
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;
%设置示例HelperAMCLVisualization可以绘制地图并更新机器人在地图上的估计姿态，粒子和激光扫描读数。
visualizationHelper = ExampleHelperAMCLVisualization(map);
%机器人运动对于AMCL算法至关重要。在此示例中，我们使用ExampleHelperAMCLWanderer类随机驱动TurtleBot，该类使用controllerVFH类在环境内驱动机器人的同时避免障碍。
wanderHelper = ...
    ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);
%机器人四处移动时，AMCL算法会在每个时间步更新里程表和传感器读数。请等待几秒钟，然后初始化粒子并将其绘制在图中。在此示例中，我们将运行numUpdates AMCL更新。如果机器人无法收敛到正确的机器人姿势，请考虑使用更大的numUpdates。
numUpdates = 60;
i = 0;
while i < numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
    
    % Drive robot to next pose.
    wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end
    
end
