velocity = 0.2;     % meters per second
%为/ cmd_vel主题以及包含速度值的相应消息创建发布者。
robotCmd = rospublisher("/cmd_vel") ;
velMsg = rosmessage(robotCmd);
%根据速度变量设置机器人的前进速度（沿X轴），然后将命令发布到机器人。让它移动片刻，然后将其停止。
velMsg.Linear.X = velocity;%线速度
%velMsg.Angular.Z =velocity;%角速度
send(robotCmd,velMsg)
pause(10)
velMsg.Linear.X = 0;
velMsg.Angular.Z =0;
send(robotCmd,velMsg)
%要查看速度主题发布的消息的类型，请执行以下操作：
%rostopic type /cmd_vel
%该主题期望消息类型为geometry_msgs / Twist，这恰好是上面创建的velMsg的类型。
%要查看哪些节点正在发布和订阅给定主题，请使用以下命令：rostopic info TOPICNAME。以下命令列出了速度主题的发布者和订阅者。 MATLAB被列为发布者之一。
%接收机器人的位置和方向，TurtleBot使用/ odom主题发布其当前位置和方向（统称为位姿）。由于TurtleBot没有配备GPS系统，因此该姿势将与机器人首次打开时的姿势有关。
%创建里程计消息的订阅者
%等待订阅者返回数据，然后提取数据并将其分配给变量x，y和z：
odomMsg = receive(odomSub,3);
pose = odomMsg.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;
%TurtleBot的方向在姿势的Orientation属性中作为四元数存储。使用quat2eul（机器人系统工具箱）将其转换为更方便的欧拉角表示。要以度为单位显示机器人的当前方向theta，请执行以下几行。
lidarSub = rossubscriber("/scan");
%订阅激光雷达主题后，等待数据，然后使用绘图显示它
scanMsg = receive(lidarSub);
figure
plot(scanMsg)
%要在机器人短时间转动时连续显示更新的激光雷达扫描，请使用以下while循环：
% velMsg.Angular.Z = velocity;
% send(robotCmd,velMsg)
% tic
% while toc < 20
%   scanMsg = receive(lidarSub);
%   plot(scanMsg)
% end
% velMsg.Angular.Z = 0;
% send(robotCmd,velMsg)
