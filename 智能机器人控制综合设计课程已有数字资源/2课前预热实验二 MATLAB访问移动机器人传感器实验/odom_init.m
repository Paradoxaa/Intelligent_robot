odomresetpub = rospublisher('/mobile_base/commands/reset_odometry'); % Reset odometry
odomresetmsg = rosmessage(rostype.std_msgs_Empty);
send(odomresetpub,odomresetmsg);