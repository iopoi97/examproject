bag=rosbag('/home/paolino/bagfiles/subset1.bag');
odom_g=select(bag,'Topic','/gazebo_odom');
odom_gStructs = readMessages(odom_g,'DataFormat','struct');
x_gaz=cellfun(@(m) double(m.Pose.Pose.Position.X),odom_gStructs);
y_gaz=cellfun(@(m) double(m.Pose.Pose.Position.Y),odom_gStructs);

odom_1=select(bag,'Topic','/odom1');
odom_1Structs = readMessages(odom_1,'DataFormat','struct');
x_odom_1=cellfun(@(m) double(m.Pose.Pose.Position.X),odom_1Structs);
y_odom_1=cellfun(@(m) double(m.Pose.Pose.Position.Y),odom_1Structs);

plot(x_gaz,y_gaz);
hold on;
plot(x_odom_1,y_odom_1);
title('Path according to passive Odometry and simulator');
xlabel('x(t) [m]');
ylabel('y(t) [m]');
legend('Simulator','Passive Odometry');