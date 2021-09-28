bag=rosbag('/home/paolino/bagfiles/subset.bag');

 wh=select(bag,'Topic','/diff_wheels/vel');
 whStructs=readMessages(wh);
  vect=[1, 1];
  vect=vect';
  vect=whStructs{1,1}.Data;
  for i=2:276528
  vect=[vect whStructs{i,1}.Data];
  end
  wh1=vect(1,:);
  wh2=vect(2,:);
 
 wh_gaz=select(bag,'Topic','/joint_states');
 wh_gazStruct=readMessages(wh_gaz);
 vect_gaz=[1, 1];
 vect_gaz=vect_gaz';
 vect_gaz=wh_gazStruct{1,1}.Velocity;
 for i=2:18515
 vect_gaz=[vect_gaz wh_gazStruct{i,1}.Velocity];
 end
 wh1_gaz=vect_gaz(1,:);
 wh2_gaz=vect_gaz(2,:);



imus=select(bag,'Topic','/imu');
imusstruct=readMessages(imus,'DataFormat','struct');
phi=cellfun(@(m) double(m.Orientation.X),imusstruct);
theta=cellfun(@(m) double(m.Orientation.Y),imusstruct);
psi=cellfun(@(m) double(m.Orientation.Z),imusstruct);
w=cellfun(@(m) double(m.Orientation.W),imusstruct);
[yaw,pitch,roll]=quat2angle([w,phi,theta,psi]);


vel_1=select(bag,'Topic','/right_wheel_velocity_controller/command');
vel_1struct=readMessages(vel_1,'DataFormat','struct');
vel_1val=cellfun(@(m) double(m.Data),vel_1struct);


vel_2=select(bag,'Topic','/left_wheel_velocity_controller/command');
vel_2struct=readMessages(vel_2,'DataFormat','struct');
vel_2val=cellfun(@(m) double(m.Data),vel_2struct);

err=select(bag,'Topic','/err');
errStruct=readMessages(err,'DataFormat','struct');
err_values=cellfun(@(m) double(m.Data),errStruct);

odom=select(bag,'Topic','/odom');
odomStructs = readMessages(odom,'DataFormat','struct');
x_odom=cellfun(@(m) double(m.Pose.Pose.Position.X),odomStructs);
y_odom=cellfun(@(m) double(m.Pose.Pose.Position.Y),odomStructs);

odom_g=select(bag,'Topic','/gazebo_odom');
odom_gStructs = readMessages(odom_g,'DataFormat','struct');
x_gaz=cellfun(@(m) double(m.Pose.Pose.Position.X),odom_gStructs);
y_gaz=cellfun(@(m) double(m.Pose.Pose.Position.Y),odom_gStructs);
phi_gaz=cellfun(@(m) double(m.Pose.Pose.Orientation.X),odom_gStructs);
theta_gaz=cellfun(@(m) double(m.Pose.Pose.Orientation.Y),odom_gStructs);
psi_gaz=cellfun(@(m) double(m.Pose.Pose.Orientation.Z),odom_gStructs);
w_gaz=cellfun(@(m) double(m.Pose.Pose.Orientation.W),odom_gStructs);
[yaw_gaz,pitch_gaz,roll_gaz]=quat2angle([w_gaz,phi_gaz,theta_gaz,psi_gaz]);
%yaw_gaz=[zeros(3714,1);yaw_gaz];
rng(0,'twister');
r = randi([1, 61642],1,8);
x_gaz(r)=[];
x_gaz=x_gaz';
y_gaz(r)=[];
y_gaz=y_gaz';

plot(x_odom,y_odom,'Linewidth',1.2);
axis([-10 10 -10 10]);
 title('Robot path in the environment');
 xlabel('x(t)[m]');
 ylabel('y(t)[m]');
 hold on;
 plot(x_gaz,y_gaz,'Linewidth',1.2);
 legend('Odometry pose','Simulator pose'),
 hold on;
 I=imread('/home/paolino/Immagini/willo.png');
 h=image(xlim,-ylim,I);
 uistack(h,'bottom');
 figure;
 x_odom=x_odom';
 y_odom=y_odom';

odom_1=select(bag,'Topic','/odom1');
odom_1Structs = readMessages(odom_1,'DataFormat','struct');
x_odom_1=cellfun(@(m) double(m.Pose.Pose.Position.X),odom_1Structs);
y_odom_1=cellfun(@(m) double(m.Pose.Pose.Position.Y),odom_1Structs);
rng(0,'twister');
r = randi([1, 61641],1,7);
x_odom_1(r)=[];
x_odom_1=x_odom_1';
y_odom_1(r)=[];
y_odom_1=y_odom_1';



pose_gaz=[x_gaz; y_gaz];
pose_odom=[x_odom; y_odom];
pose_odom_1=[x_odom_1; y_odom_1];

time=[0:(odom.EndTime-odom.StartTime)/odom.NumMessages:(odom.EndTime-odom.StartTime)-((odom.EndTime-odom.StartTime)/odom.NumMessages)];
ex=x_gaz-x_odom;
ey=y_gaz-y_odom;
ex_1=x_gaz-x_odom_1;
ey_1=y_gaz-y_odom_1;
plot(time,ex,'Linewidth',1.2);
title('Position Error between Odometry with IMU and the Simulator pose');
xlabel('Time [s]');
ylabel('e(t) [m]');
hold on;
plot(time,ey,'Linewidth',1.2);
legend('$e_x(t)$','$e_y(t)$','Interpreter','latex','FontSize',15);
figure;
for i=1:length(ex)
    nor(i)=sqrt(ex(i)^2+ey(i)^2);
    nor_1(i)=sqrt(ex_1(i)^2+ey_1(i)^2);
end
plot(time,nor,'Linewidth',1.2);
title('Norm of position error between Odometry with IMU and the Simulator pose');
xlabel('Time [s]');
ylabel('||e(t)|| [m]');
figure;
plot(time,nor_1,'Linewidth',1.2);
title('Norm of position error between passive Odometry and the Simulator pose');
xlabel('Time [s]');
ylabel('||e(t)|| [m]');
figure;
time1=[0:(err.EndTime-err.StartTime)/err.NumMessages:(err.EndTime-err.StartTime)-((err.EndTime-err.StartTime)/err.NumMessages)];
plot(time1,err_values,'Linewidth',1.2);
title('Tracking norm error');
xlabel('Time [s]');
ylabel('||e(t)|| [m]');
figure
time2=[0:(imus.EndTime-imus.StartTime)/imus.NumMessages:(imus.EndTime-imus.StartTime)-((imus.EndTime-imus.StartTime)/imus.NumMessages)];
plot(time2,yaw,'Linewidth',1.2);
title('Yaw angle measured by the IMU vs Simulator');
xlabel('Time [s]');
ylabel('Yaw [rad]');
hold on;
time3=[0:(odom_g.EndTime-odom_g.StartTime)/odom_g.NumMessages:(odom_g.EndTime-odom_g.StartTime)-((odom_g.EndTime-odom_g.StartTime)/odom_g.NumMessages)];
plot(time3,yaw_gaz,'Linewidth',1.2);
legend('IMU','Simulator');
figure;
time4=[0:(vel_1.EndTime-vel_1.StartTime)/vel_1.NumMessages:(vel_1.EndTime-vel_1.StartTime)-((vel_1.EndTime-vel_1.StartTime)/vel_1.NumMessages)];
plot(time4,vel_1val,'Linewidth',1.2);
title('Right and left wheels velocities commanded by the I/O linearization controller');
xlabel('Time [s]');
ylabel('$\omega$ [rad/s]','Interpreter','latex');
hold on;
plot(time4,vel_2val,'Linewidth',1.2);
legend('$\omega_r$','$\omega_l$','Interpreter','latex');
figure
time5=[0:(wh.EndTime-wh.StartTime)/wh.NumMessages:(wh.EndTime-wh.StartTime)-((wh.EndTime-wh.StartTime)/wh.NumMessages)];
time6=[0:(wh_gaz.EndTime-wh_gaz.StartTime)/15429:(wh_gaz.EndTime-wh_gaz.StartTime)-((wh_gaz.EndTime-wh_gaz.StartTime)/15429)];
plot(time5, wh1,'Linewidth',1.2);
title('Right wheel velocity by the Plugin vs Joint States');
hold on;
plot(time6, wh1_gaz,'Linewidth',1.2);
xlabel('Time [s]');
ylabel('$\omega_r$ [rad/s]','Interpreter','latex');
legend('Plugin','Joint States');
figure;
plot(time5,wh2,'Linewidth',1.2);
title('Left wheel velocity by the Plugin vs Joint States');
xlabel('Time [s]');
ylabel('$\omega_l$ [rad/s]','Interpreter','latex');
hold on;
plot(time6,wh2_gaz,'Linewidth',1.2);
legend('Plugin','Joint States');

