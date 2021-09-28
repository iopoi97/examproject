#include "odom.h"

ODOM::ODOM(){
    _encoders_sub = _nh.subscribe("/diff_wheels/vel", 1, &ODOM::encoders_sub_callback, this);
    _imu_sub= _nh.subscribe("/imu", 1, &ODOM::imu_sub_callback, this);
    _odom_pub = _nh.advertise<nav_msgs::Odometry>("/odom",1);
    _wl=0.0;
    _wr=0.0;
    _x=0.0;
    _y=0.0;
    _th=0.0;
    _first_odom=false;
}

void ODOM::encoders_sub_callback(std_msgs::Float32MultiArray  msg){
    if (msg.data.size() < 2) return;
    _wl= msg.data[0];
    cout<<_wl<<endl;
    _wr=msg.data[1];
    cout<<_wr<<endl;
}

void ODOM::imu_sub_callback(sensor_msgs::Imu msg){
    tf::Quaternion q (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, _th);
    if (_th>2*M_PI)
        _th=_th-2.0*M_PI;
    else if(_th<0.0) 
        _th=_th+2.0*M_PI;
}

void ODOM::compute_odom(){
     ros::Rate r(200);
    tf::TransformBroadcaster odom_broadcaster;
  long double dt;
  long double delta_phil;
  long double delta_phir;
  long double vk;
  long double wk;
  dt=1/200.0;
  ros::Time current_time, last_time;
    while(ros::ok()){
        delta_phir= _wr*dt;
        delta_phil= _wl*dt;
        vk= (0.1/(2*dt))*(delta_phir+delta_phil);
        wk=(0.1/(0.4*dt))*(delta_phir-delta_phil);

        //runge kutta  
        _x +=vk*dt*cos(_th+((wk*dt)/2));
        if(isnan(_x))
            _x=0;
        _y +=vk*dt*sin(_th+((wk*dt)/2));
        if(isnan(_y))
            _y=0;
        cout<<"x= "<<_x<<endl;
        cout<<"y= "<<_y<<endl;
        cout<<"th= "<<_th<<endl<<endl;
        
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(_th);
        
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "robot_footprint";
        odom_trans.transform.translation.x = _x;
        odom_trans.transform.translation.y = _y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
        
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        
        //set the position
        odom.pose.pose.position.x = _x;
        odom.pose.pose.position.y = _y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "robot_footprint";
        odom.twist.twist.linear.x = vk*cos(_th);
        odom.twist.twist.linear.y = vk*sin(_th);
        odom.twist.twist.angular.z = wk;
        
        //publish the message
        _odom_pub.publish(odom);
        r.sleep();
    }
}

void ODOM::run(){

    boost::thread odom_t( &ODOM::compute_odom, this);

    ros::spin();
}

int main(int argc, char** argv){
 ros::init(argc,argv, "odom_node");

    ODOM odom;
    odom.run();


    return 0;
}
