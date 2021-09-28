#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include "gazebo_msgs/ModelStates.h"
#include "tf/tf.h"
#include <math.h>
using namespace std;

float wr;
float wl;
double th=0.000000;
//nav_msgs::Odometry odom1; just to plot gazebo pose


//to take the wheels velocities

void encoders_sub_callback(const std_msgs::Float32MultiArray & msg){
    if (msg.data.size() < 2) return;
    wl= msg.data[0];
    cout<<wl<<endl;
    wr=msg.data[1];
    cout<<wr<<endl;
}



//to take yaw value from the IMU sensor

void imu_sub_callback(const sensor_msgs::Imu & msg){
    tf::Quaternion q (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    double roll, pitch;
    tf::Matrix3x3(q).getRPY(roll, pitch, th);
    if (th>2*M_PI)
        th=th-2.0*M_PI;
    else if(th<0.0) 
        th=th+2.0*M_PI;
}




/* Just to plot gazebo pose
void gazebo_cb(gazebo_msgs::ModelStates msg){
   tf::TransformBroadcaster odom_broadcaster;
   double x=msg.pose[24].position.x;
   double y=msg.pose[24].position.y;
   tf::Quaternion q (msg.pose[24].orientation.x, msg.pose[24].orientation.y, msg.pose[24].orientation.z, msg.pose[24].orientation.w);
   double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
   double x_d=msg.twist[24].linear.x;
   double y_d=msg.twist[24].linear.y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "Odom";
    odom_trans.child_frame_id = "robot_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.1;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    //next, we'll publish the odometry message over ROS
    odom1.header.stamp = ros::Time::now();
    odom1.header.frame_id = "odom";
    //set the position
    odom1.pose.pose.position.x = x;
    odom1.pose.pose.position.y = y;
    odom1.pose.pose.position.z = 0.0;
    odom1.pose.pose.orientation = odom_quat;
    odom1.child_frame_id = "robot_footprint";
    odom1.twist.twist.linear.x = x_d;
    odom1.twist.twist.linear.y = y_d;
    odom1.twist.twist.angular.z =0.0;
}
*/




int main(int argc, char** argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Subscriber encoders_sub= n.subscribe("/diff_wheels/vel",1,encoders_sub_callback);
    ros::Subscriber imu_sub= n.subscribe("/imu",1,imu_sub_callback);
    //ros::Subscriber gazebo_sub = n.subscribe("/gazebo/model_states", 1, gazebo_cb);
    //ros::Publisher gazebo_pub = n.advertise<nav_msgs::Odometry>("gazebo_odom",1);
    tf::TransformBroadcaster odom_broadcaster;
    long double x=0.0;
    long double y=0.0;
    long double dt;
    long double delta_phil;
    long double delta_phir;
    long double vk;
    long double wk;
    wr=0.0;
    wl=0.0;
    ros::Rate r(200);
    dt = 1.0/200.0;
  
    while(ros::ok()){
    
        //compute odometry in a typical way given the velocities of the wheels
        delta_phir= wr*dt;
        
        delta_phil= wl*dt;
        vk= (0.1/(2.0*dt))*(delta_phir+delta_phil);
        wk=(0.1/(0.4*dt))*(delta_phir-delta_phil);

        //runge kutta
        x +=(vk*dt*cos(th+((wk*dt)/2.0)));
        if(isnan(x))
            x=0.0;
        y += (vk*dt*sin(th+((wk*dt)/2.0)));
        if(isnan(y))
            y=0.0;
        cout<<"x= "<<x<<endl;
        cout<<"y= "<<y<<endl;
        cout<<"th_grad= "<<th*180.0/M_PI<<endl;
        cout<<"th_rad= "<<th<<endl;
        
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "robot_footprint";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.1;
        odom_trans.transform.rotation = odom_quat;
        
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
        
        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "robot_footprint";
        odom.twist.twist.linear.x = vk*cos(th);
        odom.twist.twist.linear.y = vk*sin(th);
        odom.twist.twist.angular.z = wk;
        
        //publish the message
        odom_pub.publish(odom);
        //gazebo_pub.publish(odom1);
        r.sleep();
        ros::spinOnce();  
    }
}
