#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include "boost/thread.hpp"
#include <sensor_msgs/Imu.h>
#include "tf/tf.h"
using namespace std;

class ODOM{
    public:
        ODOM();
        void run();
        void encoders_sub_callback (std_msgs::Float32MultiArray  msg);
        void compute_odom ();
        void imu_sub_callback(sensor_msgs::Imu msg);

    private:
    ros::NodeHandle _nh;
         float _wl;
         float _wr;
        long double _x;
        long double _y;
        double _th;
        bool _first_odom;

        ros::Subscriber _encoders_sub;
        ros::Subscriber _imu_sub;
        ros::Publisher _odom_pub;

};
