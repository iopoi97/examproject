#include "ros/ros.h"

//-Input
#include "geometry_msgs/Twist.h" //to control robot in velocity
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h" //to read odometry
#include "sensor_msgs/LaserScan.h" //to read lidar
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "tf/tf.h"
#include "gazebo_msgs/ModelStates.h"
#include "boost/thread.hpp"

using namespace std;

class NAVIGATION{

    public:
        NAVIGATION();
        void run(); //used to start all the parallel functions
        void odometry_cb(nav_msgs::Odometry odom);
        void laser_cb(sensor_msgs::LaserScan laser);
        void art_pot ();
        void send_art_to_control ();
        void planner();
        void rotate(float angle);
        void move(float x, float y, float vx, float vy);
        void marker_cb(std_msgs::UInt32MultiArray marker);
        void local_minima_manager();
        int _id_to_search;

    private:
        ros::NodeHandle _nh;
        geometry_msgs::Point    _curr_p;     //current position retrieved from odometry
        double _curr_q_dot[2];
        //double   _artif_accxy[2];  //accelerazioni generate dai potenziali artificiali 
        double   _sp_velxy[2];
        double   _sp_xy[2];
        double   _curr_yaw;
        ros::Subscriber _odom_sub;
        ros::Publisher _wl_vel_pub;
        ros::Publisher _wr_vel_pub;
        ros::Publisher _e_pub;
        ros:: Subscriber _lidar_sub;
        ros:: Subscriber _marker_reader;
        bool _local_minima;
        float _norm_e;
        //float _pos_error[2];
        bool _first_odom;
        double _min_range[800];
        double _range_angle[800];
        vector<geometry_msgs::Point> _des_q;
        geometry_msgs::Point    _des_p1;
        geometry_msgs::Point    _des_p2;
        geometry_msgs::Point    _des_p3;
        geometry_msgs::Point    _base;
        int _goal_index;
        bool _first_artif;
        bool _local_obstacle;
        int _counter;
        int _read_marker;
        bool _already_aumented;
        bool _arrived_in;
        bool _rotate;
        int _previous_index;
        
};
