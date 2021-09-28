// ar_markers.cpp (STEFANO MARIA GIORDANO 25/07/2021)

#include "header.h"

int markerID=0;

// Callback Aruco -----------------------------------------------------------------------------------------------------------------------------------------

void aruco_callback(const std_msgs::UInt32MultiArray &msg) { 

    cout<<"Aruco Callback"<<endl ;

    if (msg.data.empty()==true) { markerID=0 ; } 

    else markerID = msg.data[0] ; 
    
    cout<<"----------------------------------"<<endl;
    cout<<"---------->  markerID [ "<<markerID<<" ]"<<endl;
    cout<<"----------------------------------"<<endl<<endl;
    
  }
// ---------------------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv){

  ros::init(argc, argv, "Ar_Markers");

  ros::NodeHandle n;
  ros::Publisher  pub  = n.advertise<std_msgs::Int32>("/marker", 1)  ;
  ros::Subscriber sub  = n.subscribe("/aruco_marker_publisher/markers_list", 1, aruco_callback);  
      
  ros::Rate r(100);
    while(ros::ok()){

    ros::spinOnce();                                                                                      // check for incoming messages

        
        std_msgs::Int32 msg_flag  ;
        msg_flag.data  = markerID       ; 
        
        //publish the messages
        pub.publish(msg_flag)       ;             

        r.sleep();

    }

    return 0;
}