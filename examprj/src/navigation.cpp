#include "navigation.h"
using namespace std;

#define ETAO 2.0  //activation threshold for repulsive potential



//CONSTRUCTOR
NAVIGATION::NAVIGATION(){
   
   //initializing odom lidar and control topics
   _odom_sub = _nh.subscribe("/odom", 1, &NAVIGATION::odometry_cb, this);
   _lidar_sub = _nh.subscribe("/scan", 1, &NAVIGATION::laser_cb, this);
   _marker_reader=_nh.subscribe("/aruco_marker_publisher/markers_list",1, &NAVIGATION::marker_cb, this);
   _wl_vel_pub = _nh.advertise<std_msgs::Float64>("/left_wheel_velocity_controller/command", 1);
   _wr_vel_pub = _nh.advertise<std_msgs::Float64>("/right_wheel_velocity_controller/command",1);
   //_e_pub= _nh.advertise<std_msgs::Float64>("/err",1); //just for plots
   
    
   //initializing all the other variables
   _first_odom=false;
   _goal_index=0;
   _sp_xy[0]=0.0;
   _sp_xy[1]=0.0;
   _sp_velxy[0]=0.0;
   _sp_velxy[1]=0.0;
   _first_artif=false;
   _norm_e=1.0;
   //_pos_error[0]=0.0;
   //_pos_error[1]=0.0;
   _curr_q_dot[0]=0.0;
   _curr_q_dot[1]=0.0;
   _local_minima=false;
   _des_p1.x=7.80;
   _des_p1.y=-2.5;
   _des_p3.x=7.50;
   _des_p3.y=8.00;
   _des_p2.x=-7.50;
   _des_p2.y=-1.50;
   _base.x=0.0;
   _base.y=0.0;
   _des_q={_des_p1, _des_p2, _des_p3, _base};
   _local_obstacle=false;
   _counter=0;
   _read_marker=0;
   _already_aumented=false;
   _arrived_in=false;
   _rotate=false;
   _id_to_search=100;
   _previous_index=0;
}



//taking odometry data
void NAVIGATION::odometry_cb(nav_msgs::Odometry odom){

   //position  
   _curr_p.x = odom.pose.pose.position.x;
   _curr_p.y = odom.pose.pose.position.y;

   //orientation
   tf::Quaternion q (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
   double roll, pitch;
   tf::Matrix3x3(q).getRPY(roll, pitch, _curr_yaw);
   
   //limiting yaw angle in [0,2pi]
   if (_curr_yaw>2*M_PI)
      _curr_yaw=_curr_yaw-2.0*M_PI;
   else if(_curr_yaw<0.0) 
      _curr_yaw=_curr_yaw+2.0*M_PI;

   //velocities
   _curr_q_dot[0] = odom.twist.twist.linear.x;
   _curr_q_dot[1]= odom.twist.twist.linear.y;

   //as first odom comes true control and planning can start
   _first_odom=true;    
}




//reading ar markers
void NAVIGATION::marker_cb(std_msgs::UInt32MultiArray marker){
   if (marker.data.empty()==true){ 
      _read_marker=0 ; 
   } 
   else _read_marker = marker.data[0] ; 
}




//taking lidar data
void NAVIGATION::laser_cb(sensor_msgs::LaserScan laser){
   //wait for odometry
   if(!_first_odom)return;  

   //this will count how many "obstacles" we have
   _counter=0; 

   //we consider a 90° angular range to check for obstacles
   int start_ind = int( ((90-45) / 180.0*M_PI) / laser.angle_increment);
   int end_ind =   int( ((90+45) / 180.0*M_PI) / laser.angle_increment);

   //to check, during local minima managing, if there is any obstacle in front of the robot
   _local_obstacle=false; 

   //being online we dont'know how many obstacles are there in the environment, 
   //so we can just check if any of the lidar's ranges is beyond our repulsive potential activation threshold
   for(int i=start_ind; i<end_ind; i++){
      if(laser.ranges[i]<ETAO){   
         _min_range[_counter]=laser.ranges[i];
         _range_angle[_counter]=(i*laser.angle_increment)-(M_PI/2)+_curr_yaw;
         _counter++;
      }
   }    

   //now, among all the ranges beyond the threshold we search for the minimum one
   int min=100;
   for(int i=0; i<_counter;i++){
      if(_min_range[i]<min){
         min=_min_range[i];
      }
   }

   //if the min range is beyond 0.7 I have an obstacle close to the robot, 
   //and we will consider it during local minima managing 
   if(min<0.3) _local_obstacle=true;
}



//to set rotations of a user defined angle
void NAVIGATION::rotate(float angle){
   ros::Rate wait_r(10);
   float yaw=_curr_yaw;
   float goal_yaw= yaw+angle;

   if(goal_yaw>2*M_PI) goal_yaw=goal_yaw-2*M_PI;
   else if(goal_yaw<=0.0) goal_yaw=goal_yaw+2*M_PI;
   _rotate=true;

   while(fabs(_curr_yaw - goal_yaw)>=0.05 && _read_marker==0){
      wait_r.sleep();
   }
   _rotate=false;
}



//to set motion of a user defined x and y, with user defined vx and vy
void NAVIGATION::move(float x, float y, float vx, float vy){
   ros::Rate wait_m(10);
   geometry_msgs::Point p;
   p.x=_curr_p.x;
   p.y=_curr_p.y;

   geometry_msgs::Point goal_p;
   goal_p.x=p.x+x;
   goal_p.y=p.y+y;
   _sp_xy[0]=goal_p.x;
   _sp_xy[1]=goal_p.y;

   if(fabs(x)>0.0)
      _sp_velxy[0]=vx;
   else
      _sp_velxy[0]=0.0;

   if(fabs(y)>0.0)
      _sp_velxy[1]=vy;
   else
      _sp_velxy[1]=0.0;

   while(fabs(_curr_p.x-goal_p.x)>0.1 | fabs(_curr_p.y-goal_p.y)>0.1){
      wait_m.sleep();
      if(_local_obstacle){
         cout<<"cannot continue move because there is a local obstacle"<<endl;
         break;
      }
   }

   //reset APF setpoints
   _sp_velxy[0]=0.0;
   _sp_velxy[1]=0.0;
}



//to manage local minima through a "[" or "]" trajectory
void NAVIGATION::local_minima_manager(){
   rotate(M_PI);
   if(_goal_index==2 | (_previous_index==1 && _goal_index>3))
      move (-1.0, 0.0, -0.2, 0.0);
   else
      move (1.0,0.0,0.2,0.0);
   move (0.0, 3.0, 0.0, 0.2);
   if(_goal_index==2 | (_previous_index==1 && _goal_index>3))
      move (1.0, 0.0, 0.2, 0.0);
   else
      move (-1.0,0.0,-0.2,0.0);
   _local_minima=false;
   cout<<"Exiting Local Minima Manager"<<endl;   
}




//planning through artificial potential fields
void NAVIGATION::art_pot(){
   
   //wait for odometry
   if(!_first_odom){ /*cout<<"aspetto odom"<<endl;*/return; }

   //here we define the goal for the artificial potentials planner
   geometry_msgs::Point des_p;
   des_p.x=_des_q[_goal_index].x;
   des_p.y=_des_q[_goal_index].y;

   //to go back to the base
//   if(_goal_index>_des_q.size())
//      des_p=_base;

   //to move forward in the room in search of the ar marker
   if(_norm_e<0.07 &&_read_marker==0 && des_p!=_base ){
      _already_aumented=false;
      _arrived_in=true;
      _des_q[_goal_index].y=des_p.y-1.0;
      des_p.y=_des_q[_goal_index].y;
   }

   //to pass to the other rooms just at the first sight of the same AR marker the goal index is augmented
   if(_read_marker!=_id_to_search && _read_marker!=0 && des_p!=_base){
      if(!_already_aumented){
         cout<<"I found this ID, but it wasn't the one: "<<_read_marker<<endl;
         _goal_index++;
         _already_aumented=true;
      }
   }

   //to move to the base if the user defined marker id is found
   if(_read_marker==_id_to_search && !_already_aumented){
      cout<<"I found the desired id: "<<_id_to_search<<endl;
      _previous_index=_goal_index;
      _goal_index=3;
      _already_aumented=true;
   }

   //to set a rotation to find for the ar marker
   if(_arrived_in){
      rotate(M_PI);
      rotate(M_PI);
      _arrived_in=false;
   }

   //defining errors
   float pos_error[2];
   pos_error[0]=des_p.x-_curr_p.x;
   pos_error[1]=des_p.y-_curr_p.y;
   _norm_e=sqrt(pow(pos_error[0],2)+pow(pos_error[1],2));

   //defining gains for the artificial potentials 
   float ka;
   ka=1.6;
   float kr;
   kr=0.08;
 
 //attractive potential
   float f[2];
   float Ua;
   
   //defining conic or parabolid attrattive potential field  
   if(_norm_e<1.00){
      f[0]=ka*pos_error[0];
      f[1]=ka*pos_error[1];
      Ua=1/2*ka*pow(_norm_e,2);
   }
   else{
      f[0]=ka*pos_error[0]/_norm_e; 
      f[1]=ka*pos_error[1]/_norm_e;
      Ua=ka*_norm_e;
   }

 //repulsive potential
   float fr[2]={0.0, 0.0};
   float Ur=0.0;

   //computing cumulative repulsive potential field for each range under ETAO
   for(int i=0;i<_counter;i++){
    
      //defining obstacle coordinates
      float scostam[2];
      scostam[0]=_min_range[i]*cos(_range_angle[i]);
      scostam[1]= _min_range[i]*sin(_range_angle[i]);
      geometry_msgs::Point q_obst;
      q_obst.x=_curr_p.x+scostam[0];
      q_obst.y=_curr_p.y+scostam[1];
    
     //computing single repulsive potential
      fr[0]+=kr/(pow(_min_range[i],2))*pow((1/_min_range[i])-(1/ETAO),1)*((_curr_p.x-q_obst.x)/(_min_range[i]));
      fr[1]+=kr/(pow(_min_range[i],2))*pow((1/_min_range[i])-(1/ETAO),1)*((_curr_p.y-q_obst.y)/(_min_range[i]));
      Ur+=kr/1*pow(((1/_min_range[i])-1/ETAO),1);
   }

   //total potential
   float U;
   U=Ua+Ur;
   double artif_accxy[2];

   //here we consider also a damping factor to stop the robot in the desired position, as the force field has to be considered as an accelleration field 
   artif_accxy[0]=f[0]+fr[0]-(2.5*_curr_q_dot[0]); 
   artif_accxy[1]=f[1]+fr[1]-(2.5*_curr_q_dot[1]);

   //local minima check
   if(fabs(f[0]+fr[0])<0.05 && fabs(f[1]+fr[1])<0.05 && U>0.5 && _first_artif){
      _local_minima=true;
      cout<<"I'm in a local minima"<<endl; 
   }   
   
   //integration
   float dt;
   dt=1.0/100.0;

   //velocities
   _sp_velxy[0]+=artif_accxy[0]*dt;
   if(isnan(_sp_velxy[0])) _sp_velxy[0]=0.0;
   _sp_velxy[1]+=artif_accxy[1]*dt;
   if(isnan(_sp_velxy[1])) _sp_velxy[1]=0.0;

   //positions
   _sp_xy[0]+=_sp_velxy[0]*dt;
   if(isnan(_sp_xy[0])) _sp_xy[0]=0.0;
   _sp_xy[1]+=_sp_velxy[1]*dt;
   if(isnan(_sp_xy[1])) _sp_xy[1]=0.0;

   _first_artif=true;
}



//planner thread calling art_pot() and local_minima_manger() functions
void NAVIGATION::planner(){
   bool moved=false;
   bool rotated=false;
   ros::Rate plan_r(100);
   ros::Rate r_wait(10);
   while(!_first_odom)
      r_wait.sleep();

   while(ros::ok()){
      if(_local_minima){
         local_minima_manager();
      }
      else{
         art_pot();
      } 
      plan_r.sleep();
   }
}



//low level controller
void NAVIGATION::send_art_to_control(){
   
   //sleep until there is not the first odometry
   ros::Rate r_wait(10);
   while(!_first_odom)
      r_wait.sleep();

   //defining time variables for integration and the controller's variables         
   float v,w;
   float b=0.1;
   float k1=0.1;
   float k2=0.1;
   float u1, u2;
   std_msgs::Float64 wleft,wright;
   geometry_msgs::Point Y;

   //starting the control loop
   ros::Rate r(100);
   while(ros::ok()){
      
      //to numerically manage rotations 
      if(_rotate){
         v=0.0;
         w=0.5;
      }
      else{
         
         //input-output linearization
         //Y point definition
         Y.x=_curr_p.x+b*cos(_curr_yaw);
         Y.y=_curr_p.y+b*sin(_curr_yaw);
      
         //mapping the positions for the y point
         float q_des[2];
         q_des[0]=_sp_xy[0]+b*cos(_curr_yaw);
         q_des[1]=_sp_xy[1]+b*sin(_curr_yaw);
         
         /*just for plots
         std_msgs::Float64 norme;
         norme.data=sqrt(pow((q_des[0]-Y.x),2)+pow((q_des[1]-Y.y),2));
         _e_pub.publish(norme);
         */

         //defining the virtual inputs
         u1=_sp_velxy[0]+(k1*(q_des[0]-Y.x));
         u2=_sp_velxy[1]+(k2*(q_des[1]-Y.y));

         //defining heading and steering velocities
         v= cos(_curr_yaw)*u1+sin(_curr_yaw)*u2;
         w=-sin(_curr_yaw)/b*u1+cos(_curr_yaw)/b*u2; 
      }
   
      //regular mapping heading and steering velocities to the wheels if we are not in a local minima
      wleft.data= (v/0.1)-(0.4*w/(2*0.1));
      wright.data= (v/0.1)+(0.4*w/(2*0.1));
      if(isnan(wleft.data)) wleft.data=0.0;
      if(isnan(wright.data)) wright.data=0.0;

      //publishing wheels velocities to the high level controller
      _wr_vel_pub.publish(wright);
      _wl_vel_pub.publish(wleft);
      
      r.sleep();
   }
}



//start all parallel functions
void NAVIGATION::run(){
      boost::thread send_art_to_control_t( &NAVIGATION::send_art_to_control, this);
      boost::thread planning_t(&NAVIGATION::planner,this);
      ros::spin();      
}





//main
int main(int argc, char** argv){   
   ros::init(argc,argv, "mobile_navigation_node");
   NAVIGATION nav;
      cout<<"Set the Id to search among 8, 26 and 582: ";
   cin>>nav._id_to_search;
   cout<<endl<<"Ok: Let's start. Remind to start odometry"<<endl;
   nav.run();
   return 0;
}
