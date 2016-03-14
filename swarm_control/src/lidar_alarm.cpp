#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
//#include <vector>

const double SAFE_DISTANCE = 1.0; //set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist=5.0; // global var to hold length of a SINGLE LIDAR ping--in front

int ping_index_= -1; // NOT real; callback will have to find this

double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;

bool laser_alarm_1=false;   
bool laser_alarm_2=false;
bool laser_alarm_3=false;
bool laser_alarm_4=false;
bool laser_alarm_5=false;
bool laser_alarm_6=false;
 
ros::Publisher lidar_alarm_publisher_1;
ros::Publisher lidar_alarm_publisher_2;
ros::Publisher lidar_alarm_publisher_3;
ros::Publisher lidar_alarm_publisher_4;
ros::Publisher lidar_alarm_publisher_5;
ros::Publisher lidar_alarm_publisher_6;

ros::Publisher lidar_dist_publisher_1;
ros::Publisher lidar_dist_publisher_2;
ros::Publisher lidar_dist_publisher_3;
ros::Publisher lidar_dist_publisher_4;
ros::Publisher lidar_dist_publisher_5;
ros::Publisher lidar_dist_publisher_6;


float obstac_dist1;
float obstac_dist2;
float obstac_dist3;
float obstac_dist4;
float obstac_dist5;
float obstac_dist6;

int ping_front_min = 0;
int ping_front_max = 0;

int counter = 0;

void laserCallback1(const sensor_msgs::LaserScan& laser_scan1) {

        ROS_INFO("robot_1 alarm");
        angle_min_ = laser_scan1.angle_min;
        angle_max_ = laser_scan1.angle_max;
        angle_increment_ = laser_scan1.angle_increment;
        range_min_ = laser_scan1.range_min;
        range_max_ = laser_scan1.range_max;

        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;   

        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        obstac_dist1 = laser_scan1.ranges[ping_index_]; 

bool front1;
//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front1 = true;  
       }
     else{
        front1 = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg1;

    if(front1){
       ROS_WARN("Obstacles in front");
       laser_alarm_1=true;  // notice lidar and laser
       lidar_alarm_msg1.data = laser_alarm_1;
       lidar_alarm_publisher_1.publish(lidar_alarm_msg1);
    }
     else{
     laser_alarm_1 = false;
     lidar_alarm_msg1.data = laser_alarm_1;
     lidar_alarm_publisher_1.publish(lidar_alarm_msg1);
     }

  //distance message
   std_msgs::Float32 lidar_dist_msg1;
   lidar_dist_msg1.data = obstac_dist1;
   lidar_dist_publisher_1.publish(lidar_dist_msg1);      
}


void laserCallback2(const sensor_msgs::LaserScan& laser_scan2) {

        ROS_INFO("robot_2 alarm");
        angle_min_ = laser_scan2.angle_min;
        angle_max_ = laser_scan2.angle_max;
        angle_increment_ = laser_scan2.angle_increment;
        range_min_ = laser_scan2.range_min;
        range_max_ = laser_scan2.range_max;

        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;    

        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        obstac_dist2 = laser_scan2.ranges[ping_index_]; 

        bool front2;
//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan2.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front2 = true;  
       }
     else{
        front2 = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg2;

    if(front2){
       ROS_WARN("Obstacles in front");
       laser_alarm_2=true;  // notice lidar and laser
       lidar_alarm_msg2.data = laser_alarm_2;
       lidar_alarm_publisher_2.publish(lidar_alarm_msg2);
    }
     else{
     laser_alarm_2 = false;
     lidar_alarm_msg2.data = laser_alarm_2;
     lidar_alarm_publisher_2.publish(lidar_alarm_msg2);
     }  

   std_msgs::Float32 lidar_dist_msg2;
   lidar_dist_msg2.data = obstac_dist2;
   lidar_dist_publisher_2.publish(lidar_dist_msg2);   
}

void laserCallback3(const sensor_msgs::LaserScan& laser_scan3) {

        ROS_INFO("robot_3 alarm");
        angle_min_ = laser_scan3.angle_min;
        angle_max_ = laser_scan3.angle_max;
        angle_increment_ = laser_scan3.angle_increment;
        range_min_ = laser_scan3.range_min;
        range_max_ = laser_scan3.range_max;

        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;    

        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        obstac_dist3 = laser_scan3.ranges[ping_index_]; 

        bool front3;
//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan3.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front3 = true;  
       }
     else{
        front3 = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg3;

    if(front3){
       ROS_WARN("Obstacles in front");
       laser_alarm_3=true;  // notice lidar and laser
       lidar_alarm_msg3.data = laser_alarm_3;
       lidar_alarm_publisher_3.publish(lidar_alarm_msg3);
    }
     else{
     laser_alarm_3 = false;
     lidar_alarm_msg3.data = laser_alarm_3;
     lidar_alarm_publisher_3.publish(lidar_alarm_msg3);
     }  

   std_msgs::Float32 lidar_dist_msg3;
   lidar_dist_msg3.data = obstac_dist3;
   lidar_dist_publisher_3.publish(lidar_dist_msg3);   
}

void laserCallback4(const sensor_msgs::LaserScan& laser_scan4) {

        ROS_INFO("robot_4 alarm");
        angle_min_ = laser_scan4.angle_min;
        angle_max_ = laser_scan4.angle_max;
        angle_increment_ = laser_scan4.angle_increment;
        range_min_ = laser_scan4.range_min;
        range_max_ = laser_scan4.range_max;

        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;    

        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        obstac_dist4 = laser_scan4.ranges[ping_index_];

        bool front4; 
//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan4.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front4 = true;  
       }
     else{
        front4 = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg4;

    if(front4){
       ROS_WARN("Obstacles in front");
       laser_alarm_4=true;  // notice lidar and laser
       lidar_alarm_msg4.data = laser_alarm_4;
       lidar_alarm_publisher_4.publish(lidar_alarm_msg4);
    }
     else{
     laser_alarm_4 = false;
     lidar_alarm_msg4.data = laser_alarm_4;
     lidar_alarm_publisher_4.publish(lidar_alarm_msg4);
     }  

   std_msgs::Float32 lidar_dist_msg4;
   lidar_dist_msg4.data = obstac_dist4;
   lidar_dist_publisher_4.publish(lidar_dist_msg4);   
}

void laserCallback5(const sensor_msgs::LaserScan& laser_scan5) {

        ROS_INFO("robot_5 alarm");
        angle_min_ = laser_scan5.angle_min;
        angle_max_ = laser_scan5.angle_max;
        angle_increment_ = laser_scan5.angle_increment;
        range_min_ = laser_scan5.range_min;
        range_max_ = laser_scan5.range_max;

        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;    

        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        obstac_dist5 = laser_scan5.ranges[ping_index_]; 

bool front5;
//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan5.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front5 = true;  
       }
     else{
        front5 = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg5;

    if(front5){
       ROS_WARN("Obstacles in front");
       laser_alarm_5=true;  // notice lidar and laser
       lidar_alarm_msg5.data = laser_alarm_5;
       lidar_alarm_publisher_5.publish(lidar_alarm_msg5);
    }
     else{
     laser_alarm_5 = false;
     lidar_alarm_msg5.data = laser_alarm_5;
     lidar_alarm_publisher_5.publish(lidar_alarm_msg5);
     }  

   std_msgs::Float32 lidar_dist_msg5;
   lidar_dist_msg5.data = obstac_dist5;
   lidar_dist_publisher_5.publish(lidar_dist_msg5);   
}

void laserCallback6(const sensor_msgs::LaserScan& laser_scan6) {

        ROS_INFO("robot_6 alarm");
        angle_min_ = laser_scan6.angle_min;
        angle_max_ = laser_scan6.angle_max;
        angle_increment_ = laser_scan6.angle_increment;
        range_min_ = laser_scan6.range_min;
        range_max_ = laser_scan6.range_max;

        ping_front_max = (int) (0.6-angle_min_)/angle_increment_;    //-90 degree ping
        ping_front_min = (int) (-0.6 - angle_min_)/angle_increment_;    

        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        obstac_dist6 = laser_scan6.ranges[ping_index_]; 

bool front6;
//front
     for(ping_index_=ping_front_min;ping_index_<ping_front_max;ping_index_++){
         ping_dist = laser_scan6.ranges[ping_index_];
         if(ping_dist<SAFE_DISTANCE){
             counter+=1;
           }
        } 

     if(counter>1){ 
        front6 = true;  
       }
     else{
        front6 = false;
     }
     counter = 0;  //clear counter
     

std_msgs::Bool lidar_alarm_msg6;

    if(front6){
       ROS_WARN("Obstacles in front");
       laser_alarm_6=true;  // notice lidar and laser
       lidar_alarm_msg6.data = laser_alarm_6;
       lidar_alarm_publisher_6.publish(lidar_alarm_msg6);
    }
     else{
     laser_alarm_6 = false;
     lidar_alarm_msg6.data = laser_alarm_6;
     lidar_alarm_publisher_6.publish(lidar_alarm_msg6);
     }  

   std_msgs::Float32 lidar_dist_msg6;
   lidar_dist_msg6.data = obstac_dist6;
   lidar_dist_publisher_6.publish(lidar_dist_msg6);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "new_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    //1
    ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("lidar_alarm_1", 1);
    lidar_alarm_publisher_1 = pub1;

    ros::Publisher pub_dist1 = nh.advertise<std_msgs::Bool>("lidar_dist_1", 1);
    lidar_dist_publisher_1 = pub_dist1;

    //2    
    ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("lidar_alarm_2", 1);
    lidar_alarm_publisher_2 = pub2;

    ros::Publisher pub_dist2 = nh.advertise<std_msgs::Bool>("lidar_dist_2", 1);
    lidar_dist_publisher_2 = pub_dist2; 

    //3
    ros::Publisher pub3 = nh.advertise<std_msgs::Bool>("lidar_alarm_3", 1);
    lidar_alarm_publisher_3 = pub3;

    ros::Publisher pub_dist3 = nh.advertise<std_msgs::Bool>("lidar_dist_3", 1);
    lidar_dist_publisher_3 = pub_dist3;

    //4 
    ros::Publisher pub4 = nh.advertise<std_msgs::Bool>("lidar_alarm_4", 1);
    lidar_alarm_publisher_4 = pub4;

    ros::Publisher pub_dist4 = nh.advertise<std_msgs::Bool>("lidar_dist_4", 1);
    lidar_dist_publisher_4 = pub_dist4; 

    //5
    ros::Publisher pub5 = nh.advertise<std_msgs::Bool>("lidar_alarm_5", 1);
    lidar_alarm_publisher_5 = pub5;

    ros::Publisher pub_dist5 = nh.advertise<std_msgs::Bool>("lidar_dist_5", 1);
    lidar_dist_publisher_5 = pub_dist5; 

    //6
    ros::Publisher pub6 = nh.advertise<std_msgs::Bool>("lidar_alarm_6", 1);
    lidar_alarm_publisher_6 = pub6; 

    ros::Publisher pub_dist6 = nh.advertise<std_msgs::Bool>("lidar_dist_6", 1);
    lidar_dist_publisher_6 = pub_dist6;


    ros::Subscriber lidar_subscriber1 = nh.subscribe("/scan1", 1, laserCallback1);
    ros::Subscriber lidar_subscriber2 = nh.subscribe("/scan2", 1, laserCallback2);
    ros::Subscriber lidar_subscriber3 = nh.subscribe("/scan3", 1, laserCallback3);
    ros::Subscriber lidar_subscriber4 = nh.subscribe("/scan4", 1, laserCallback4);
    ros::Subscriber lidar_subscriber5 = nh.subscribe("/scan5", 1, laserCallback5);
    ros::Subscriber lidar_subscriber6 = nh.subscribe("/scan6", 1, laserCallback6);

    ros::spin();
    return 0; // should never get here, unless roscore dies
}

