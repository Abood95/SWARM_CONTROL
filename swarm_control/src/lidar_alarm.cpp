#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <swarm_control/obstacles.h>
#include "swarm_control.h"

//#include <vector>
using namespace std;
// these values to be set within the laser callback
// NOT real; callback will have to find this

std::vector<double> rela_dist1;
std::vector<double> rela_angle1;
void laserCallback1(const sensor_msgs::LaserScan& laser_scan1) {
	    float ping_dist1=5.0; 
        int ping_index_1= -1; 

        ROS_INFO("robot_1 alarm");
        double angle_min_1 = laser_scan1.angle_min;
        double angle_max_1 = laser_scan1.angle_max;
        double angle_increment_1 = laser_scan1.angle_increment;
        double range_min_1 = laser_scan1.range_min;
        double range_max_1 = laser_scan1.range_max;

        double ping_front_max1 = (int) (0.6-angle_min_1)/angle_increment_1;    //-90 degree ping
        double ping_front_min1 = (int) (-0.6 - angle_min_1)/angle_increment_1;   

	   double angle1;
	   for(ping_index_1=ping_front_min1;ping_index_1<ping_front_max1;ping_index_1++){
		   ping_dist1 = laser_scan1.ranges[ping_index_1];
		   if(ping_dist1 < range_max_1){
               rela_dist1.push_back(ping_dist1);
			   angle1 = ping_index_1 * angle_increment_1 + angle_min_1;
			   rela_angle1.push_back(angle1);			   
		   }
	   }
}

std::vector<double> rela_dist2;
std::vector<double> rela_angle2;
void laserCallback2(const sensor_msgs::LaserScan& laser_scan2) {
	    float ping_dist2=5.0; 
        int ping_index_2= -1; 

        ROS_INFO("robot_2 alarm");
        double angle_min_2 = laser_scan2.angle_min;
        double angle_max_2 = laser_scan2.angle_max;
        double angle_increment_2 = laser_scan2.angle_increment;
        double range_min_2 = laser_scan2.range_min;
        double range_max_2 = laser_scan2.range_max;

        double ping_front_max2 = (int) (0.6-angle_min_2)/angle_increment_2;    //-90 degree ping
        double ping_front_min2 = (int) (-0.6 - angle_min_2)/angle_increment_2;   

	   double angle2;
	   for(ping_index_2=ping_front_min2;ping_index_2<ping_front_max2;ping_index_2++){
		   ping_dist2 = laser_scan2.ranges[ping_index_2];
		   if(ping_dist2 < range_max_2){
               rela_dist2.push_back(ping_dist2);
			   angle2 = ping_index_2 * angle_increment_2 + angle_min_2;
			   rela_angle2.push_back(angle2);			   
		   }
	   }
}


std::vector<double> rela_dist3;
std::vector<double> rela_angle3;
void laserCallback3(const sensor_msgs::LaserScan& laser_scan3) {
	    float ping_dist3=5.0; 
        int ping_index_3= -1; 

        ROS_INFO("robot_3 alarm");
        double angle_min_3 = laser_scan3.angle_min;
        double angle_max_3 = laser_scan3.angle_max;
        double angle_increment_3 = laser_scan3.angle_increment;
        double range_min_3 = laser_scan3.range_min;
        double range_max_3 = laser_scan3.range_max;

        double ping_front_max3 = (int) (0.6-angle_min_3)/angle_increment_3;    //-90 degree ping
        double ping_front_min3 = (int) (-0.6 - angle_min_3)/angle_increment_3;   

	   double angle3;
	   for(ping_index_3=ping_front_min3;ping_index_3<ping_front_max3;ping_index_3++){
		   ping_dist3 = laser_scan3.ranges[ping_index_3];
		   if(ping_dist3 < range_max_3){
               rela_dist3.push_back(ping_dist3);
			   angle3 = ping_index_3 * angle_increment_3 + angle_min_3;
			   rela_angle3.push_back(angle3);			   
		   }
	   }
}

std::vector<double> rela_dist4;
std::vector<double> rela_angle4;
void laserCallback4(const sensor_msgs::LaserScan& laser_scan4) {
	    float ping_dist4=5.0; 
        int ping_index_4= -1; 

        ROS_INFO("robot_4 alarm");
        double angle_min_4 = laser_scan4.angle_min;
        double angle_max_4 = laser_scan4.angle_max;
        double angle_increment_4 = laser_scan4.angle_increment;
        double range_min_4 = laser_scan4.range_min;
        double range_max_4 = laser_scan4.range_max;

        double ping_front_max4 = (int) (0.6-angle_min_4)/angle_increment_4;    //-90 degree ping
        double ping_front_min4 = (int) (-0.6 - angle_min_4)/angle_increment_4;   

	   double angle4;
	   for(ping_index_4=ping_front_min4;ping_index_4<ping_front_max4;ping_index_4++){
		   ping_dist4 = laser_scan4.ranges[ping_index_4];
		   if(ping_dist4 < range_max_4){
               rela_dist4.push_back(ping_dist4);
			   angle4 = ping_index_4 * angle_increment_4 + angle_min_4;
			   rela_angle4.push_back(angle4);			   
		   }
	   }
}

std::vector<double> rela_dist5;
std::vector<double> rela_angle5;
void laserCallback5(const sensor_msgs::LaserScan& laser_scan5) {
	    float ping_dist5=5.0; 
        int ping_index_5= -1; 

        ROS_INFO("robot_5 alarm");
        double angle_min_5 = laser_scan5.angle_min;
        double angle_max_5 = laser_scan5.angle_max;
        double angle_increment_5 = laser_scan5.angle_increment;
        double range_min_5 = laser_scan5.range_min;
        double range_max_5 = laser_scan5.range_max;

        double ping_front_max5 = (int) (0.6-angle_min_5)/angle_increment_5;    //-90 degree ping
        double ping_front_min5 = (int) (-0.6 - angle_min_5)/angle_increment_5;   

	   double angle5;
	   for(ping_index_5=ping_front_min5;ping_index_5<ping_front_max5;ping_index_5++){
		   ping_dist5 = laser_scan5.ranges[ping_index_5];
		   if(ping_dist5 < range_max_5){
               rela_dist5.push_back(ping_dist5);
			   angle5 = ping_index_5 * angle_increment_5 + angle_min_5;
			   rela_angle5.push_back(angle5);			   
		   }
	   }
}

std::vector<double> rela_dist6;
std::vector<double> rela_angle6;
void laserCallback6(const sensor_msgs::LaserScan& laser_scan6) {
	    float ping_dist6=5.0; 
        int ping_index_6= -1; 

        ROS_INFO("robot_6 alarm");
        double angle_min_6 = laser_scan6.angle_min;
        double angle_max_6 = laser_scan6.angle_max;
        double angle_increment_6 = laser_scan6.angle_increment;
        double range_min_6 = laser_scan6.range_min;
        double range_max_6 = laser_scan6.range_max;

        double ping_front_max6 = (int) (0.6-angle_min_6)/angle_increment_6;    //-90 degree ping
        double ping_front_min6 = (int) (-0.6 - angle_min_6)/angle_increment_6;   

	   double angle6;
	   for(ping_index_6=ping_front_min6;ping_index_6<ping_front_max6;ping_index_6++){
		   ping_dist6 = laser_scan6.ranges[ping_index_6];
		   if(ping_dist6 < range_max_6){
               rela_dist6.push_back(ping_dist6);
			   angle6 = ping_index_6 * angle_increment_6 + angle_min_6;
			   rela_angle6.push_back(angle6);			   
		   }
	   }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_lidar_alarm"); //name this node
    ros::NodeHandle n; 
	
	SwarmControl SwarmControl(n);
	SwarmControl.set_init_pose(0,0,0); //x=0, y=0, psi=0
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::ServiceClient client_1 = nh.serviceClient<swarm_control::obstacles>("obsts1");  
	ros::ServiceClient client_2 = nh.serviceClient<swarm_control::obstacles>("obsts2"); 
	ros::ServiceClient client_3 = nh.serviceClient<swarm_control::obstacles>("obsts3"); 
	ros::ServiceClient client_4 = nh.serviceClient<swarm_control::obstacles>("obsts4"); 
	ros::ServiceClient client_5 = nh.serviceClient<swarm_control::obstacles>("obsts5"); 
	ros::ServiceClient client_6 = nh.serviceClient<swarm_control::obstacles>("obsts6"); 
	
////////////////////////////////////////coordinates   
	while (!client_1.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service1");
	
	geometry_msgs::PoseStamped temp_pose = SwarmControl.current_pose_1;
	double psi = trajBuilder_.convertPlanarQuat2Psi(temp_pose.pose.orientation);
	int obst_num = rela_dist1.size();
	double dx;
	double dy;
	geometry_msgs::PoseStamped obst_pose1;
	swarm_control::obstacles swarm_1;
	for (int i = 0; i < obst_num;i++){
	    dx = rela_dist1[i] * cos(fabs(psi - rela_angle1[i]));
	    dy= rela_dist1[i] * sin(psi - rela_angle1[i]);
	    obst_pose1.pose.position.x = temp_pose.pose.position.x + dx;
	    obst_pose1.pose.position.y = temp_pose.pose.position.y + dy;
		swarm_1.request.obst_posi.push_back(obst_pose1);
	}
	client_1.call(swarm_1);	
//////////////	
	while (!client_2.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service2");
	
	temp_pose = SwarmControl.current_pose_2;
	psi = trajBuilder_.convertPlanarQuat2Psi(temp_pose.pose.orientation);
	obst_num = rela_dist2.size();
	geometry_msgs::PoseStamped obst_pose2;
	swarm_control::obstacles swarm_2;
	for (int i = 0; i < obst_num;i++){
	    dx = rela_dist2[i] * cos(fabs(psi - rela_angle2[i]));
	    dy = rela_dist2[i] * sin(psi - rela_angle2[i]);
	    obst_pose2.pose.position.x = temp_pose.pose.position.x + dx;
	    obst_pose2.pose.position.y = temp_pose.pose.position.y + dy;
		swarm_2.request.obst_posi.push_back(obst_pose2);
	}
	client_2.call(swarm_2);	
////////////////////////////	
 	while (!client_3.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service3");
	
    temp_pose = SwarmControl.current_pose_3;
	psi = trajBuilder_.convertPlanarQuat2Psi(temp_pose.pose.orientation);
	obst_num = rela_dist3.size();
	geometry_msgs::PoseStamped obst_pose3;
	swarm_control::obstacles swarm_3;
	for (int i = 0; i < obst_num;i++){
	    dx = rela_dist3[i] * cos(fabs(psi - rela_angle3[i]));
	    dy = rela_dist3[i] * sin(psi - rela_angle3[i]);
	    obst_pose3.pose.position.x = temp_pose.pose.position.x + dx;
	    obst_pose3.pose.position.y = temp_pose.pose.position.y + dy;
		swarm_3.request.obst_posi.push_back(obst_pose3);
	}
	client_3.call(swarm_3);	
	///////////////////////////////
 	while (!client_4.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service4");
	
    temp_pose = SwarmControl.current_pose_4;
	psi = trajBuilder_.convertPlanarQuat2Psi(temp_pose.pose.orientation);
	obst_num = rela_dist4.size();
	geometry_msgs::PoseStamped obst_pose4;
	swarm_control::obstacles swarm_4;
	for (int i = 0; i < obst_num;i++){
	    dx = rela_dist4[i] * cos(fabs(psi - rela_angle4[i]));
	    dy = rela_dist4[i] * sin(psi - rela_angle4[i]);
	    obst_pose4.pose.position.x = temp_pose.pose.position.x + dx;
	    obst_pose4.pose.position.y = temp_pose.pose.position.y + dy;
		swarm_4.request.obst_posi.push_back(obst_pose4);
	}
	client_4.call(swarm_4);	
//////////////////	
 	while (!client_5.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service5");
	
    temp_pose = SwarmControl.current_pose_5;
	psi = trajBuilder_.convertPlanarQuat2Psi(temp_pose.pose.orientation);
	obst_num = rela_dist5.size();
	geometry_msgs::PoseStamped obst_pose5;
	swarm_control::obstacles swarm_5;
	for (int i = 0; i < obst_num;i++){
	    dx = rela_dist5[i] * cos(fabs(psi - rela_angle5[i]));
	    dy = rela_dist5[i] * sin(psi - rela_angle5[i]);
	    obst_pose5.pose.position.x = temp_pose.pose.position.x + dx;
	    obst_pose5.pose.position.y = temp_pose.pose.position.y + dy;
		swarm_5.request.obst_posi.push_back(obst_pose5);
	}
	client_5.call(swarm_5);	
////////////////////////////	
 	while (!client_6.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service6");
	
    temp_pose = SwarmControl.current_pose_6;
	psi = trajBuilder_.convertPlanarQuat2Psi(temp_pose.pose.orientation);
	obst_num = rela_dist6.size();
	geometry_msgs::PoseStamped obst_pose6;
	swarm_control::obstacles swarm_6;
	for (int i = 0; i < obst_num;i++){
	    dx = rela_dist6[i] * cos(fabs(psi - rela_angle6[i]));
	    dy = rela_dist6[i] * sin(psi - rela_angle6[i]);
	    obst_pose6.pose.position.x = temp_pose.pose.position.x + dx;
	    obst_pose6.pose.position.y = temp_pose.pose.position.y + dy;
		swarm_6.request.obst_posi.push_back(obst_pose6);
	}
	client_6.call(swarm_6);	
	///////////////////////////////////////////////
    ros::Subscriber lidar_subscriber1 = nh.subscribe("/scan1", 1, laserCallback1);
    ros::Subscriber lidar_subscriber2 = nh.subscribe("/scan2", 1, laserCallback2);
    ros::Subscriber lidar_subscriber3 = nh.subscribe("/scan3", 1, laserCallback3);
    ros::Subscriber lidar_subscriber4 = nh.subscribe("/scan4", 1, laserCallback4);
    ros::Subscriber lidar_subscriber5 = nh.subscribe("/scan5", 1, laserCallback5);
    ros::Subscriber lidar_subscriber6 = nh.subscribe("/scan6", 1, laserCallback6);

    ros::spin();
    return 0; // should never get here, unless roscore dies
}

