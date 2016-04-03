#include "swarm_control.cpp"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  SwarmControl swarm_control(nh);
  
  ros::Publisher geo_twist = nh.advertise <geometry_msgs::Twist> ("robot1/cmd_vel", 1, true);

  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  //put some points in the path queue--hard coded here
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  x_vec.resize(6);
  y_vec.resize(6);

  x_vec[0] = 0;   //1
  y_vec[0] = 0; 

  x_vec[1] = 1;   //2
  y_vec[1] = 0;

  x_vec[2] = 2;   //3
  y_vec[2] = 0;

  x_vec[3] = 0;   //4
  y_vec[3] = 5;

  x_vec[4] = 2;   //5
  y_vec[4] = 5;

  x_vec[5] = 0;   //6
  y_vec[5] = 4;

  swarm_control.set_initial_position(x_vec, y_vec);  ///psi defined to be 0 

  swarm_control.set_des_pose(10.0, 10.0, 0.0);
/*
need to know obst_positions
*/
geometry_msgs::PoseStamped temp;
temp.pose.position.x = 5;
temp.pose.position.y = 3;

  std::vector<geometry_msgs::PoseStamped> obst_posi;
  obst_posi.push_back(temp);

  std::vector<geometry_msgs::PoseStamped> swarm2_obst;
  std::vector<geometry_msgs::PoseStamped> swarm3_obst;
  std::vector<geometry_msgs::PoseStamped> swarm4_obst;
  std::vector<geometry_msgs::PoseStamped> swarm5_obst;
  std::vector<geometry_msgs::PoseStamped> swarm6_obst;

  swarm2_obst.push_back(temp);
  swarm3_obst.push_back(temp);
  swarm4_obst.push_back(temp);
  swarm5_obst.push_back(temp);
  swarm6_obst.push_back(temp);

  swarm_control.ComputeConsumption(
    swarm2_obst,
    swarm3_obst,
    swarm4_obst,
    swarm5_obst,
    swarm6_obst);    // give consumptiion to decisionmaker 

  swarm_control.DecisionMaker();   //give corresponding targets

  ////swarm_1
  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm1_pose, swarm_control.des_pose_1,   ////PSO compute pose vector
    swarm_control.des_state_vec_1); //des_state_vec_1 

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm2_pose, swarm_control.des_pose_2,
    swarm_control.des_state_vec_2); //des_state_vec_2 

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm3_pose, swarm_control.des_pose_3,
    swarm_control.des_state_vec_3); //des_state_vec_3   
  
  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm4_pose, swarm_control.des_pose_4,
    swarm_control.des_state_vec_4); //des_state_vec_4   

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm5_pose, swarm_control.des_pose_5,
    swarm_control.des_state_vec_5); //des_state_vec_5

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm6_pose, swarm_control.des_pose_6,
    swarm_control.des_state_vec_6); //des_state_vec_6  
  // main loop; publish a desired state every iteration
    
  std::vector<nav_msgs::Odometry> vec_of_tran;
  geometry_msgs::PoseStamped new_temp;
<<<<<<< HEAD
  new_temp.pose.position = swarm_control.des_state_vec_1[0].pose.pose.position;
  swarm_control.build_point_and_go(swarm_control.current_swarm1_pose,new_temp,vec_of_tran);
  int num = vec_of_tran.size();
=======
  int des_num =  swarm_control.des_state_vec_1.size();
  for(int i = 0; i < des_num; i++){
  	  new_temp.pose.position = swarm_control.des_state_vec_1[i].pose.pose.position;
      double x = new_temp.pose.position.x;
      ROS_INFO("des x %d is: %f", i, x);

      double y = new_temp.pose.position.y;
      ROS_INFO("des y %d is: %f", i, y);

  }

  swarm_control.build_point_and_go(swarm_control.current_swarm1_pose,new_temp,vec_of_tran);
  double position_x = vec_of_tran[10].pose.pose.position.x;
  double position_y = vec_of_tran[10].pose.pose.position.y;

  ROS_INFO("position x is %f",position_x);
  ROS_INFO("position y is %f", position_y);

  int num = vec_of_tran.size();

>>>>>>> 9af954596a3ba5f1ee7cf94f8527e2938059bccf
  for(int i = 0; i < num; i++){
  	  vec_of_tran[i].header.stamp = ros::Time::now();
	  geometry_msgs::Twist g_twist = vec_of_tran[i].twist.twist;
	  double x_speed = g_twist.linear.x;
	  double z_speed = g_twist.angular.z;
	//  ROS_INFO("x speed is %f",x_speed);
	//  ROS_INFO("z speed is %f",z_speed);
	  geo_twist.publish(g_twist);
	  looprate.sleep(); 
  }
<<<<<<< HEAD
  
=======
 
>>>>>>> 9af954596a3ba5f1ee7cf94f8527e2938059bccf
    ros::spinOnce();
    //sleep for defined sample period, then do loop again

}

