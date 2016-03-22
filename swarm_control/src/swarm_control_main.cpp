#include "swarm_control.h"
#include <iostream>
#include <swarm_control/obstacles.h>
using namespace std;

bool *alarm_p;

void alarmCB1(const std_msgs::Bool g_alarm1) {

  if (g_alarm1.data == true){
    *alarm_p1 = true;
  }
  else{*alarm_p1 = false;}
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  SwarmControl SwarmControl(nh);
  
  alarm_p1 = &(SwarmControl.alarm1);
  //maybe try give position vectors back directly
  ros::ServiceServer service1 = n.advertiseService("obsts1", callback1);
  ros::ServiceServer service2 = n.advertiseService("obsts2", callback2);
  ros::ServiceServer service3 = n.advertiseService("obsts3", callback3);
  ros::ServiceServer service4 = n.advertiseService("obsts4", callback4);
  ros::ServiceServer service5 = n.advertiseService("obsts5", callback5);
  ros::ServiceServer service6 = n.advertiseService("obsts6", callback6);
 
  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  //put some points in the path queue--hard coded here

  // main loop; publish a desired state every iteration
  while (ros::ok()) {
	  
    //SwarmControl.pub_next_state();

    ros::spinOnce();
    looprate.sleep(); //sleep for defined sample period, then do loop again
  }
  return 0;
}

