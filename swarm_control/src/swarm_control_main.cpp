#include "swarm_control.h"
#include <iostream>
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

  ros::Subscriber sub1 = nh.subscribe("/lidar_alarm_1", 1, alarmCB1);


  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  SwarmControl.set_init_pose(0,0,0); //x=0, y=0, psi=0
  //put some points in the path queue--hard coded here
  SwarmControl.append_path_queue(5.0,0.0,0.0);

  // main loop; publish a desired state every iteration
  while (ros::ok()) {
    SwarmControl.pub_next_state();

    ros::spinOnce();
    looprate.sleep(); //sleep for defined sample period, then do loop again
  }
  return 0;
}

