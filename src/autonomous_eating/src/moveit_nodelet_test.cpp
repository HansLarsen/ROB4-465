#include "ros/ros.h"
#include "ros/time.h"

// main where it all begins
int main(int argc, char **argv){
  ros::init(argc, argv, "master_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);


  while (ros::ok()){
    //master_node->handleGesture();

    ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}