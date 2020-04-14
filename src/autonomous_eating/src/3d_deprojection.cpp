#include "ros/ros.h"
#include "librealsense/rs.h"
#include "librealsense/rsutil.h"
#include "std_msgs/String.h"

class MyDeprojector {
    const rs_intrinsics * intrinsicMatrix;
    ros::Publisher * returnPub;

    public:
    MyDeprojector(ros::NodeHandle * n) {
        //Topic is wrong
        n->subscribe("/realsense_f200/depth/camerainfo", 1000, &MyDeprojector::cordinatecallback, this);
        n->subscribe("/realsense_f200/depth/camerainfo", 1000, &MyDeprojector::intrinsiccallback, this);

    }
    //message types are wrong
    void cordinatecallback(const std_msgs::String::ConstPtr& msg)
    {
      //todo: get intrinsics from the topic.

      //pixel location i.e. x and y
      float pixels [2] = {msg->data[0], msg->data[1]};
      //todo: depth is the pixel value on the raw depth image
      float depth = msg->data[2];
      float cords [3] = {0, 0, 0};

      rs_deproject_pixel_to_point(cords, intrinsicMatrix, pixels, depth);

      //retun via returnPub
    }
    //message types are wrong
    void intrinsiccallback(const std_msgs::String::ConstPtr& msg)
    {
      //todo: construct intrinsic matrix.
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle * n = new ros::NodeHandle();

  MyDeprojector runningNode = MyDeprojector(n);

  ros::spin();

  return 0;
}