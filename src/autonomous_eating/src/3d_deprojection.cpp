#include "ros/ros.h"
#include "librealsense/rs.h"
#include "librealsense/rsutil.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include <std_msgs/Float64.h>

class MyDeprojector {
    rs_intrinsics intrinsicMatrix;
    ros::Publisher * returnPub;
    ros::Subscriber sub1;
    ros::Subscriber sub2;

    public:
    MyDeprojector(ros::NodeHandle * n) {
        sub1 = n->subscribe("/camera/depth/camera_info", 1000, &MyDeprojector::cordinatecallback, this);
        sub2 = n->subscribe("/camera/depth/camera_info", 1000, &MyDeprojector::intrinsiccallback, this);

    }
    //message types are wrong
    void cordinatecallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
      //std::cout << "coordinates" << std::endl;
      //todo: get intrinsics from the topic.
      
      //pixel location i.e. x and y
      //float pixels [2] = {msg->data[0], msg->data[1]};
      //todo: depth is the pixel value on the raw depth image
      //float depth = msg->data[2];
      //float cords [3] = {0, 0, 0};

      //rs_deproject_pixel_to_point(cords, intrinsicMatrix, pixels, depth);

      //retun via returnPub
    }
    //message types are wrong
    void intrinsiccallback(const sensor_msgs::CameraInfo::ConstPtr& camInfo_msg)
    {
      //std::cout << camInfo_msg->K[0] << std::endl;
      
      intrinsicMatrix.fx = camInfo_msg->K[0];
      intrinsicMatrix.fy = camInfo_msg->K[4];
      for(int i = 0;i<5;i++){
      intrinsicMatrix.coeffs[i] = camInfo_msg->D[i];
      }
      intrinsicMatrix.height = camInfo_msg->height;
      intrinsicMatrix.width = camInfo_msg->width;
      intrinsicMatrix.model = rs_distortion::RS_DISTORTION_NONE; 
      intrinsicMatrix.ppx = camInfo_msg->K[2];
      intrinsicMatrix.ppy = camInfo_msg->K[5];

      std::cout << "fx:\t" << intrinsicMatrix.fx << "\nfy:\t" << intrinsicMatrix.fy << std::endl;
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