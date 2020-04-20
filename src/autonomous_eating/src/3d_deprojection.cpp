#include "ros/ros.h"
#include "librealsense/rs.h"
#include "librealsense/rsutil.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include "autonomous_eating/deproject.h"

class MyDeprojector {
    ros::NodeHandle* n;
    rs_intrinsics intrinsicMatrix;
    ros::Publisher * returnPub;
    float pixel_array[2];
    ros::Subscriber sub;
    ros::Publisher pub_cords;
    ros::ServiceServer deproject_server;
    bool hasIntrinsics;

    public:
    MyDeprojector(ros::NodeHandle * n) {
        this->n = n;
        sub = n->subscribe("/r200/camera/depth/camera_info", 1000, &MyDeprojector::intrinsiccallback, this);
        pub_cords = n->advertise<std_msgs::Float32MultiArray>("/3D_cordinates",1);
        deproject_server = n->advertiseService("deproject_pixel_to_world", &MyDeprojector::deproject_func, this);
        ROS_INFO_STREAM("deproject-server running!");
        hasIntrinsics = false;
    }

    bool deproject_func(autonomous_eating::deproject::Request& req, autonomous_eating::deproject::Response &res)
    {        
      if(hasIntrinsics)
      {
        //pixel location i.e. x and y
        float pixels [2] = {req.x, req.y};
        //todo: depth is the pixel value on the raw depth image
        float depth = req.z;
        float cords [3] = {0, 0, 0};
        rs_deproject_pixel_to_point(cords, &intrinsicMatrix, pixels, depth);

        res.x = cords[0];
        res.y = cords[1];
        res.z = cords[2];
        return true;
      }
      else
      {
        ROS_INFO_STREAM("Failed to serve a client");
        return false;
      }
    }

    //message types are wrong
    void cordinatecallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
      //pixel location i.e. x and y
      float pixels [2] = {msg->data[0], msg->data[1]};
      //todo: depth is the pixel value on the raw depth image
      float depth = msg->data[2];
      float cords [3] = {0, 0, 0};
      rs_deproject_pixel_to_point(cords, &intrinsicMatrix, pixels, depth);

      std_msgs::Float32MultiArray fma;
      fma.data.push_back(cords[0]);
      fma.data.push_back(cords[1]);
      fma.data.push_back(cords[2]);
      pub_cords.publish(fma);
    }
    
    void intrinsiccallback(const sensor_msgs::CameraInfo::ConstPtr& camInfo_msg)
    {
      ROS_INFO_STREAM("Recieving intrinsics");
      intrinsicMatrix.fx = camInfo_msg->K[0];
      intrinsicMatrix.fy = camInfo_msg->K[4];

      //only when a distortion is applied
      /*
      for(int i = 0;i<5;i++){
        intrinsicMatrix.coeffs[i] = camInfo_msg->D[i];
      }
      */

      intrinsicMatrix.height = camInfo_msg->height;
      intrinsicMatrix.width = camInfo_msg->width;
      intrinsicMatrix.model = rs_distortion::RS_DISTORTION_NONE; 
      intrinsicMatrix.ppx = camInfo_msg->K[2];
      intrinsicMatrix.ppy = camInfo_msg->K[5];
      sub = this->n->subscribe("/pixel_Cords", 10, &MyDeprojector::cordinatecallback, this);
      hasIntrinsics = true;
      ROS_INFO_STREAM("Recieved intrinsics");
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deprojection");
  ros::NodeHandle * n = new ros::NodeHandle();
  
  MyDeprojector runningNode = MyDeprojector(n);

  

  ros::spin();

  return 0;
}