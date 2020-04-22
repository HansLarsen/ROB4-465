#include "ros/ros.h"
#include "ros/time.h"
#include "autonomous_eating/deproject.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/face.hpp"
#include "autonomous_eating/face_cords.h"
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::face;

Mat depth_image, color_image;
bool new_color_img = false, new_depth_img = false;
struct faceData
{
  std::vector<Rect> faces;
  vector<vector<Point2f>> landmarks;
};

class Face_worker
{
  CascadeClassifier face_cascade; 
  Ptr<Facemark> facemark;
  bool debug;

public:
  void init(String face_cascade_filename, String landmark_model_filename, bool debug);
  faceData detectFace(Mat frame);
  ~Face_worker();
};

void Face_worker::init(String face_cascade_filename, String landmark_model_filename, bool _debug)
{
  //load trained data to viola-jones detector
  face_cascade.load(face_cascade_filename);
  //ready facial landmark detection
  FacemarkLBF::Params params;
  params.model_filename = landmark_model_filename; // the trained model
  facemark = FacemarkLBF::create(params);
  facemark->loadModel(params.model_filename);
  debug = _debug;
}

faceData Face_worker::detectFace(Mat frame)
{
  if( frame.empty() )
  {
    ROS_ERROR_STREAM("EMPTY FACE FRAME, in: Face_worker::detectFace");// << "--(!) No captured frame -- Break!\n";
  }
  GaussianBlur(frame, frame, Size(5,5),0);
  Mat frame_gray;
  cvtColor( frame, frame_gray, COLOR_BGR2GRAY );

  //resize frame first for better performance, if necessary
  /*
  Size scale = frame_gray.size();
  float to_scale = 300000.0/(frame.cols*frame.rows);
  scale.width = scale.width*to_scale;
  scale.height = scale.height*to_scale;
  resize(frame_gray,frame_gray,scale);
  */

  //-- 3. Apply the classifier to the frame
  //equalizeHist( frame_gray, frame_gray ); //should be done for real images, bad for gazebo images
  //-- Detect faces
  std::vector<Rect> faces;
  face_cascade.detectMultiScale(frame_gray, faces);
  //fit landmarks:
  vector<vector<Point2f>> landmarks;
  facemark->fit(frame,faces,landmarks);

  //save in faceData struct to return it:
  faceData data;
  data.faces = faces;
  data.landmarks = landmarks;


  // optionally draw faces, and landmarks on the frame
  if(debug)
  {
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        rectangle(frame, faces[i].tl(), faces[i].br(), Scalar(255,0,255), 4);
        for(int j = 0; j < landmarks[i].size(); j++)
        {
            Point2f point = landmarks.at(i).at(j);
            ellipse(frame, point, Size(10,10),0,0,0, Scalar(0,255,0),3);
        }
    }
    //-- Show image
    imshow( "Capture - Face detection", frame );
    waitKey(1);
  }
  
  return data;
}

Face_worker::~Face_worker()
{
  delete facemark;
}

void image_raw_callback(const sensor_msgs::ImageConstPtr& msg);

void depth_raw_callback(const sensor_msgs::ImageConstPtr& msg);

int main( int argc, char* argv[] )
{
  string param;
  bool debug = false;

  ros::init(argc, argv, "face_detector_node");
  ros::NodeHandle n;


  String face_classifier_filename = "/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/extra/haarcascade_frontalface_alt.xml";
  String face_landmark_filename = "/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/extra/lbfmodel.yaml";
  Face_worker faceworker;
  faceworker.init(face_classifier_filename,face_landmark_filename, debug);

  ros::Subscriber color_image_raw_sub = n.subscribe("/r200/camera/color/image_raw", 5, &image_raw_callback);
  ros::Subscriber depth_image_raw_sub = n.subscribe("/r200/camera/depth/image_raw", 5, &depth_raw_callback);
  ros::ServiceClient deproject_client = n.serviceClient<autonomous_eating::deproject>("deproject_pixel_to_world");
  ros::Publisher pub_cords = n.advertise<autonomous_eating::face_cords>("/face_cords",1);
  faceData faces;
  ROS_INFO_STREAM("initialized, ready to find faces!");

  int n_usedPoints = faces.landmarks.size()-17;
  float depthLandmarkX[n_usedPoints];
  float depthLandmarkY[n_usedPoints];
  float depthLandmarkZ[n_usedPoints];
  float mouthMidPointInFront[3];
  float disFromFace = 5;
  float startingDis = 15;
  Mat A(3,faces.landmarks.size(),CV_32FC1);
  Mat B(1,faces.landmarks.size(),CV_32FC1);
  autonomous_eating::face_cords face_cords_msg;

  while (ros::ok()){

    if(new_color_img && new_depth_img)
    {
      new_depth_img = false;
      new_color_img = false;
      faces = faceworker.detectFace(color_image);
      if(debug)
        imshow("depth", depth_image);
      
      if(faces.faces.size() > 1)// we have multiple faces, skip until only one face is detected
      {
        if(debug)
          ROS_WARN_STREAM("MORE THAN 1 FACE DETECTED");
        continue;
      }
      if (faces.landmarks.size() == 0)
        continue;

      
      ROS_INFO_STREAM(faces.landmarks.size());
      //fit a plane to the face:
      // ensure coordinates fit in depth_image even if resolution is not 1:1

      autonomous_eating::deproject srv;
      for (size_t i = 17; i < faces.landmarks[0].size()-1; i++){
        srv.request.x = ((float)faces.landmarks[0].at(0).x / (float)color_image.cols)*(float)depth_image.cols;
        srv.request.y = ((float)faces.landmarks[0].at(0).y / (float)color_image.rows)*(float)depth_image.rows;
        srv.request.z = (float)depth_image.at<uint16_t>(srv.request.x, srv.request.y);
        
        if(deproject_client.call(srv)){ //successfully called service
    
          depthLandmarkX[i] = srv.response.x;
          depthLandmarkY[i] = srv.response.y;
          depthLandmarkZ[i] = srv.response.z;
          
        }
        else{ //failed to call service
          ROS_WARN_STREAM("FAILED TO CALL DEPROJECT SERVICE!");
        }
      }

/*
      // The equation for a plane is: ax+by+c=z. And we have x y z above
      // To find a b c we use formula Ak=B, where x is a array of a b c
      // A is [xi, yi, 1] and B is [zi]
      
      for (size_t i = 0; i < 3; i++){
        for (size_t j = 0; j < n_usedPoints; j++){        
          switch (i){          
            case 0:
              A.at<float>(Point(i,j)) = depthLandmarkX[j];
              B.at<float>(Point(i,j)) = depthLandmarkZ[j];
              break;            
            case 1:
              A.at<float>(Point(i,j)) = depthLandmarkY[j];
              break;            
            case 2:
              A.at<float>(Point(i,j)) = 1;
              break;
          }
        }        
      }
      
      Mat K = (A.t() * A).inv() * A.t() * B.t();
      // a = K.at<float>(0,0), b = K.at<float>(0,1) and c = K.at<float>(0,2)

      // calculating xyz of first mouth corner projected onto plane
      float k1 = (-(K.at<float>(0,0) * depthLandmarkX[62]) - (K.at<float>(0,1) * depthLandmarkY[62]) - (K.at<float>(0,2) * depthLandmarkZ[62])) / ((K.at<float>(0,0) * K.at<float>(0,0)) + (K.at<float>(0,1) * K.at<float>(0,1)) + (K.at<float>(0,2) * K.at<float>(0,2)));
      float x1 = (K.at<float>(0,0) * k1) + depthLandmarkX[62];
      float y1 = (K.at<float>(0,1) * k1) + depthLandmarkY[62];
      float z1 = (K.at<float>(0,2) * k1) + depthLandmarkZ[62];
      // calculating xyz of second mouth corner projected onto plane 
      float k2 = (-(K.at<float>(0,0) * depthLandmarkX[66]) - (K.at<float>(0,1) * depthLandmarkY[66]) - (K.at<float>(0,2) * depthLandmarkZ[66])) / ((K.at<float>(0,0) * K.at<float>(0,0)) + (K.at<float>(0,1) * K.at<float>(0,1)) + (K.at<float>(0,2) * K.at<float>(0,2)));
      float x2 = (K.at<float>(0,0) * k2) + depthLandmarkX[66];
      float y2 = (K.at<float>(0,1) * k2) + depthLandmarkY[66];
      float z2 = (K.at<float>(0,2) * k2) + depthLandmarkZ[66];
      
      // calculating xyz of the middle of the mouth projected onto the plane and pulling it out in front of the mouth (middle point + vector)
      face_cords_msg.x_p1 = ((x1 + x2) / 2) + (K.at<float>(0,0) * startingDis);
      face_cords_msg.y_p1 = ((y1 + y2) / 2) + (K.at<float>(0,1) * startingDis);
      face_cords_msg.z_p1 = ((z1 + z2) / 2) + (K.at<float>(0,2) * startingDis);

      face_cords_msg.x_p2 = ((x1 + x2) / 2) + (K.at<float>(0,0) * disFromFace);
      face_cords_msg.y_p2 = ((y1 + y2) / 2) + (K.at<float>(0,1) * disFromFace);
      face_cords_msg.z_p2 = ((z1 + z2) / 2) + (K.at<float>(0,2) * disFromFace);

      // publish face_cords here
      pub_cords.publish(face_cords_msg);
      */
    }
    ros::spinOnce();
  }
  return 0;
}

void image_raw_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  color_image = cv_ptr->image;
  new_color_img = true;
}

void depth_raw_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  depth_image = cv_ptr->image;
  new_depth_img = true;
}