#include "ros/ros.h"
#include "ros/time.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/face.hpp"
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
        rectangle(frame_gray, faces[i].tl(), faces[i].br(), Scalar(255,0,255), 4);
        for(int j = 0; j < landmarks[i].size(); j++)
        {
            Point2f point = landmarks.at(i).at(j);
            ellipse(frame_gray, point, Size(10,10),0,0,0, Scalar(0,255,0),3);
        }
    }
    //-- Show image
    imshow( "Capture - Face detection", frame_gray );
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
  bool debug = true;

  ros::init(argc, argv, "face_detector_node");
  ros::NodeHandle n;


  String face_classifier_filename = "/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/extra/haarcascade_frontalface_alt.xml";
  String face_landmark_filename = "/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/extra/lbfmodel.yaml";
  Face_worker faceworker;
  faceworker.init(face_classifier_filename,face_landmark_filename, debug);

  ros::Subscriber color_image_raw_sub = n.subscribe("/camera/color/image_raw", 5, &image_raw_callback);
  ros::Subscriber depth_image_raw_sub = n.subscribe("/camera/depth/image_raw", 5, &depth_raw_callback);

  faceData faces;
  ROS_INFO_STREAM("initialized, ready to find faces!");

  int n_usedPoints = faces.landmarks.size()-17;
  int depthLandmarkX[n_usedPoints];
  int depthLandmarkY[n_usedPoints];
  int depthLandmarkZ[n_usedPoints];
  Mat A(3,faces.landmarks.size(),CV_8UC1);
  Mat B(1,faces.landmarks.size(),CV_8UC1);

  while (ros::ok()){

    if(new_color_img && new_depth_img)
    {
      new_depth_img = false;
      new_color_img = false;
      faces = faceworker.detectFace(color_image);
      imshow("depth", depth_image);
      
      //fit a plane to the face:
      if(faces.faces.size() > 1)// we have multiple faces, skip until only one face is detected
      {
        if(debug)
          ROS_WARN_STREAM("MORE THAN 1 FACE DETECTED");
        continue;
      }
      
      // ensure coordinates fit in depth_image even if resolution is not 1:1
      for (size_t i = 0; i < n_usedPoints; i++)
      {
        depthLandmarkX[i]=(faces.landmarks[0].at(i+17).x / color_image.cols)*depth_image.cols;
        depthLandmarkY[i]=(faces.landmarks[0].at(i+17).y / color_image.rows)*depth_image.rows;
        depthLandmarkZ[i]=(int)depth_image.at<uchar>(depthLandmarkZ[i], depthLandmarkY[i]);
      }
      // The equation for a plane is: ax+by+c=z. And we have local x y z above
      // To find a b c we use formula Ak=B, where x is a array of a b c
      // A is [xi, yi, 1] and B is [zi]
      
      for (size_t x = 0; x < 3; x++){
        for (size_t y = 0; y < n_usedPoints; y++){        
          switch (x){          
            case 0:
              A.at<uchar>(Point(x,y)) = depthLandmarkX[y];
              B.at<uchar>(Point(x,y)) = depthLandmarkZ[y];
              break;            
            case 1:
              A.at<uchar>(Point(x,y)) = depthLandmarkY[y];
              break;            
            case 2:
              A.at<uchar>(Point(x,y)) = 1;
              break;
          }
        }        
      }
      Mat k = (A.t() * A).inv() * A.t() * B.t();
      // a = k.col[0], b = k.col[1] and c = k.col[2]

      //publish stuff here, add the publisher before the while loop (around line 130)
      //...
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