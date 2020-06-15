#include "ros/ros.h"
#include "ros/time.h"
#include "autonomous_eating/deproject_array.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
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
bool new_color_img = false, new_depth_img = false, find_face_trigger = false;

struct faceData{
  Rect mouth;
  Rect face;
  bool success;
};

class Face_worker
{
  CascadeClassifier face_cascade; 
  CascadeClassifier mouth_cascade;
  bool debug;

public:
  void init(String face_cascade_filename, String mouth_cascade_filename, bool debug);
  Mat drawFaces(Mat img, faceData data);
  faceData detectFace(Mat frame);
};

void Face_worker::init(String face_cascade_filename, String mouth_cascade_filename, bool _debug)
{
  //load data to cascades
  face_cascade.load(face_cascade_filename);
  mouth_cascade.load(mouth_cascade_filename);
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
  faceData data;
  data.mouth = Rect();
  data.face = Rect();
  data.success = true;

  std::vector<Rect> faces;
  face_cascade.detectMultiScale(frame_gray, faces);

  if(faces.size() != 1)
  {
    if(debug)
      ROS_INFO_STREAM("Too many faces detected");
    data.success = false;
    return data;
  }
  data.face = faces[0];

  std::vector<Rect> mouths;
  // entire face ROI
  Mat face_ROI = frame_gray(faces.at(0));
  Mat mouth_ROI = face_ROI.rowRange(face_ROI.rows / 2, face_ROI.rows);
  mouth_cascade.detectMultiScale(mouth_ROI , mouths);
  

  if(mouths.size() != 1)
  {
    ROS_INFO_STREAM("too many mouths detected");
    data.success = false;
    return data;
  }

  Point2i mouthROI_TL = faces[0].tl();
  mouthROI_TL.y +=  (faces[0].height /2);

  data.mouth.x = (mouths[0].tl() + mouthROI_TL).x;
  data.mouth.y = (mouths[0].tl() + mouthROI_TL).y;
  data.mouth.width = mouths[0].width;
  data.mouth.height = mouths[0].height;

  // optionally draw faces, and mouth on the frame
  if(debug)
  {
    rectangle(frame, faces[0].tl(), faces[0].br(), Scalar(255,0,255), 4);
    for(int i = 0; i < mouths.size(); i++)
      rectangle(frame, mouths[i].tl() + mouthROI_TL, mouths[i].br() + mouthROI_TL, Scalar(0,255,0), 4);

    //-- Show image
    imshow( "Capture - Face detection", frame );
    waitKey(1);
  }
  
  return data;
}

Mat Face_worker::drawFaces(Mat img, faceData data)
{
  
  Mat output;
  img.copyTo(output);
  
  rectangle(output, data.mouth.tl(), data.mouth.br(), Scalar(0, 255, 0), 4);
  rectangle(output, data.face.tl(), data.face.br(), Scalar(255,0,255), 4);
  
  return output;
}

void image_raw_callback(const sensor_msgs::ImageConstPtr& msg);

void depth_raw_callback(const sensor_msgs::ImageConstPtr& msg);

void find_face_callback(const std_msgs::BoolConstPtr &msg);

int main( int argc, char* argv[] )
{
  bool debug = false;
  ros::init(argc, argv, "face_detector_node");
  ros::NodeHandle n;


  String face_classifier_filename = "/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/extra/haarcascade_frontalface_alt.xml";
  String mouth_classifier_filename = "/home/ubuntu/Desktop/catkin_ws/src/autonomous_eating/extra/haarcascade_mcs_mouth.xml";
  Face_worker faceworker;
  faceworker.init(face_classifier_filename, mouth_classifier_filename, debug);

  ros::Subscriber color_image_raw_sub = n.subscribe("/r200/camera/color/image_raw", 5, &image_raw_callback);
  ros::Subscriber depth_image_raw_sub = n.subscribe("/r200/camera/depth/image_raw", 5, &depth_raw_callback);
  ros::Subscriber find_face_sub = n.subscribe("/find_face_trigger", 5, &find_face_callback);
  ros::ServiceClient deproject_client = n.serviceClient<autonomous_eating::deproject_array>("deproject_pixel_to_world_array");
  ros::Publisher pub_cords = n.advertise<autonomous_eating::face_cords>("/face_cords",5);
  ros::Publisher face_img_pub = n.advertise<sensor_msgs::Image>("/gui_figure", 5);
  ros::Publisher img_processed_pub = n.advertise<std_msgs::Int32>("/images_processed", 5);
  faceData faces;
  ROS_INFO_STREAM("initialized, ready to find faces!");

  //total landmarks: 5
  int n_usedPoints = 5;
  float depthLandmarkX[n_usedPoints];
  float depthLandmarkY[n_usedPoints];
  float depthLandmarkZ[n_usedPoints];
  float mouthMidPointInFront[3];
  float disFromFace = 1;
  float startingDis = 0.8;
  Mat A(n_usedPoints,3,CV_32FC1); // Mat K = (A.t() * A).inv() * A.t() * B;
  Mat B(n_usedPoints,1,CV_32FC1);
  Mat K(1,3,CV_32FC1);
  autonomous_eating::face_cords face_cords_msg;
  int64 startElse, endElse, postDeproject, preDeproject, start; // debug timings
  int images_processed = 0;
  while (ros::ok()){

    if(new_color_img && new_depth_img && find_face_trigger)
    {
      if(debug)
      {
        start = getTickCount();
        endElse = start;
      }
      new_depth_img = false;
      new_color_img = false;

      std_msgs::Int32 int_msg;
      int_msg.data = images_processed;
      img_processed_pub.publish(int_msg);

      images_processed++;

      faces = faceworker.detectFace(color_image);
      if(!faces.success)
      {
        //failed detection
        if(debug)
          ROS_WARN_STREAM("MOUTH DETECTION FAILED");

        //publish empty image to gui:
        cvtColor(color_image, color_image, COLOR_BGR2RGB);
        cv_bridge::CvImage msg;
        msg.encoding = "rgb8";
        msg.header.frame_id = "none";
        msg.header.seq = 0;
        msg.header.stamp = ros::Time::now();
        msg.image = color_image;
        face_img_pub.publish(msg);
        continue;
      }
  
      

      Mat face_img = faceworker.drawFaces(color_image, faces);
      cvtColor(face_img, face_img, COLOR_BGR2RGB);
      cv_bridge::CvImage msg;
      msg.encoding = "rgb8";
      msg.header.frame_id = "none";
      msg.header.seq = 0;
      msg.header.stamp = ros::Time::now();
      msg.image = face_img;
      face_img_pub.publish(msg);


      if(debug)
        imshow("depth", depth_image);
      

      if(debug)
        preDeproject = getTickCount();

      // fit a plane to the face:
      // first find five points, (a cross in the ROI)
      // ensure coordinates fit in depth_image even if resolution is not 1:1
      
      autonomous_eating::deproject_array srv;

      srv.request.x.push_back(faces.mouth.tl().x);
      for(int i = 0; i < 3; i++)
        srv.request.x.push_back(faces.mouth.tl().x + (faces.mouth.width/2));
      srv.request.x.push_back(faces.mouth.tl().x + faces.mouth.width);

      srv.request.y.push_back(faces.mouth.tl().y + (faces.mouth.height/2));
      srv.request.y.push_back(faces.mouth.tl().y);
      srv.request.y.push_back(faces.mouth.tl().y + (faces.mouth.height/2));
      srv.request.y.push_back(faces.mouth.tl().y + faces.mouth.height);
      srv.request.y.push_back(faces.mouth.tl().y + (faces.mouth.height/2));


      for (size_t i = 0; i < 5; i++){
        srv.request.z.push_back((float)depth_image.at<uint16_t>(Point(srv.request.x[i], srv.request.y[i])));
        //cout << srv.request.z[i] << endl;
      }
      //cout << endl;
      for(int i = 0; i < srv.request.z.size(); i++)
      {
        if(srv.request.z[i] > 1800)
        {
          ROS_INFO_STREAM("depth out of range for srv.request.z");
          continue;
        }
      }

      if(deproject_client.call(srv)){ //successfully called service
        for(int i = 0; i < srv.response.x.size(); i++)
        {
          depthLandmarkX[i] = srv.response.x[i];
          depthLandmarkY[i] = srv.response.y[i];
          depthLandmarkZ[i] = srv.response.z[i];
        }
      }
      else{ //failed to call service
        ROS_WARN_STREAM("FAILED TO CALL DEPROJECT SERVICE!");
        continue;
      }

      if(debug)
        postDeproject = getTickCount();

      // The equation for a plane is: ax+by+c=z. And we have already found x y z above
      // To find a b c we use formula A*K=B, where 
      // K is a 3 by 1 colum vector containing [a; b; c]
      // A is a 51 by 3 matrix containing [xi, yi, 1] and 
      // B is a 51 by 1 colum vector containing [zi]

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

      K = (A.t() * A).inv() * A.t() * B;
      // a = K.at<float>(Point(0,0)), b = K.at<float>(Point(0,1)) and c = K.at<float>(Point(0,2))
      
      // calculating xyz of first mouth corner projected onto plane
      float k1 = (-(K.at<float>(Point(0,0)) * depthLandmarkX[0]) - (K.at<float>(Point(0,1)) * depthLandmarkY[0]) - (K.at<float>(Point(0,2)) * depthLandmarkZ[0])) / ((K.at<float>(Point(0,0)) * K.at<float>(Point(0,0))) + (K.at<float>(Point(0,1)) * K.at<float>(Point(0,1))) + (K.at<float>(Point(0,2)) * K.at<float>(Point(0,2))));
      float x1 = (K.at<float>(Point(0,0)) * k1) + depthLandmarkX[0];
      float y1 = (K.at<float>(Point(0,1)) * k1) + depthLandmarkY[0];
      float z1 = (K.at<float>(Point(0,2)) * k1) + depthLandmarkZ[0];

      // calculating xyz of second mouth corner projected onto plane 
      float k2 = (-(K.at<float>(Point(0,0)) * depthLandmarkX[4]) - (K.at<float>(Point(0,1)) * depthLandmarkY[4]) - (K.at<float>(Point(0,2)) * depthLandmarkZ[4])) / ((K.at<float>(Point(0,0)) * K.at<float>(Point(0,0))) + (K.at<float>(Point(0,1)) * K.at<float>(Point(0,1))) + (K.at<float>(Point(0,2)) * K.at<float>(Point(0,2))));
      float x2 = (K.at<float>(Point(0,0)) * k2) + depthLandmarkX[4];
      float y2 = (K.at<float>(Point(0,1)) * k2) + depthLandmarkY[4];
      float z2 = (K.at<float>(Point(0,2)) * k2) + depthLandmarkZ[4];

      // calculating xyz of the middle of the mouth projected onto the plane and pulling it out in front of the mouth (middle point + vector)
      face_cords_msg.x_p1 = ((x1 + x2) / 2) + (K.at<float>(Point(0,0)) * startingDis);
      face_cords_msg.y_p1 = ((y1 + y2) / 2) + (K.at<float>(Point(0,1)) * startingDis);
      face_cords_msg.z_p1 = ((z1 + z2) / 2) + (K.at<float>(Point(0,2)) * startingDis);

      face_cords_msg.x_p2 = ((x1 + x2) / 2) + (K.at<float>(Point(0,0)) * disFromFace);
      face_cords_msg.y_p2 = ((y1 + y2) / 2) + (K.at<float>(Point(0,1)) * disFromFace);
      face_cords_msg.z_p2 = ((z1 + z2) / 2) + (K.at<float>(Point(0,2)) * disFromFace);



      // publish face_cords here

      pub_cords.publish(face_cords_msg);
      if(debug)
      {
        auto end = getTickCount();
        auto prepareTime = (preDeproject-start)/getTickFrequency();
        auto deprojectTime = (postDeproject - preDeproject)/getTickFrequency();
        auto mathTime = (end - postDeproject)/getTickFrequency();
        auto otherTime = (endElse - startElse)/getTickFrequency();
        auto loopTime = (end - startElse)/getTickFrequency();
        cout << "Prepare time: " << prepareTime << endl
            << "deproject time: " << deprojectTime << endl
            << "mathTime: " << mathTime << endl
            << "else time: " << otherTime << endl
            << "total loop time: " << loopTime << endl << endl;
      
        startElse = end;
      }
      
      
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

void find_face_callback(const std_msgs::BoolConstPtr &msg)
{
  find_face_trigger = msg->data;
}
