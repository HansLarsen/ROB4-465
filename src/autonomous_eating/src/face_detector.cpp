#include "ros/ros.h"
#include "ros/time.h"
#include "opencv2/core/version.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/face.hpp"
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::face;

struct faceData
{
  std::vector<Rect> faces;
  vector<vector<Point2f>> landmarks;
};

class Face_worker
{
  CascadeClassifier face_cascade; 
  Ptr<Facemark> facemark;

public:
  void init(String face_cascade_filename, String landmark_model_filename);
  faceData detectFace(Mat frame);
  ~Face_worker();
};

void Face_worker::init(String face_cascade_filename, String landmark_model_filename)
{
  //load trained data to viola-jones detector
  face_cascade.load(face_cascade_filename);
  //ready facial landmark detection
  FacemarkLBF::Params params;
  params.model_filename = landmark_model_filename; // the trained model
  facemark = FacemarkLBF::create(params);
  facemark->loadModel(params.model_filename);
}

faceData Face_worker::detectFace(Mat frame)
{
  if( frame.empty() )
  {
    ROS_ERROR_STREAM("EMPTY FACE FRAME, in: Face_worker::detectFace");// << "--(!) No captured frame -- Break!\n";
  }

  //convert img to gray
  Mat frame_gray;
  cvtColor( frame, frame_gray, COLOR_BGR2GRAY );

  //resize frame first for better performance:
  Size scale = frame_gray.size();
  float to_scale = 300000.0/(frame.cols*frame.rows);
  scale.width = scale.width*to_scale;
  scale.height = scale.height*to_scale;
  resize(frame_gray,frame_gray,scale);

  //-- 3. Apply the classifier to the frame
  equalizeHist( frame_gray, frame_gray );
  //-- Detect faces
  std::vector<Rect> faces;
  face_cascade.detectMultiScale( frame_gray, faces );

  //fit landmarks:
  vector<vector<Point2f>> landmarks;
  facemark->fit(frame,faces,landmarks);

  //save in faceData struct to return it:
  faceData data;
  data.faces = faces;
  data.landmarks = landmarks;
  return data;


  /* optionally draw faces, and landmarks on the frame
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
  */
}

Face_worker::~Face_worker()
{
  delete facemark;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "face_detector_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  String face_classifier_filename = "insert_me";
  String face_landmark_filename = "lbfmodel.yaml";
  Face_worker faceworker;
  faceworker.init(face_classifier_filename,face_landmark_filename);


  while (ros::ok()){
    //master_node->handleGesture();

    ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}

