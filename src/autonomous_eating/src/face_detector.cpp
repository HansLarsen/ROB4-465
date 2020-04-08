#include "ros/ros.h"
#include "ros/time.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::face;


class Face_worker
{
  Face_worker();
  Face_worker(String face_cascade_filename, String landmark_model_filename);
  ~Face_worker();
  CascadeClassifier face_cascade; 
  Ptr<Facemark> facemark;

  void detectFace();
};

Face_worker::Face_worker(String face_cascade_filename, String landmark_model_filename)
{
  //load trained data to viola-jones detector
  face_cascade.load(face_cascade_filename);
  //ready facial landmark detection
  FacemarkLBF::Params params;
  params.model_filename = "lbfmodel.yaml"; // the trained model
  facemark = FacemarkLBF::create(params);
  facemark->loadModel(params.model_filename);
}
void Face_worker::detectFace()
{
  Mat frame;
  if( frame.empty() )
  {
    cout << "--(!) No captured frame -- Break!\n";
    return;
  }

  //convert to gray
  Mat frame_gray;
  cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
  //resize frame first for better performance:
  Size scale = frame_gray.size();
  scale.width = scale.width/2;
  scale.height = scale.height/2;
  resize(frame_gray,frame_gray,scale);
  resize(frame, frame, scale);
  //-- 3. Apply the classifier to the frame

  equalizeHist( frame_gray, frame_gray );
  //-- Detect faces
  std::vector<Rect> faces;
  face_cascade.detectMultiScale( frame_gray, faces );

  vector<vector<Point2f>> landmarks;
  facemark->fit(frame,faces,landmarks);

  for ( size_t i = 0; i < faces.size(); i++ )
  {
      rectangle(frame, faces[i].tl(), faces[i].br(), Scalar(255,0,255), 4);
      for(int j = 0; j < landmarks[i].size(); j++)
      {
          Point2f point = landmarks.at(i).at(j);
          ellipse(frame, point, Size(10,10),0,0,0, Scalar(0,255,0),3);
      }

  }
  //-- Show what you got
  imshow( "Capture - Face detection", frame );
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "face_detector_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);


  while (ros::ok()){
    //master_node->handleGesture();

    ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}

