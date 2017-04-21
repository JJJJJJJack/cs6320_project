#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <lsd.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

using namespace cv;
using namespace std;
using namespace ros;

Mat kinect_img;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    cv::imshow("Hi", cv_ptr->image);
    cvtColor(cv_ptr->image, kinect_img, COLOR_BGR2GRAY);

    // Output modified video stream
  }

int main( int argc,char** argv)  
{   
    ros::init(argc, argv, "cs6320");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber image_sub_;
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &imageCb);
    
    VideoCapture cap(1); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
		
		Mat monocam_img;
		int count = 0;
		while (ros::ok())
    {
      Mat frame;
      cap >> frame; // get a new frame from camera
      
      cvtColor(frame, monocam_img, COLOR_BGR2GRAY);
      //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
      //Canny(edges, edges, 0, 60, 3);
      
      imshow( "Display window", monocam_img );  
      if(count%100==0)cout<<".";
		  if(waitKey(30) >= 0) break;
		  
		  string monocam_imgname="/home/jack/catkin_ws/src/cs6320_project/figure/calibration/Monocam_";
			char mcbuff[20];
			sprintf (mcbuff, "%03d", count);
			monocam_imgname.append(mcbuff);
			monocam_imgname.append(".jpg");
			
			string kinect_imgname="/home/jack/catkin_ws/src/cs6320_project/figure/calibration/Kinect_";
			char kcbuff[20];
			sprintf (kcbuff, "%03d", count);
			kinect_imgname.append(kcbuff);
			kinect_imgname.append(".jpg");
		  
		  imwrite( monocam_imgname, monocam_img );
		  imwrite( kinect_imgname, kinect_img );
		  
		  ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
    //
    return 0 ;  
}  
