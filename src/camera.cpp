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

using namespace cv;
using namespace std;
using namespace ros;



int main( int argc,char** argv)  
{   
    ros::init(argc, argv, "cs6320");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
		
		Mat edges;
		int count = 0;
		while (ros::ok())
    {
      Mat frame;
      cap >> frame; // get a new frame from camera
      
      cvtColor(frame, edges, COLOR_BGR2GRAY);
      GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
      Canny(edges, edges, 0, 60, 3);
      
      imshow( "Display window", edges );  
      if(count%100==0)cout<<".";
		  if(waitKey(30) >= 0) break;
		  ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
    //
    return 0 ;  
}  
