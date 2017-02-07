#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <lsd.h>

#include <cs6320/Signal.h>

using namespace cv;
using namespace std;
using namespace ros;



int main( int argc,char** argv)  
{   
    ros::init(argc, argv, "cs6320");
    ros::NodeHandle n;
    
    ros::Publisher signal_pub = n.advertise<cs6320::Signal>("position_attitude", 1000);
    ros::Rate loop_rate(100);
    
    int line_number=0;
    double * out;
    
    const cv::Mat input = cv::imread("/home/jack/catkin_ws/src/cs6320_project/figure/lena.jpg", 0); //Load as grayscale 
    int X=input.rows, Y=input.cols;
    
		cv::Mat dst;

		// Convert to double (much faster than a simple for loop)
		input.convertTo(dst, CV_64F, 1, 0);
		double *ptrDst = dst.ptr<double>(); 
	  if(dst.isContinuous()) {
				  
				for(int i = 0; i < dst.total(); ++i) {
				        double value = ptrDst[i]; // Or do some other stuff
				}
		} else {
				for(int i = 0; i < dst.rows; ++i) {
				    double *ptrDst = dst.ptr<double>(i);

				    for(int j = 0; j < dst.cols; ++j) {
				        double value = ptrDst[j];
				    }
				}
		}
		
		
		int count = 0;
		while (ros::ok())
    {
    
      imshow( "Display window", input );  
      if(count%100==0)cout<<".";
		  out = lsd(&line_number,ptrDst,X,Y);
		  cout<<line_number<<endl;
		  //for(int i=0;i<line_number;i++)
      //{
      //  for(int j=0;j<7;j++)
      //    printf("%f ",out[7*i+j]);
      //  printf("\n");
      //}
		  //cvWaitKey(0);  
		  if(waitKey(30) >= 0) break;
		  ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
    //
    return 0 ;  
}  
