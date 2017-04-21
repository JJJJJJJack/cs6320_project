#include "ros/ros.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

//this function automatically gets rid of points for which tracking fails

  vector<float> err;					
  Size winSize=Size(21,21);																								
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}

     }

}


void featureDetection(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void CalculateRotation( Mat a, geometry_msgs::Quaternion& q ){
  float trace = a.at<double>(0,0) + a.at<double>(1,1) + a.at<double>(2,2);
  if( trace > 0 ) {
    float s = 0.5f / sqrtf(trace+ 1.0f);
    q.w = 0.25f / s;
    q.x = ( a.at<double>(2,1) - a.at<double>(1,2) ) * s;
    q.y = ( a.at<double>(0,2) - a.at<double>(2,0) ) * s;
    q.z = ( a.at<double>(1,0) - a.at<double>(0,1) ) * s;
  } else {
    if ( a.at<double>(0,0) > a.at<double>(1,1) && a.at<double>(0,0) > a.at<double>(2,2) ) {
      float s = 2.0f * sqrtf( 1.0f + a.at<double>(0,0) - a.at<double>(1,1) - a.at<double>(2,2));
      q.w = (a.at<double>(2,1) - a.at<double>(1,2) ) / s;
      q.x = 0.25f * s;
      q.y = (a.at<double>(0,1) + a.at<double>(1,0) ) / s;
      q.z = (a.at<double>(0,2) + a.at<double>(2,0) ) / s;
    } else if (a.at<double>(1,1) > a.at<double>(2,2)) {
      float s = 2.0f * sqrtf( 1.0f + a.at<double>(1,1) - a.at<double>(0,0) - a.at<double>(2,2));
      q.w = (a.at<double>(0,2) - a.at<double>(2,0) ) / s;
      q.x = (a.at<double>(0,1) + a.at<double>(1,0) ) / s;
      q.y = 0.25f * s;
      q.z = (a.at<double>(1,2) + a.at<double>(2,1) ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + a.at<double>(2,2) - a.at<double>(0,0) - a.at<double>(1,1) );
      q.w = (a.at<double>(1,0) - a.at<double>(0,1) ) / s;
      q.x = (a.at<double>(0,2) + a.at<double>(2,0) ) / s;
      q.y = (a.at<double>(1,2) + a.at<double>(2,1) ) / s;
      q.z = 0.25f * s;
    }
  }
}
