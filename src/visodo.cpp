/*

The MIT License

Copyright (c) 2015 Avi Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 1000

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!
timeval previous_timer, current_timer;
double odom_x=0, odom_y=0, odom_z=0;
nav_msgs::Odometry odom;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  gettimeofday(&current_timer,NULL);
  double time_interval = (current_timer.tv_sec-previous_timer.tv_sec)+(current_timer.tv_usec-previous_timer.tv_usec)*1e-6;
  odom_x += time_interval * msg->twist.twist.linear.x;
  odom_y += time_interval * msg->twist.twist.linear.y;
  odom_z += time_interval * msg->twist.twist.linear.z;
  previous_timer = current_timer;
}

int main( int argc, char** argv )	{

  ros::init(argc, argv, "cs6320");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("iRobot_0/odom", 1, odom_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_mono", 50);

  // initial timer
  gettimeofday(&current_timer,NULL);
  previous_timer = current_timer;

  Mat img_1, img_2;
  Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

  //ofstream myfile;
  //myfile.open ("results1_1.txt");

  double scale = 1.00;
  VideoCapture cap(1); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
    return -1;

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);

  //read the first two frames from the dataset
  Mat img_1_c;
  cap >> img_1_c;
  sleep(1);
  Mat img_2_c;
	cap >> img_2_c;

  if ( !img_1_c.data || !img_2_c.data ) { 
    std::cout<< " --(!) Error reading images " << std::endl; return -1;
  }

  // we work with grayscale images
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
  featureDetection(img_1, points1);        //detect features in img_1
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

  //TODO: add a fucntion to load these values directly from KITTI's calib files
  // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
  double focal = 578;
  cv::Point2d pp(333, 249);
  //recovering the pose and the essential matrix
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;

  R_f = R.clone();
  t_f = t.clone();

  clock_t begin = clock();

  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

  Mat traj = Mat::zeros(600, 600, CV_8UC3);
  
	while(ros::ok()){
  	Mat currImage_c;
  	cap >> currImage_c;
  	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
  	vector<uchar> status;
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


   for(int i=0;i<prevFeatures.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
  		prevPts.at<double>(0,i) = prevFeatures.at(i).x;
  		prevPts.at<double>(1,i) = prevFeatures.at(i).y;

  		currPts.at<double>(0,i) = currFeatures.at(i).x;
  		currPts.at<double>(1,i) = currFeatures.at(i).y;
    }

   scale = sqrt(odom_x*odom_x+odom_y*odom_y+odom_z*odom_z);
   odom_x = 0; odom_y = 0; odom_z = 0;

    //cout << "Scale is " << scale << endl;

    if ((scale>0.01)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;

      odom.header.stamp = ros::Time::now();
      odom.pose.pose.position.x = scale*t.at<double>(0);
      odom.pose.pose.position.y = scale*t.at<double>(1);
      odom.pose.pose.position.z = scale*t.at<double>(2);
      geometry_msgs::Quaternion qwt;
      CalculateRotation(R, qwt);
      odom.pose.pose.orientation = qwt;
      odom_pub.publish(odom);
    }
  	
    else {
     cout << "scale below 0.1, or incorrect translation" << endl;
    }
    
   // lines for printing results
   // myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

  // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
 	  if (prevFeatures.size() < MIN_NUM_FEAT)	{
      //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
      cout << "trigerring redection" << endl;
 		  featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);

 	  }

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Road facing camera", currImage_c );
    imshow( "Trajectory", traj );

    waitKey(1);
  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  //cout << R_f << endl;
  //cout << t_f << endl;

  return 0;
}
