
#include "vo_features.h"

using namespace cv;
using namespace std;

nav_msgs::Odometry odom_mono,odom_kine;
double delay=0;double lastx,lasty,lastz;
void odommono_callback(const nav_msgs::Odometry::ConstPtr& msg){
  odom_mono.pose.pose.position.x = odom_mono.pose.pose.position.x+msg->pose.pose.position.x;
  odom_mono.pose.pose.position.y = odom_mono.pose.pose.position.y+msg->pose.pose.position.y;
  odom_mono.pose.pose.position.z = odom_mono.pose.pose.position.z+msg->pose.pose.position.z;
  odom_mono.pose.pose.orientation.w = odom_mono.pose.pose.orientation.w+msg->pose.pose.orientation.w;
  odom_mono.pose.pose.orientation.x = odom_mono.pose.pose.orientation.x+msg->pose.pose.orientation.x;
  odom_mono.pose.pose.orientation.y = odom_mono.pose.pose.orientation.y+msg->pose.pose.orientation.y;
  odom_mono.pose.pose.orientation.z = odom_mono.pose.pose.orientation.z+msg->pose.pose.orientation.z;
}
void odomkine_callback(const nav_msgs::Odometry::ConstPtr& msg){
  odom_kine.pose.pose.position.x = odom_kine.pose.pose.position.x+msg->pose.pose.position.x;
  odom_kine.pose.pose.position.y = odom_kine.pose.pose.position.y+msg->pose.pose.position.y;
  odom_kine.pose.pose.position.z = odom_kine.pose.pose.position.z+msg->pose.pose.position.z;
  odom_kine.pose.pose.orientation.w = odom_kine.pose.pose.orientation.w+msg->pose.pose.orientation.w;
  odom_kine.pose.pose.orientation.x = odom_kine.pose.pose.orientation.x+msg->pose.pose.orientation.x;
  odom_kine.pose.pose.orientation.y = odom_kine.pose.pose.orientation.y+msg->pose.pose.orientation.y;
  odom_kine.pose.pose.orientation.z = odom_kine.pose.pose.orientation.z+msg->pose.pose.orientation.z;
  if (lastx==msg->pose.pose.position.x&&lasty==msg->pose.pose.position.y&&lastz==msg->pose.pose.position.z){delay=delay+0.01;}else delay=0;
  lastx=msg->pose.pose.position.x;
  lasty=msg->pose.pose.position.y;
  lastz=msg->pose.pose.position.z;
}
int main( int argc, char** argv )	{

  ros::init(argc, argv, "cs6320");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  // from monocular
  std_srvs::Empty srv;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("add_two_ints");
  
  ros::Subscriber odom1_sub = n.subscribe<nav_msgs::Odometry>("odom_mono", 1, odommono_callback);
  // from kinect
  ros::Subscriber odom2_sub = n.subscribe<nav_msgs::Odometry>("odom_out", 1, odomkine_callback);
	while(ros::ok()){
	if (delay>0.5){client.call(srv);}
  }

  return 0;
}
