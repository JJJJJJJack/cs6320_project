#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>

#include <signal_sub_pub/Signal.h>


#include <time.h>

using namespace std;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "signal_publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher signal_pub = n.advertise<signal_sub_pub::Signal>("position_attitude", 1000);

  ros::Rate loop_rate(80);

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int count = 0;
  srand((int)time(0));
  signal_sub_pub::Signal data;
  bool Start_timer = true;

  ifstream pose_attitude;
  pose_attitude.open("/home/jack/Dropbox/PhD at Utah/Class/6225/Assignment/Final Project/Motion Planning Final Project/Final_Script/pose_attitude.txt", ios_base::in);
  if(!pose_attitude){// if it does not work
    cerr << "Can't open Data!\n";
    return 0;
  }
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    if(Start_timer == true){
      string line;
      if(getline(pose_attitude, line) == false){
        data.attitude_pitch = 0;
        data.attitude_roll = 0;
      }else{
        istringstream in(line);
        in>>data.pose_x>>data.pose_y>>data.attitude_pitch>>data.attitude_roll;
      }
    }
    data.header.stamp = ros::Time::now();

    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
    if(totaltime < 10){
      Start_timer = false;
      data.attitude_pitch = 0;
      data.attitude_roll = 0;
    }else
      Start_timer = true;
    if(totaltime < 11){
      data.attitude_pitch = data.attitude_pitch > 20/57.3 ? 20/57.3 : data.attitude_pitch;
      data.attitude_pitch = data.attitude_pitch < -20/57.3 ? -20/57.3 : data.attitude_pitch;
      data.attitude_roll = data.attitude_roll > 20/57.3 ? 20/57.3 : data.attitude_roll;
      data.attitude_roll = data.attitude_roll < -20/57.3 ? -20/57.3 : data.attitude_roll;
    }
    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    signal_pub.publish(data);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
