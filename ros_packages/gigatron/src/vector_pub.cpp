/**
 * vector_pub.cpp
 * Gigatron publisher for motor control and steering tests.
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2015-09-13    creation
 *
 **/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"


/**
 * This node demonstrates simple sending of messages over the ROS system.
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
  ros::init(argc, argv, "vector_pub");

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
  ros::Publisher control_pub = n.advertise<geometry_msgs::Vector3>("control", 1000);

  /**
   * A ros::Rate object allows you to specify a frequency that you would 
   * like to loop at. It will keep track of how long it has been since 
   * the last call to Rate::sleep(), and sleep for the correct amount of time.
   */
  ros::Rate loop_rate(5);  // run at 5hz

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;


  /**
   * This is a message object. You stuff it with data, and then publish it.
   */
  geometry_msgs::Vector3 msg; //$ command message



  msg.x = 0;
  msg.y = 0;
  msg.z = 0;

  while (ros::ok() && count < 15)
  {

    
    msg.y = count;
    msg.z = count;
    // ROS_INFO_STREAM is a replacement for cout
    ROS_INFO_STREAM("Motor velocity command: " << msg.y << " [m/s]");
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    control_pub.publish(msg);

    /** 
     * Calling ros::spinOnce() here is not necessary for this simple program, 
     * because we are not receiving any callbacks. However, if you were to 
     * add a subscription into this application, and did not have ros::spinOnce() 
     * here, your callbacks would never get called. So, add it for good measure.
     */
    ros::spinOnce();

    /**
     * Now we use the ros::Rate object to sleep for the time remaining to let us 
     * hit our 10hz publish rate.
     */
    loop_rate.sleep();
    ++count;
  }

  ROS_INFO_STREAM("Slowing down!");
  while (ros::ok() && count > 0)   // count down
  {

    msg.x = 10.0 * (3.14159 / 180.0); 
    msg.y = count;
    msg.z = count;

    ROS_INFO_STREAM("Motor velocity command: " << msg.y << " [m/s]");
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    --count;
  }
  count = 50;
  ROS_INFO_STREAM("Going at a constant 5 m/s!");
  while (ros::ok() && count > 0)   // count down
  {
    msg.x = 20.0 * (3.14159 / 180.0); 
    msg.y = 5;
    msg.z = 5;

    ROS_INFO_STREAM("Motor velocity command: " << msg.y << " [m/s]");
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    --count;
  }
  ROS_INFO_STREAM("SPEED!");
  while (ros::ok() && count < 15)   // count up again, but faster
  {
    msg.x = 0.0; 
    msg.y = count;
    msg.z = count;

    ROS_INFO_STREAM("Motor velocity command: " << msg.y << " [m/s]");
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    count = count + 2;
  }
  ROS_INFO_STREAM("Slowing down again, but FASTER!");
  while (ros::ok() && count >= 0)   // count down again, but faster
  {
    msg.x = -10.0 * (3.14159 / 180.0); 
    msg.y = count;
    msg.z = count;

    ROS_INFO_STREAM("Motor velocity command: " << msg.y << " [m/s]");
    control_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    count = count - 2;
  }

  ROS_INFO_STREAM("DONE!");

  return 0;
}
