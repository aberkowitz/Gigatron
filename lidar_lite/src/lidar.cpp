/**
 * lidar.cpp
 * Gigatron publisher LIDAR-Lite data via I2C communication
 * 
 * @author  Syler Wagner  <syler@mit.edu>
 * @date    2015-07-11    creation
 *
 * This node publishes ROS msgs with the distance, previous distance, and velocity 
 * readings from a LIDAR-Lite sensor.
 * 
 * Usage Instructions:
 * 1. You need root privileges to access I2C data, so log in to root.
 *      sudo su
 * 2. Startup roscore in its own terminal window.
 *      roscore
 * 3. Start lidar in a separate window to publish the LIDAR-Lite data.
 *      rosrun lidar_lite lidar
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <lidarlite.h>
#include "ros/ros.h"
#include "lidar_lite/LidarMsg.h"

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

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
  ros::init(argc, argv, "lidar");

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
  ros::Publisher lidar_pub = n.advertise<lidar_lite::LidarMsg>("lidar", 1000);

  /**
   * A ros::Rate object allows you to specify a frequency that you would 
   * like to loop at. It will keep track of how long it has been since 
   * the last call to Rate::sleep(), and sleep for the correct amount of time.
   */
  ros::Rate loop_rate(10);  // run at 10hz

    LidarLite *lidarLite = new LidarLite(); // create a new LidarLite object
    int err = lidarLite->openLidarLite();
    if (err < 0) {
        printf("Error: %d", lidarLite->error);
    } 
    else {

        while(ros::ok() && lidarLite->error >= 0 && getkey() != 27) {  // 27 is the ESC key

            int distance = lidarLite->getDistance();
            int previousDistance = lidarLite->getPreviousDistance();
            // printf("Distance: %dcm\n", dist);
            int velocity = lidarLite->getVelocity();
            printf("Distance: %5d cm  |  Previous Distance: %5d cm   | Velocity: % 8d \n",distance,previousDistance,velocity);
            
            /**
             * This is a message object. You stuff it with data, and then publish it.
             */
            lidar_lite::LidarMsg msg;
            msg.distance = distance;
            msg.previous = previousDistance;
            msg.velocity = velocity;
            /**
             * The publish() function is how you send messages. The parameter
             * is the message object. The type of this object must agree with the type
             * given as a template parameter to the advertise<>() call, as was done
             * in the constructor above.
             */
            lidar_pub.publish(msg);

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
        }
    }
    
    lidarLite->closeLidarLite();
}
