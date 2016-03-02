#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

#include <unistd.h>

#include <string>
#include <sstream>

using std::string;
using std::vector;
using std::exception;
using std::cerr;
using std::endl;

serial::Serial ser;

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
        device.hardware_id.c_str() );
    }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  enumerate_ports();

  //serial::Serial joystick_serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));

  int retryNum = 0;

  while (!ser.isOpen() && retryNum < 10000)
  {
    try
    {
      ser.setPort("/dev/ttyACM0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      if (ser.isOpen())
      {
    	  ser.close();
      }
      ser.open();
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("Unable to open port; Retry #%d", ++retryNum);
      usleep(10000);
    }
  }

  
  ROS_INFO("%s%s", "Is the serial port open? ", (ser.isOpen() ? "[Yes]" : "[No]"));

//  size_t bytes_wrote = joystick_serial.write("START");

  size_t bytes_wrote = ser.write("START");

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
  ros::init(argc, argv, "talker");

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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(100);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    //ss << "hello world " << count;
    //string reading = joystick_serial.read();

    string reading;
/*
    joystick_serial.readline(reading);
    reading.erase(std::remove(reading.begin(), reading.end(), '\n'), reading.end());
*/

    if(ser.available())
    {
      ser.readline(reading);
      reading.erase(std::remove(reading.begin(), reading.end(), '\n'), reading.end());
      ss << reading;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
    
      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      chatter_pub.publish(msg);
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
