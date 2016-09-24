#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char** argv)
{
  srand (time(NULL));

  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("ir0_scan", 50);

  ros::NodeHandle nh("~");

  unsigned int num_readings = 6;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];
/*
  tf::TransformBroadcaster br;
  tf::Transform transform;

  double frame_x  = 0.28;
  double frame_y  = -1.3;
  double frame_z  = -0.11;
  nh.setParam("frame_x", frame_x);
  nh.setParam("frame_y", frame_y);
  nh.setParam("frame_z", frame_z);
*/


  int count = 0;
  ros::Rate r(20.0);

  double intense = 0.0;

  while(n.ok())
    {
    //generate some fake data for our laser scan
    intense = 0.5 + (double)(rand() % 10 + 1) / (double)200;
    for(unsigned int i = 0; i < num_readings; ++i)
    {
/*
      ranges[i] = count;
      intensities[i] = count;
*/

        /* generate secret number between 1 and 10: */
        ranges[i] = intense;
        intensities[i] = intense;

    }
    ros::Time scan_time = ros::Time::now();
/*
    nh.getParam("frame_x", frame_x);
    nh.getParam("frame_y", frame_y);
    nh.getParam("frame_z", frame_z);

    transform.setOrigin( tf::Vector3(frame_x, frame_y, frame_z) );
    transform.setRotation( tf::Quaternion(1, 0, 0, 1) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Head", "ir_sensor0_frame"));

    r.sleep();
*/
    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "ir_sensor0_frame";
    scan.angle_min = -0.03;
    scan.angle_max = 0.03;
    scan.angle_increment = 0.01;
    scan.time_increment = 0.05;
    scan.range_min = 0.0;
    scan.range_max = 2.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i)
    {
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    r.sleep();
  }
}
