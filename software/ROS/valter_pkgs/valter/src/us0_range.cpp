#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <time.h>
#include <sensor_msgs/Range.h>

int main(int argc, char** argv)
{
  srand (time(NULL));

  ros::init(argc, argv, "us_range_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::Range>("us0_scan", 50);

  unsigned int num_readings = 6;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  tf::TransformBroadcaster br;
  tf::Transform transform;

  double frame_x  = 0.28;
  double frame_y  = -1.1;
  double frame_z  = -0.11;
  n.setParam("frame_x", frame_x);
  n.setParam("frame_y", frame_y);
  n.setParam("frame_z", frame_z);



  int count = 0;
  ros::Rate r(10.0);

  double intense = 0.0;

  while(n.ok())
  {

    n.getParam("frame_x", frame_x);
    n.getParam("frame_y", frame_y);
    n.getParam("frame_z", frame_z);

    transform.setOrigin( tf::Vector3(frame_x, frame_y, frame_z) );
    transform.setRotation( tf::Quaternion(1, 0, 0, 1) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Head", "us_sensor0_frame"));

    //populate the Range message
    sensor_msgs::Range range_msg;

    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = "us_sensor0_frame";
    range_msg.field_of_view = 0.25;
    range_msg.min_range = 0.03;
    range_msg.max_range = 2.0;
    range_msg.header.stamp = ros::Time::now();

    range_msg.range = 0.5 + (double)(rand() % 10 + 1) / (double)100;


    scan_pub.publish(range_msg);
    r.sleep();
  }
}
