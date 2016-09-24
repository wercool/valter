#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  srand (time(NULL));

  ros::init(argc, argv, "ir0_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster br;
  tf::Transform transform;

  double frame_x  = 0.28;
  double frame_y  = -1.3;
  double frame_z  = -0.11;

  ros::Rate r(100.0);

    while(n.ok())
    {
        transform.setOrigin( tf::Vector3(frame_x, frame_y, frame_z) );
        transform.setRotation( tf::Quaternion(1, 0, 0, 1) );

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Head", "ir_sensor0_frame"));

        r.sleep();
    }
}
