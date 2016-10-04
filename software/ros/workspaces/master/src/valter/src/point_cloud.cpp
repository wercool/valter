#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");

    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    unsigned int num_points = 10000;

    int count = 0;
    ros::Rate r(1.0);

    double pX = 0.0;
    double pY = 0.0;
    double pZ = 0.0;
    double intense = 0.0;

    srand (time(NULL));

    while(n.ok())
    {

        transform.setOrigin( tf::Vector3(0.07, 0.2, 0.0) );
        transform.setRotation( tf::Quaternion(1, 0, 0, 1) );

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Head", "pcl_sensor_frame"));

        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "pcl_sensor_frame";

        cloud.points.resize(num_points);

        //we'll also add an intensity channel to the cloud
        cloud.channels.resize(1);
        cloud.channels[0].name = "intensities";
        cloud.channels[0].values.resize(num_points);

        //generate some fake data for our point cloud
        for(unsigned int i = 0; i < num_points; ++i)
        {
            pX = 1 + (double)(rand() % 10 + 1) / (double)10;
            pY = (double)(rand() % 40 + 1) / (double)10 - 2;
            pZ = (double)i / (double)5200; //(double)(rand() % 20 + 1) / (double)10;
            intense = 0.5 + (double)(rand() % 10 + 1) / (double)20;
            cloud.points[i].x = pX;
            cloud.points[i].y = pY;
            cloud.points[i].z = pZ;
            cloud.channels[0].values[i] = intense;
        }

        cloud_pub.publish(cloud);
        r.sleep();
    }

}
