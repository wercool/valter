#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <angles/angles.h>

// Simple Navigation States
typedef enum _SimpleNavigationState {
   
    SN_STOPPED = 1,
    SN_MOVING = 2,
    SN_ROTATING = 3,
    SN_MOVING_AS = 4,
    SN_ROTATING_AS = 5,
    SN_GOAL_ORIENTATION_FINALIZATION = 6
} SimpleNavigationState;

// Simple Navigation State
SimpleNavigationState state;

// Global frame_id
std::string global_frame_id;

// Target position
geometry_msgs::PoseStamped goal;
// Robot odometry
nav_msgs::Odometry odom;

bool rotate_in_place;

void NavGoalSetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;
    ROS_INFO("Goal [x,y,th]: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.z);

    if(rotate_in_place) 
        state = SN_ROTATING;
    else 
        state = SN_MOVING;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
    ROS_INFO("Odometry [x,y,th]: %f, %f, %f", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.orientation.z);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_simple");

    ROS_INFO("Move Valter's Base Simple for ROS");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Making all the publishing and subscriptions...
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odometry", 20, OdomCallback);
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, NavGoalSetCallback);

    // Parameters
    double goal_tolerance;
    double attraction_coefficient;
    double alpha;
    double max_linear_velocity;
    double min_linear_velocity;
    double angular_threshold;
    double in_place_angular_velocity;

    pn.param("goal_tolerance", goal_tolerance, 0.1);
    pn.param("attraction_coefficient", attraction_coefficient, 0.01);
    pn.param("alpha", alpha, 0.1);
    pn.param("max_linear_velocity", max_linear_velocity, 0.1);
    pn.param("min_linear_velocity", min_linear_velocity, 0.02);
    pn.param("angular_threshold", angular_threshold, 0.05);
    pn.param("in_place_angular_velocity", in_place_angular_velocity, 0.05);

    state = SN_STOPPED;

    if(angular_threshold == 0.0)
    {
        rotate_in_place = false;
        ROS_INFO("MoveBase Simple -- Not using in-place rotations.");
    }
    else
    {
        rotate_in_place = true;
        ROS_INFO("MoveBase Simple -- Using in-place rotations.");
    }

    // Main Loop
    ros::Rate r(30.0);

    while(n.ok())
    {
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;

        double current_orientation = tf::getYaw(odom.pose.pose.orientation);
        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;

        // Send the new velocities to the robot...
        geometry_msgs::Twist cmd_vel;

        // If we reached our target position
        if((state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS) && sqrt(pow(current_x-goal.pose.position.x,2)+pow(current_y-goal.pose.position.y,2)) < goal_tolerance)
        {
            state = SN_GOAL_ORIENTATION_FINALIZATION;
            linear_velocity = 0.0;
            angular_velocity = 0.0;
            ROS_INFO("Goal position reached with tolerance %f", goal_tolerance);

            double goal_orientation = tf::getYaw(goal.pose.orientation);
            double goal_current_diff = fabs(angles::shortest_angular_distance(current_orientation, goal_orientation));

            ROS_INFO("Goal to Current orientation diff: %f", goal_current_diff);

            if (goal_current_diff < angular_threshold)
            {
                state = SN_STOPPED;
                ROS_INFO("Goal reached finally");
            }

            cmd_vel.linear.x = linear_velocity;
            cmd_vel.linear.y = linear_velocity;
            cmd_vel.angular.z = angular_velocity;

            cmd_vel_pub.publish(cmd_vel);
        }

        if (state == SN_GOAL_ORIENTATION_FINALIZATION)
        {
            double goal_orientation = tf::getYaw(goal.pose.orientation);
            double goal_current_diff = fabs(angles::shortest_angular_distance(current_orientation, goal_orientation));

            ROS_INFO("Goal to Current orientation diff: %f", goal_current_diff);

            if (goal_current_diff > angular_threshold)
            {
                angular_velocity = -alpha*(angles::shortest_angular_distance(goal_orientation, current_orientation));
            }
            else
            {
                linear_velocity = 0.0;
                angular_velocity = 0.0;
                state = SN_STOPPED;
                ROS_INFO("Goal reached after finalization");
            }

            cmd_vel.linear.x = linear_velocity;
            cmd_vel.linear.y = linear_velocity;
            cmd_vel.angular.z = angular_velocity;

            cmd_vel_pub.publish(cmd_vel);
        }

        // If we are moving...
        if(state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS)
        {
            ROS_INFO("moving...");
            double G_attr_x = -attraction_coefficient*(current_x - goal.pose.position.x);
            double G_attr_y = -attraction_coefficient*(current_y - goal.pose.position.y);
            ROS_INFO("G_attr [x,y]: %f, %f", G_attr_x, G_attr_y);

            double target_orientation = atan2(G_attr_y, G_attr_x);

            linear_velocity = sqrt(G_attr_x*G_attr_x + G_attr_y*G_attr_y);

            if(fabs(linear_velocity) > max_linear_velocity) linear_velocity = (linear_velocity > 0 ? max_linear_velocity : -max_linear_velocity);
            if(fabs(linear_velocity) < min_linear_velocity) linear_velocity = (linear_velocity > 0 ? min_linear_velocity : -min_linear_velocity);

            angular_velocity = -alpha*(angles::shortest_angular_distance(target_orientation, current_orientation));

            // If we intend to rotate before moving forward...
            if(state == SN_ROTATING || state == SN_ROTATING_AS)
            {
                linear_velocity = 0.0;
                angular_velocity = (angular_velocity < 0 ? -in_place_angular_velocity : in_place_angular_velocity);

                if(fabs(angles::shortest_angular_distance(current_orientation, target_orientation)) < angular_threshold)
                {
                    angular_velocity = 0.0;
                    if(state == SN_ROTATING) state = SN_MOVING;
                    else if(state == SN_ROTATING_AS) state = SN_MOVING_AS;
                }
            }


            cmd_vel.linear.x = linear_velocity;
            cmd_vel.linear.y = linear_velocity;
            cmd_vel.angular.z = angular_velocity;

            cmd_vel_pub.publish(cmd_vel);

        }

        ros::spinOnce();
        r.sleep();
    }
    return(0);
}
