/*
 * torso_server.cpp
 *
 *  Created on: Oct 23, 2014
 *      Author: Janno Lunenburg
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

double EPSILON = 0.005; // tolerance for determining whether goal is reached (in meters)

actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>* as_;

ros::Publisher torso_pub_;
ros::Subscriber torso_sub_;

bool meas_received;
sensor_msgs::JointState torso_meas_;

void torsoMeasurementCb(const sensor_msgs::JointStateConstPtr& meas) {
    torso_meas_ = *meas;
    meas_received = true;
}

void setGoalTolerance(const control_msgs::FollowJointTrajectoryGoalConstPtr goal, std::vector<double>& goal_tolerance) {
    int n_joints = goal->trajectory.joint_names.size();
    goal_tolerance.resize(n_joints, EPSILON);
    for (unsigned int i = 0; i < n_joints; i++) {
        for (unsigned int j = 0; j < goal->goal_tolerance.size(); j++) {
            if (goal->trajectory.joint_names[i] == goal->goal_tolerance[j].name) {
                goal_tolerance[i] = goal->goal_tolerance[j].position;
            }
        }
    }
}

void goalCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal) {

    ros::Rate r(10);
    ros::NodeHandle n;
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = torso_goal;
    control_msgs::FollowJointTrajectoryFeedback feedback;
    sensor_msgs::JointState torso_ref;

    int goal_index = 0;
    int n_joints = goal->trajectory.joint_names.size();
    torso_ref.name = goal->trajectory.joint_names;
    feedback.joint_names = goal->trajectory.joint_names;
    std::vector<double> goal_tolerance;
    setGoalTolerance(goal, goal_tolerance);

    while(n.ok() && as_->isActive()) {
        
        if (!meas_received) {
            ROS_WARN("Torso server has not received any measurements yet, aborting");
            as_->setAborted();
            return;
        }

        torso_ref.header.stamp = ros::Time::now();

        // ToDo: check time?
        // ToDo: assumes measurements and references in the same order

        // Check if new goal is available
        if (as_->isNewGoalAvailable()) {
            ROS_INFO("New goal received.");
            goal = as_->acceptNewGoal();
            goal_index = 0;
            n_joints = goal->trajectory.joint_names.size();
            torso_ref.name = goal->trajectory.joint_names;
            feedback.joint_names = goal->trajectory.joint_names;
            setGoalTolerance(goal, goal_tolerance);
        }

        if (goal_index == goal->trajectory.points.size()) {
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = 0;
            as_->setSucceeded(result, "Torso goal succeeded!");
        } else {
            // Publish reference
            torso_ref.position = goal->trajectory.points[goal_index].positions;
            torso_ref.velocity = goal->trajectory.points[goal_index].velocities;
            torso_ref.effort = goal->trajectory.points[goal_index].effort;
            torso_pub_.publish(torso_ref);

            // Publish feedback
            feedback.header.stamp = ros::Time::now();
            feedback.actual.positions = torso_meas_.position;
            feedback.desired = goal->trajectory.points[goal_index];
            // ToDo: error?
            as_->publishFeedback(feedback);

            // Count number of converged joints
            unsigned int n_converged = 0;
            for (unsigned int i = 0; i < n_joints; i++) {
                if (fabs(torso_ref.position[i] - torso_meas_.position[i]) < goal_tolerance[i]) {
                    ++n_converged;
                }
            }
            if (n_converged == n_joints) {
                ++goal_index;
            }

        }

        r.sleep();

    }

}

int main(int argc, char** argv) {

    meas_received = false;

    ros::init(argc, argv, "torso_server");
	ros::NodeHandle nh("~");

    torso_pub_ = nh.advertise<sensor_msgs::JointState>("references", 1);
    torso_sub_ = nh.subscribe("measurements", 10, &torsoMeasurementCb);

    as_ = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(nh, nh.getNamespace(), &goalCb, false);
    as_->start();

	ROS_INFO("Action server is active and spinning...");

	ros::spin();

	return 0;
}

