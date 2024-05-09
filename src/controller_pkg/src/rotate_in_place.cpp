#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

#define PI M_PI

#define TFOUTPUT 1
tf::Vector3 current_position;
Eigen::Vector3d euler_angles;
bool position_initialized = false;

void onCurrentState(const nav_msgs::Odometry& cur_state) {
    if (!position_initialized) {
        
        current_position.setX(cur_state.pose.pose.position.x);
        current_position.setY(cur_state.pose.pose.position.y);
        current_position.setZ(cur_state.pose.pose.position.z);
        
        Eigen::Quaterniond q_current(cur_state.pose.pose.orientation.w, 
                                     cur_state.pose.pose.orientation.x, 
                                     cur_state.pose.pose.orientation.y, 
                                     cur_state.pose.pose.orientation.z);
        euler_angles = q_current.toRotationMatrix().eulerAngles(0,1,2);
        position_initialized = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_in_place");
    ros::NodeHandle n;
    

    ros::Subscriber state_sub = n.subscribe("current_state_est", 1, onCurrentState);

 
    ros::Rate rate(500); // 500Hz
    while (ros::ok() && !position_initialized) {
        ros::spinOnce();
        rate.sleep();
    }


    tf::Vector3 initial_position = current_position;
    Eigen::Vector3d initial_euler_angles = euler_angles;



    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    while (ros::ok()) {
        tf::Vector3 origin(initial_position);
        double rotation_time = (ros::Time::now()-start).toSec();
        double angle = (rotation_time / 10) * 2 * PI;;
        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = 0;
        velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

        tf::Quaternion q;

        desired_pose.setOrigin(origin);
        
        q.setRPY(0, 0, angle);
        desired_pose.setRotation(q);
   
        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Rotation angel"
           << " alpha:" << initial_euler_angles;
        ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif
        if (rotation_time >= 10) {

            ROS_INFO("Rotation complete. Node will shutdown.");
            ros::shutdown();
            break;
        }
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
