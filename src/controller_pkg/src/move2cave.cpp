#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define PI M_PI

#define TFOUTPUT 1
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move2cave");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif
    
    tf::Vector3 origin(-38,10,6.5);
    tf::Vector3 target(-320, 10, 14.5);
    bool rotation_started = false;
    ros::Time rotation_start_time;
    double upTime = 5;

// loop start
    while (ros::ok()) {
        double t = (ros::Time::now() - start).toSec();
         // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

        tf::Quaternion q;
        if (t <= upTime) {

            tf::Vector3 upDisplacement(0.0, 0.0, 2);
            desired_pose.setOrigin(origin + upDisplacement);
            q.setRPY(0, 0, PI); 
        } else if (!rotation_started) {
            
            double elapsedTimeSinceUp = t - upTime;
            if (elapsedTimeSinceUp <= 60) {
                tf::Vector3 stepDisplacement = (target - origin - tf::Vector3(0.0, 0.0, 2)) * (elapsedTimeSinceUp / 60.0);
                desired_pose.setOrigin(origin + stepDisplacement + tf::Vector3(0.0, 0.0, 2));

                // calculate orientation
                tf::Vector3 flight_direction = target - origin;
                flight_direction.normalize();
                double yaw = atan2(flight_direction.y(), flight_direction.x());
                tf::Quaternion q;
                q.setRPY(0, 0, PI+yaw); 
                // q.setRPY(0, 0, PI/2); 
            } else {
                desired_pose.setOrigin(target);
                rotation_started = true;
                rotation_start_time = ros::Time::now();
            }

        } else {
            
            double rotation_time = (ros::Time::now() - rotation_start_time).toSec();
            double angle = (rotation_time / 10) * 4 * PI; 
            q.setRPY(0, 0, PI+angle);
            desired_pose.setOrigin(target);
            if (rotation_time >= 10) {
                
                ROS_INFO("Rotation complete. Node will shutdown.");
                ros::shutdown();
                break;
            }
        }

        // No rotation - keep the orientation constant
        desired_pose.setRotation(q);

        // Acceleration is considered to be zero for simplicity
        acceleration.linear.x = 0;
        acceleration.linear.y = 0;
        acceleration.linear.z = 0;


        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = q.getX();
        msg.transforms[0].rotation.y = q.getY();
        msg.transforms[0].rotation.z = q.getZ();
        msg.transforms[0].rotation.w = q.getW();
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        std::stringstream ss;
        ss << "Trajectory Position"
           << " x:" << desired_pose.getOrigin().x()
           << " y:" << desired_pose.getOrigin().y()
           << " z:" << desired_pose.getOrigin().z();
        ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
#endif

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
