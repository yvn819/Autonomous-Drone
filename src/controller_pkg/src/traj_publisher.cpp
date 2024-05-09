#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define STATIC_POSE 0
#define STATIC 0
#define CIRCLE 1
#define CAVE 2
#define SHANXIAN 3
#define PI M_PI

#define TFOUTPUT 1
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    while (ros::ok()) {
        tf::Vector3 origin(-38,10,6.5);

        double t = (ros::Time::now()-start).toSec();

        // Quantities to fill in
        tf::Transform desired_pose(tf::Transform::getIdentity());

        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;

        tf::Quaternion q;
        int state = 3;
        switch(state){
                case STATIC:{
                       tf::Vector3 displacement(0,0,2);
                       desired_pose.setOrigin(origin+displacement);
                       q.setRPY(0,0,PI/2);
                       count++;
                       std::cout<<"Desired Orientation" << count << std::endl;
                       desired_pose.setRotation(q);
                        if (t >= 5) {
                                ROS_INFO("static complete. Node will shutdown.");
                                ros::shutdown();
                                break;
                        }
                }

                case CIRCLE:{
                        double R = 3.0;
                        double timeScale = 5.0;
                        desired_pose.setOrigin(
                                        origin + tf::Vector3(R*sin(t/timeScale), R*cos(t/timeScale), 2.0)
                                );
                        q.setRPY(0,0,-t/timeScale);
                        desired_pose.setRotation(q);
                        velocity.linear.x = R*cos(t/timeScale)/timeScale;
                        velocity.linear.y = -R*sin(t/timeScale)/timeScale;
                        velocity.linear.z = 0;

                        velocity.angular.x = 0.0;
                        velocity.angular.y = 0.0;
                        velocity.angular.z = -1.0/timeScale;

                        acceleration.linear.x = -R*sin(t/timeScale)/timeScale/timeScale;
                        acceleration.linear.y = -R*cos(t/timeScale)/timeScale/timeScale;
                        acceleration.linear.z = 0;    
                        break;
                }

                case CAVE: {
                        // Set the initial upward movement parameters
                        double upwardSpeed = 0.5; // m/s
                        double upTime = 5; // s for upward movement
                        double fixedTime = 30;

                        // Define the target and calculate the displacement for the complex movement
                        tf::Vector3 target(-800, -125, 0);
                        tf::Vector3 initialDisplacement = target - origin;

                        // Time since starting the movement towards the target
                        double elapsedTimeSinceUp = t - upTime;

                        if (t <= upTime) {
                                // Upward movement for the first upTime seconds
                                desired_pose.setOrigin(origin + tf::Vector3(0.0, 0.0, upwardSpeed * t));
                                velocity.linear.z = upwardSpeed; // Upward speed
                                velocity.linear.x = 0;
                                velocity.linear.y = 0;
                        } else {
                                // Calculate total displacement needed at this point in time
                                if (elapsedTimeSinceUp <= fixedTime) {
                                // Calculate step displacement based on total displacement needed and the elapsed time
                                tf::Vector3 stepDisplacement = initialDisplacement * (elapsedTimeSinceUp / fixedTime);
                                desired_pose.setOrigin(origin + stepDisplacement);

                                // Update velocity towards the target
                                velocity.linear.x = initialDisplacement.x() / fixedTime;
                                velocity.linear.y = initialDisplacement.y() / fixedTime;
                                velocity.linear.z = initialDisplacement.z() / fixedTime;
                                } else {
                                // Stop movement by setting velocity to zero after reaching the target
                                double R = 3.0;
                                double timeScale = 5.0;
                                desired_pose.setOrigin(
                                                origin + tf::Vector3(R*sin(t/timeScale), R*cos(t/timeScale), 2.0)
                                        );
                                q.setRPY(0,0,-t/timeScale);
                                desired_pose.setRotation(q);
                                velocity.linear.x = R*cos(t/timeScale)/timeScale;
                                velocity.linear.y = -R*sin(t/timeScale)/timeScale;
                                velocity.linear.z = 0;

                                velocity.angular.x = 0.0;
                                velocity.angular.y = 0.0;
                                velocity.angular.z = -1.0/timeScale;

                                acceleration.linear.x = -R*sin(t/timeScale)/timeScale/timeScale;
                                acceleration.linear.y = -R*cos(t/timeScale)/timeScale/timeScale;
                                acceleration.linear.z = 0;    
                                break;
                                }
                        }

                        // No rotation - keep the orientation constant
                        q.setRPY(0, 0, 0);
                        desired_pose.setRotation(q);

                        // Acceleration is considered to be zero for simplicity
                        acceleration.linear.x = 0;
                        acceleration.linear.y = 0;
                        acceleration.linear.z = 0;

                        break;
                }

                case SHANXIAN:{
                        tf::Vector3 cave(-310, 10, 14.5);
                        desired_pose.setOrigin(cave);
                        q.setRPY(0,0,PI);
                        count++;
                        std::cout<<"Desired Orientation" << count << std::endl;
                        desired_pose.setRotation(q);
                        if (t >= 5) {
                                ROS_INFO("shanxian complete. Node will shutdown.");
                                ros::shutdown();
                                break;
                        }
                }
                    
                // Other cases could be defined here
        }



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
        ++count;
    }


    return 0;
}
