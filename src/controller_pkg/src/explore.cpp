#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <ros/timer.h>
#include <std_msgs/Bool.h>

#define TFOUTPUT 1
#define PI M_PI


enum State {
    INITIAL,
    UPDOWN,
    ROTATING,
    FOLLOWPATH
};

class exploreCave
{

    tf::Transform desired_pose;
    tf::Quaternion q;
    ros::Timer timer;
    ros::Timer timer_stateMachine;

    // current position
    tf::Vector3 current_position;
    Eigen::Vector3d euler_angles;

    // path points
    std::vector<geometry_msgs::PoseStamped> path_points;
    double path_x, path_y;

    // initial height
    float height = 14.5;

    // up, down, rotate
    bool down_flag = false;
    bool up_flag = false;
    bool rotate_flag = false;
    ros::Time rotate_start_time;
    tf::Vector3 rotate_position;

public:

 

    exploreCave(){
        state_sub = nh.subscribe("current_state_est", 1, &exploreCave::onCurrentState, this);
        up_sub = nh.subscribe("/up", 1, &exploreCave::upCallback, this);
        down_sub = nh.subscribe("/down", 1, &exploreCave::downCallback, this);
        target_sub = nh.subscribe<nav_msgs::Path>("robot_path", 1, &exploreCave::pathCallback, this);
        desired_state_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);

        timer = nh.createTimer(ros::Duration(10.0), &exploreCave::timerCallback, this);
        desired_pose = tf::Transform::getIdentity();
        timer_stateMachine = nh.createTimer(ros::Duration(0.1), &exploreCave::controlLoop_, this);

    };
    ~exploreCave(){};


    void onCurrentState(const nav_msgs::Odometry& cur_state) {
        // ROS_INFO("onCurrentState");
        current_position.setX(cur_state.pose.pose.position.x);
        current_position.setY(cur_state.pose.pose.position.y);
        current_position.setZ(cur_state.pose.pose.position.z);

        Eigen::Quaterniond q_current(cur_state.pose.pose.orientation.w, 
                                        cur_state.pose.pose.orientation.x, 
                                        cur_state.pose.pose.orientation.y, 
                                        cur_state.pose.pose.orientation.z);
        euler_angles = q_current.toRotationMatrix().eulerAngles(0,1,2);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // ROS_INFO("path");
        if (!msg->poses.empty()) {
            path_points.clear();
            path_points  = msg->poses;
            path_x = path_points[3].pose.position.x;
            path_y = path_points[3].pose.position.y;
            // ROS_INFO("path point: %f, %f", path_points[0].pose.position.x, path_points[0].pose.position.y);
        }
        else ROS_INFO("not found");
    }

    void downCallback(const std_msgs::Bool::ConstPtr& msg) {
        down_flag = msg->data;
        // Process the received boolean data as needed
        if (down_flag) {
            ROS_INFO("Received a true value on /down topic, too low.");
            height ++;
            state = UPDOWN;
        } 
    }

    void upCallback(const std_msgs::Bool::ConstPtr& msg) {
        up_flag = msg->data;
        // Process the received boolean data as needed
        if (up_flag) {
            ROS_INFO("Received a true value on /up topic, too high.");
            height --;
            state = UPDOWN;
        }
    }

    void timerCallback(const ros::TimerEvent& event) {
        state = ROTATING;
        rotate_start_time = ros::Time::now(); // re-initialize the timer for rotation
        rotate_position = current_position;// remember the place of start rotation
    }


    void controlLoop_(const ros::TimerEvent& t)  {
        // ROS_INFO("in control loop");

        switch(state) {
            case INITIAL:{
                static ros::Time startTime = ros::Time::now(); // 记录开始时间
                ros::Duration delay(2.0);
                state = FOLLOWPATH;
            }
            case UPDOWN:{
                // ROS_INFO("UP/DOWN: %f", height - current_position.z());
                // ROS_INFO("path point: %f, %f", path_points[0].pose.position.x, path_points[0].pose.position.y);
                // position
                desired_pose.setOrigin(tf::Vector3(current_position.x(), current_position.y(), height));
                // angle
                double yaw = atan2(path_y - current_position.y(), path_x - current_position.x());
                q.setRPY(0, 0, yaw);
                desired_pose.setRotation(q);
                state = FOLLOWPATH;
                break;
            }    
            case ROTATING:{
                double rotation_time = (ros::Time::now() - rotate_start_time).toSec();
                ROS_INFO("Doing Rotation ... ...");

                double angle = (rotation_time / 5) * 2 * PI;
                q.setRPY(0, 0, euler_angles[0]+angle); // yaw is the 1st one
                desired_pose.setRotation(q);
                desired_pose.setOrigin(rotate_position);
                if (rotation_time >= 5) {
                    state = FOLLOWPATH;
                }
                break;
            }
            case FOLLOWPATH:{
                ROS_INFO("Current Position: (%f, %f, %f)",
                        current_position.x(),
                        current_position.y(),
                        current_position.z());
                ROS_INFO("Going to: (%f, %f, %f)",
                        path_x,
                        path_y,
                        height);
                // position
                desired_pose.setOrigin(tf::Vector3(path_x,
                                                    path_y,
                                                    height));
                // angle
                double yaw = atan2(path_y - current_position.y(), path_x - current_position.x());
                q.setRPY(0, 0, yaw);
                desired_pose.setRotation(q);
                break;
            }
        }
        // velocity initialization
        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


        tf::Vector3 displacement(path_x - current_position.x(), path_y - current_position.y(), height - current_position.z());
        displacement.normalize();
        velocity.linear.x = displacement.x()/2;
        velocity.linear.y = displacement.y()/2;
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

    }

private:
    ros::NodeHandle nh;
    ros::Rate loop_rate{1};
    State state = INITIAL;

    ros::Subscriber state_sub;
    ros::Subscriber up_sub;
    ros::Subscriber down_sub;
    ros::Subscriber target_sub;
    ros::Publisher desired_state_pub;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explore");
    exploreCave node;
    ros::spin();
    return 0;
}
