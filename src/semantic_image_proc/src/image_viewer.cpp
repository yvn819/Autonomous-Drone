#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

int target_recognition = 0;
Eigen::Vector3d euler_angles;


void onCurrentState(const nav_msgs::Odometry& cur_state) {
    Eigen::Quaterniond q(cur_state.pose.pose.orientation.w, 
                         cur_state.pose.pose.orientation.x, 
                         cur_state.pose.pose.orientation.y, 
                         cur_state.pose.pose.orientation.z);
    // 从四元数计算欧拉角
    euler_angles = q.toRotationMatrix().eulerAngles(0,1,2);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        
        if (cv::countNonZero(gray_image) > 0) 
        {
            target_recognition = 1; 

            
            int rectWidth = gray_image.cols * 0.1;
            int rectHeight = gray_image.rows * 0.2;
            int rectX = (gray_image.cols - rectWidth) / 2;
            int rectY = (gray_image.rows - rectHeight) / 2;
            cv::Rect centerRect(rectX, rectY, rectWidth, rectHeight);

            
            cv::Mat centerRegion = gray_image(centerRect);
            if (cv::countNonZero(centerRegion) > 0) 
            {
                
                cv::Mat left_half = centerRegion(cv::Rect(0, 0, centerRegion.cols / 2, centerRegion.rows));
                cv::Mat right_half = centerRegion(cv::Rect(centerRegion.cols / 2, 0, centerRegion.cols / 2, centerRegion.rows));
                int left_non_zero = cv::countNonZero(left_half);
                int right_non_zero = cv::countNonZero(right_half);

                
                int diff = std::abs(left_non_zero - right_non_zero);

                
                if (diff < (centerRegion.cols * centerRegion.rows * 0.05)) 
                {
                    
                    ROS_INFO("World frame Euler angles: Yaw: %f", euler_angles[2]);
                }
            }

        }

        else
        {
            target_recognition = 0; 
        }

        
        cv::imshow("Image window", image);
        cv::waitKey(30); 

        if (target_recognition == 1)
        {
            ROS_INFO("Find Target!");
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_viewer");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe("current_state_est", 1, onCurrentState);

    ros::Subscriber sub = nh.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1, imageCallback);



    cv::namedWindow("Image window");

    ros::spin();

    cv::destroyWindow("Image window");
    return 0;
}
