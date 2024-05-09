#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <math.h>

typedef Eigen::Vector3d Position;

class lanternDetector{
public:
    bool camera_info_received = false; 
    bool target_detected = false;     // target detection

    sensor_msgs::CameraInfo camera_info;
    Eigen::Matrix3d K; // camera's internal reference matrix K
    std::unordered_map<std::string, Position> lantern_info;

    // thresholds
    const double Pixels_THRESHOLD = 100.0;
    const double DISTANCE_THRESHOLD = 50;

    cv::Mat mask;
    cv::Mat masked_depth;

    tf::TransformListener tf_listener;
    Eigen::Affine3d transform_eigen;

    Eigen::Vector3d newPosition;
    lanternDetector() : nh_() {
        camera_info_sub = nh_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/camera_info", 1, &lanternDetector::cameraInfoCallback, this);
        sub = nh_.subscribe("/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1, &lanternDetector::semCallback, this);
        depth_sub = nh_.subscribe("/realsense/depth/image", 1, &lanternDetector::depthCallback, this);
    }

    void semCallback(const sensor_msgs::ImageConstPtr& msg){
        try
        {
            // Convert the ROS image message to an OpenCV image
            cv::Mat semantic_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            // Detect the lantern in the semantic image

            cv::inRange(semantic_image, cv::Scalar(0, 200, 200), cv::Scalar(10, 255, 255), mask); // get this value from the average of non-zero pixels of the image
            // Check if the detected lantern is above a size threshold
            int num_pixels = cv::countNonZero(mask);
            // ROS_INFO("pixels number %d", num_pixels);
            // put this mask on the depth image
            
            if (num_pixels > Pixels_THRESHOLD) // the object is big enough to find it
            {
                target_detected = true;
                
            }
            else
            {
                target_detected = false;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }


    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
        if (!camera_info_received) {
            double fx = info_msg->K[0];
            double fy = info_msg->K[4];
            double cx = info_msg->K[2];
            double cy = info_msg->K[5];
            K << fx, 0, cx,
                0, fy, cy,
                0, 0, 1;
            ROS_INFO("Received camera info, K matrix set.");
            camera_info_received = true; 
        }
    }

    void checkAndAddPositionToMap(const Position& newPosition, std::unordered_map<std::string, Position>& lantern_info) {
        for (const auto& item : lantern_info) {
            if ((item.second - newPosition).norm() < DISTANCE_THRESHOLD) {
                return; // Position is close to an existing one, do not add
            }
        }
        // New position is not close to any existing one, add it with a unique identifier
        std::string newId = "lantern " + std::to_string(lantern_info.size() + 1);
        lantern_info[newId] = newPosition;
    }

    void printLanternPositions(const std::unordered_map<std::string, Position>& lantern_info) {
        for (const auto& item : lantern_info) {
            ROS_INFO("%s: Position: (%f, %f, %f)", item.first.c_str(), item.second.x(), item.second.y(), item.second.z());
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!target_detected)
            return; 

        // Convert the ROS depth image message to an OpenCV image
        cv::Mat depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        // Apply the mask to the depth image

        depth_image.copyTo(masked_depth, mask);

        // step 1: Get pixel coordinate of the image
        
        Eigen::Matrix<double,3,Eigen::Dynamic> position_pixel_disturb;
        Eigen::Matrix<double,1,Eigen::Dynamic> pixel_depth;

        int sum_mask = cv::sum(mask)[0];
        int N_mask = (int)(sum_mask/255);

        position_pixel_disturb.resize(3, N_mask);
        pixel_depth.resize(1, N_mask);
        int n = 0; 
        // pixel_coordinate origin on the top left
        for(int i = 0; i < mask.rows; i++){
            for(int j = 0; j < mask.cols; j++){
                if(mask.at<uint8_t>(i,j) == 255){
                    position_pixel_disturb(0,n) = j;
                    position_pixel_disturb(1,n) = i;
                    position_pixel_disturb(2,n) = 1;
                    pixel_depth(0,n) = depth_image.at<uint16_t>(i,j);
                    n++;
                }
            }
        }
        
        // step 2: from disturbed pixel to camera-coordinate
        Eigen::Matrix<double,3,Eigen::Dynamic> position_camera;
        position_camera.resize(3,N_mask);
        for(int i=0; i<N_mask; i++){
            position_camera(0,i) = pixel_depth(0,i) / 1000;
            position_camera(1,i) = - ((double)position_pixel_disturb(0,i) - K(0,2)) * position_camera(0,i) / K(0,0);
            position_camera(2,i) = - ((double)position_pixel_disturb(1,i) - K(1,2)) * position_camera(0,i) / K(1,1);
        }  // not a normal camera

        // step 3: coordinate transform from camera to drone and world
        Eigen::Vector3d position_camera_mean = position_camera.rowwise().mean();
        ROS_INFO("position_camera %f, %f, %f", position_camera_mean[0], position_camera_mean[1], position_camera_mean[2]);
        tf::StampedTransform transform;
        try {
            tf_listener.lookupTransform("world", "Quadrotor/DepthCamera", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        tf::transformTFToEigen(transform, transform_eigen); // Convert to Eigen
        Eigen::Vector3d newPosition = transform_eigen * position_camera_mean;
        // Add the results into the dictionary
        ROS_INFO("Position: %f, %f, %f",newPosition[0], newPosition[1], newPosition[2]);
        // Eigen::Vector3d newPosition(1.0, 2.0, 3.0); 
        checkAndAddPositionToMap(newPosition, lantern_info);
        printLanternPositions(lantern_info);

    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_info_sub, sub, depth_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lantern_detector");
    lanternDetector node;
    ros::spin();
    return 0;
}