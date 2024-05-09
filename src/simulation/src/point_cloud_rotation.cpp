// #include <pcl/conversions.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
// #include <sensor_msgs/PointCloud2.h>
// #include "pcl/common/eigen.h"
// #include "pcl/common/transforms.h"

// #include "ros/ros.h"

// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2_msgs/TFMessage.h>
// #include <tf/tf.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// class CloudPoint
// {
// private:
//     ros::Subscriber subscriber_nodes_;
//     ros::Subscriber subscriber_tf_;
//     ros::Publisher publisher_nodes_;

// public:
//     CloudPoint() : nh("~") // default setting
//     {
//         subscriber_nodes_ = nh.subscribe("/pointcloud", 5, &CloudPoint::nodes_callback, this);
//         subscriber_tf_ = nh.subscribe("/tf", 10, &CloudPoint::tf_callback, this);
//         publisher_nodes_ = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/rotation", 1);
//         ros::spin();
//     };
//     ~CloudPoint(){};
//     ros::NodeHandle nh;
//     Eigen::Matrix4f tf_Matrix;
//     Eigen::Matrix4f getTFMatrix(const tf2_msgs::TFMessage &tf_msg);
//     void nodes_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
//     void tf_callback(const tf2_msgs::TFMessage &tf_msg);
//     // void quaternion_callback()
// };

// Eigen::Matrix4f CloudPoint::getTFMatrix(const tf2_msgs::TFMessage &tf_msg)
// {
//     // initialize transformation matrix to identity matrix
//     Eigen::Matrix4f tf_Matrix = Eigen::Matrix4f::Identity();
//     if (tf_msg.transforms.size() > 0)
//     {
//         // get the first transform in the message
//         const auto &transform = tf_msg.transforms[0];

//         // convert quaternion to rotation matrix
//         tf2::Quaternion quaternion;
//         tf2::fromMsg(transform.transform.rotation, quaternion);
//         tf2::Matrix3x3 rotation_matrix(quaternion);

//         // set rotation part of transformation matrix
//         tf_Matrix << rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0], 0,
//             rotation_matrix[0][1], rotation_matrix[1][1], rotation_matrix[2][1], 0,
//             rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2], 0,
//             0, 0, 0, 1;

//         tf_Matrix << 1, 0, 0, 0,
//             0, 1, 0, 0,
//             0, 0, 1, 0,
//             0, 0, 0, 1;
//     }
//     return tf_Matrix;
// }

// void CloudPoint::nodes_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
// {
//     // Convert from sensor_msgs to pcl::PointCloud
//     // container for original ^ filtered data
//     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*in_cloud_ptr, *current_cloud_ptr);
//     pcl::transformPointCloud(*current_cloud_ptr, *current_cloud_ptr, tf_Matrix);

//     // perform the actual filtering
//     pcl::VoxelGrid<pcl::PointXYZ> vgd;
//     vgd.setInputCloud(current_cloud_ptr);
//     vgd.setLeafSize(0.2f, 0.2f, 0.2f);
//     vgd.filter(*filtered_cloud_ptr);

//     // Rotate the point cloud around X axis by 90 degrees
//     Eigen::Matrix4f rotationMatrix_x;
//     double rot_x = M_PI / 2.0; // Rotate by 90 degrees
//     rotationMatrix_x << 1.0, 0.0, 0.0, 0.0,
//                         0.0, cos(rot_x), -sin(rot_x), 0.0,
//                         0.0, sin(rot_x), cos(rot_x), 0.0,
//                         0.0, 0.0, 0.0, 1.0;

//     // Apply the rotation to the point cloud
//     pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::transformPointCloud(*filtered_cloud_ptr, *rotated_cloud_ptr, rotationMatrix_x);

//     // Convert to ROS msg
//     sensor_msgs::PointCloud2 pub_PointCloud;
//     pcl::toROSMsg(*rotated_cloud_ptr, pub_PointCloud);
//     pub_PointCloud.header = in_cloud_ptr->header;

//     // Publish the data
//     publisher_nodes_.publish(pub_PointCloud);
// }

// void CloudPoint::tf_callback(const tf2_msgs::TFMessage &tf_msg)
// {
//     tf_Matrix = getTFMatrix(tf_msg);
// }

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "cloud_voxel");
//     CloudPoint node;
//     // ros::spin();
// }