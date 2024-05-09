#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

#include "ros/ros.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

class CloudPoint
{
private:
    ros::Subscriber subscriber_nodes_;
    ros::Subscriber subscriber_tf_;
    ros::Subscriber current_state;
    ros::Subscriber subscriber_octomap_;
    ros::Publisher publisher_nodes_, publisher_nodes_OnItsHeight, publisher_nodes_ground, publisher_nodes_overhead;

public:
    CloudPoint() : nh("~") 
    {
        subscriber_nodes_ = nh.subscribe("/points", 5, &CloudPoint::nodes_callback, this);
        subscriber_tf_ = nh.subscribe("/tf", 10, &CloudPoint::tf_callback, this);
        current_state = nh.subscribe("current_state_est", 1, &CloudPoint::onCurrentState, this);
        // subscriber_octomap_ = nh.subscribe("/octomap_full", 10, &CloudPoint::octomap_callback, this);
        publisher_nodes_OnItsHeight = nh.advertise<sensor_msgs::PointCloud2>("/points/filteredOnItsHeight", 1);
        publisher_nodes_ = nh.advertise<sensor_msgs::PointCloud2>("/points/filtered", 1);
        publisher_nodes_ground = nh.advertise<sensor_msgs::PointCloud2>("/points/filteredGround", 1);
        publisher_nodes_overhead = nh.advertise<sensor_msgs::PointCloud2>("/points/filteredOverhead", 1);

        ros::spin();
    };
    ~CloudPoint(){};
    ros::NodeHandle nh;
    Eigen::Matrix4f tf_Matrix;
    double x, y, cur_height;
    Eigen::Matrix4f getTFMatrix(const tf2_msgs::TFMessage &tf_msg);
    void nodes_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
    void tf_callback(const tf2_msgs::TFMessage &tf_msg);
    void onCurrentState(const nav_msgs::Odometry& cur_state);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterOverhead(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr, double cur_height);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr, double cur_height);
    // void octomap_callback(const octomap_msgs::Octomap::ConstPtr& octomap_msg);
    double groundLevelHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr);
    // void quaternion_callback()
};

Eigen::Matrix4f CloudPoint::getTFMatrix(const tf2_msgs::TFMessage &tf_msg)
{
    // initialize transformation matrix to identity matrix
    Eigen::Matrix4f tf_Matrix = Eigen::Matrix4f::Identity();
    if (tf_msg.transforms.size() > 0)
    {
        // get the first transform in the message
        const auto &transform = tf_msg.transforms[0];

        // convert quaternion to rotation matrix
        tf2::Quaternion quaternion;
        tf2::fromMsg(transform.transform.rotation, quaternion);
        tf2::Matrix3x3 rotation_matrix(quaternion);

        // set rotation part of transformation matrix
        tf_Matrix << rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0], 0,
            rotation_matrix[0][1], rotation_matrix[1][1], rotation_matrix[2][1], 0,
            rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2], 0,
            0, 0, 0, 1;

        tf_Matrix << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    }
    return tf_Matrix;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr, double cur_height) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // double cur_height = cur_height;
    for (const pcl::PointXYZ& point : *transform_cloud_ptr) {
        if (point.z >= cur_height - 0.01 && point.z <= cur_height + 0.01) {
            filtered_cloud_ptr->points.push_back(point);
        }
    }

    filtered_cloud_ptr->header = transform_cloud_ptr->header;
    filtered_cloud_ptr->width = filtered_cloud_ptr->points.size();
    filtered_cloud_ptr->height = 1;

    return filtered_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPoint::filterOverhead(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr, double cur_height) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // double cur_height = cur_height;

    for (const pcl::PointXYZ& point : *transform_cloud_ptr) {
        if (point.z >= cur_height && point.z <= cur_height + 5 && point.y >= y-0.1 
        && point.y <= y+0.1 && point.x >= x+1.1 && point.x+5 ) {
            filtered_cloud_ptr->points.push_back(point);
        }
    }

    filtered_cloud_ptr->header = transform_cloud_ptr->header;
    filtered_cloud_ptr->width = filtered_cloud_ptr->points.size();
    filtered_cloud_ptr->height = 1;

    return filtered_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPoint::filterGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr, double cur_height) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // double cur_height = cur_height;

    for (const pcl::PointXYZ& point : *transform_cloud_ptr) {
        if (point.z >= cur_height - 5 && point.z <= cur_height && point.y 
        && point.y &&  point.x >= x+1.1 && point.x <= x+5 ) {
            filtered_cloud_ptr->points.push_back(point);
        }
    }

    filtered_cloud_ptr->header = transform_cloud_ptr->header;
    filtered_cloud_ptr->width = filtered_cloud_ptr->points.size();
    filtered_cloud_ptr->height = 1;

    return filtered_cloud_ptr;
}




void CloudPoint::onCurrentState(const nav_msgs::Odometry& cur_state){
    x = cur_state.pose.pose.position.x;
    y = cur_state.pose.pose.position.y;
    double cur_height = cur_state.pose.pose.position.z;
 
}

double CloudPoint::groundLevelHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transform_cloud_ptr){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setInputCloud(transform_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    double height = fabs(a * x + b * y + c * cur_height + d) / sqrt(a*a + b*b + c*c);
    ROS_INFO("ground level height: %f", height);
    return height;

}

void CloudPoint::nodes_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // Convert from sensor_msgs to pcl::PointCloud
    // container for original ^ filtered data
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr overhead_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in_cloud_ptr, *current_cloud_ptr);
    pcl::transformPointCloud(*current_cloud_ptr, *current_cloud_ptr, tf_Matrix);

    // perform the actual filtering
    // pcl::VoxelGrid<pcl::PointXYZ> vgd;
    // vgd.setInputCloud(current_cloud_ptr);
    // vgd.setLeafSize(0.2f, 0.2f, 0.2f);
    // vgd.setLeafSize(0.1, 0.1, 0.1);
    // vgd.filter(*filtered_cloud_ptr);
    // set downsampling
    // vgd.getDownsampleAllData(true);

    // Rotating the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    double rot_x = 0.0;
    double rot_y = M_PI / 2.0;
    double rot_z = -M_PI / 2.0;
    Eigen::Matrix4f rotMatrix_x;
    Eigen::Matrix4f rotMatrix_y;
    Eigen::Matrix4f rotMatrix_z;

    rotMatrix_x << 1.0, 0.0, 0.0, 0.0,
        0.0, cos(rot_x), -sin(rot_x), 0.0,
        0.0, sin(rot_x), cos(rot_x), 0.0,
        0.0, 0.0, 0.0, 1.0;

    rotMatrix_y << cos(rot_y), 0.0, sin(rot_y), 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sin(rot_y), 0.0, cos(rot_y), 0.0,
        0.0, 0.0, 0.0, 1.0;

    rotMatrix_z << cos(rot_z), -sin(rot_z), 0.0, 0.0,
        sin(rot_z), cos(rot_z), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    // convert to ROS msg
    sensor_msgs::PointCloud2 pub_PointCloudOnItsHeight;
    sensor_msgs::PointCloud2 pub_PointCloudground;
    sensor_msgs::PointCloud2 pub_PointCloudoverhead;
    sensor_msgs::PointCloud2 pub_PointCloud;

    // pcl::transformPointCloud(*filtered_cloud_ptr, *transform_cloud_ptr, rotMatrix_x * rotMatrix_y * rotMatrix_z);
    pcl::transformPointCloud(*current_cloud_ptr, *transform_cloud_ptr, rotMatrix_x * rotMatrix_y * rotMatrix_z);

    filtered_cloud_ptr = filterPointCloud(transform_cloud_ptr, cur_height);
    ground_ptr = filterGround(transform_cloud_ptr, cur_height);
    overhead_ptr = filterOverhead(transform_cloud_ptr, cur_height);

    // get the current height to the ground
    // double groud_level_height = groundLevelHeight(transform_cloud_ptr);

    pcl::toROSMsg(*transform_cloud_ptr, pub_PointCloud);
    pcl::toROSMsg(*filtered_cloud_ptr, pub_PointCloudOnItsHeight);
    pcl::toROSMsg(*ground_ptr, pub_PointCloudground);
    pcl::toROSMsg(*overhead_ptr, pub_PointCloudoverhead);


    pub_PointCloudOnItsHeight.header = in_cloud_ptr->header;
    pub_PointCloud.header = in_cloud_ptr->header;
    pub_PointCloudground.header = in_cloud_ptr->header;
    pub_PointCloudoverhead.header = in_cloud_ptr->header;

    // publish the data
    publisher_nodes_OnItsHeight.publish(pub_PointCloudOnItsHeight);
    publisher_nodes_ground.publish(pub_PointCloudground);
    publisher_nodes_overhead.publish(pub_PointCloudoverhead);
    publisher_nodes_.publish(pub_PointCloud);
}

void CloudPoint::tf_callback(const tf2_msgs::TFMessage &tf_msg)
{
    tf_Matrix = getTFMatrix(tf_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cloud_voxel");
    CloudPoint node;
    ros::spin();
    return 0;
}