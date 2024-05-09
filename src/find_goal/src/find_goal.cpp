#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <cmath>

#include <nav_msgs/Odometry.h>


class GlobalPlannerNode {
public:
    double cur_height;
    Eigen::Quaterniond q;

    GlobalPlannerNode() {

        map_sub_ = nh_.subscribe("/projected_map_filter", 1, &GlobalPlannerNode::mapCallback, this);
        current_state_ = nh_.subscribe("current_state_est", 1, &GlobalPlannerNode::onCurrentState, this);

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    }

    void onCurrentState(const nav_msgs::Odometry& cur_state){
        cur_height = cur_state.pose.pose.position.z;
        tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
 
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        // Convert map coordinates to world coordinates
        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("/Quadrotor/TrueState", map_msg->header.frame_id, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        geometry_msgs::PoseStamped farthest_pose;
        farthest_pose.header.frame_id = "world";
        farthest_pose.header.stamp = ros::Time::now();

        geometry_msgs::PointStamped point;

        double max_distance = 0;
        int farthest_index = -1;

        // Loop through the occupancy grid to find farthest point
        for (size_t i = 0; i < map_msg->data.size(); ++i) {
            if (map_msg->data[i] == 0) { // Free space
                geometry_msgs::Point point;
                point.x = map_msg->info.origin.position.x + (i % map_msg->info.width) * map_msg->info.resolution;
                point.y = map_msg->info.origin.position.y + (i / map_msg->info.width) * map_msg->info.resolution;
                point.z = 0; // Assuming 2D map

                // tf::Point tf_point(point.x, point.y, point.z);
                // tf_point = transform * tf_point;

                // double distance = std::sqrt(tf_point.x() * tf_point.x() + tf_point.y() * tf_point.y());
                double distance = std::sqrt(point.x * point.x + point.y * point.y);
                if (distance > max_distance) {
                    max_distance = distance;
                    // farthest_pose.pose.position.x = static_cast<double>(tf_point.x());
                    // farthest_pose.pose.position.y = static_cast<double>(tf_point.y());
                    farthest_pose.pose.position.x = point.x;
                    farthest_pose.pose.position.y = point.y;                    
                    farthest_pose.pose.position.z = cur_height; 
                    // tf::quaternionEigenToMsg(q, farthest_pose.pose.orientation);
                    farthest_pose.pose.orientation.x = 0.0;
                    farthest_pose.pose.orientation.y = 0.0;
                    farthest_pose.pose.orientation.z = 0.0;
                    farthest_pose.pose.orientation.w = 1.0;
                    farthest_index = i;
                }
            }
        }

        // Publish the farthest point
        if (farthest_index != -1) {
            goal_pub_.publish(farthest_pose);
        }
    }



    private:
        ros::NodeHandle nh_;
        ros::Subscriber map_sub_, current_state_;
        ros::Publisher goal_pub_;
        tf::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner_node");
    GlobalPlannerNode node;
    ros::spin();
    return 0;
}