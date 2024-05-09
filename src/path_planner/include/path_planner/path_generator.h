#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "../include/path_planner/a_star.hpp"

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>



class PathGenerator
{
public:
    PathGenerator();
    ~PathGenerator();

private:
    void subscribeAndPublish();
    void onCurrentState(const nav_msgs::Odometry& cur_state);
    void gridMapHandler(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
    void navGoalHandler(const geometry_msgs::PoseStamped &goal_msg);

private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_grid_map_;
    ros::Subscriber sub_nav_goal_;
    ros::Subscriber current_state_;
    ros::Publisher  pub_robot_path_;
    ros::Publisher  pub_walls_;

    AStar::Generator map_generator_;
    nav_msgs::MapMetaData map_info_;
    bool map_exsit_;
    double uav_x, uav_y, uav_z;
};

#endif //PATH_GENERATOR_H
