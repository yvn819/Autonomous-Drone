#include "../include/path_planner/path_generator.h"

#include <iostream>


PathGenerator::PathGenerator()
{
    subscribeAndPublish();
}

PathGenerator::~PathGenerator()
{

}

void PathGenerator::subscribeAndPublish()
{
    sub_grid_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/projected_map_filter", 1, &PathGenerator::gridMapHandler, this);
    sub_nav_goal_ = nh_.subscribe("move_base_simple/goal", 1, &PathGenerator::navGoalHandler, this);
    current_state_ = nh_.subscribe("current_state_est", 1, &PathGenerator::onCurrentState, this);
    pub_robot_path_ = nh_.advertise<nav_msgs::Path>("robot_path", 1, true);
    pub_walls_ = nh_.advertise<nav_msgs::OccupancyGrid>("walls", 1, true);
}

void PathGenerator::onCurrentState(const nav_msgs::Odometry& cur_state){
    uav_x = cur_state.pose.pose.position.x;
    uav_y = cur_state.pose.pose.position.y;
    uav_z = cur_state.pose.pose.position.z;
    // ROS_INFO("current state: %.2f, %.2f", uav_x, uav_y);

}

void PathGenerator::gridMapHandler(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    ROS_INFO("Generating map..");
    map_exsit_ = false;

    map_info_ = map_msg->info;

    // Generate Map, Options
    map_generator_.setWorldSize({(int)map_info_.width, (int)map_info_.height}); //{x, y}
    map_generator_.setHeuristic(AStar::Heuristic::manhattan);
    map_generator_.setDiagonalMovement(true);

    // Add Wall
    int x, y;
    for(size_t i = 0; i < map_msg->data.size(); ++i)
    {
        x = map_info_.origin.position.x + (i % map_info_.width) * map_info_.resolution;
        y = map_info_.origin.position.y + (i / map_info_.width) * map_info_.resolution;

        if(map_msg->data[i] == 100)
        {
            map_generator_.addCollision({x, y}, 0);
            // ROS_INFO("collision: %d, %d", x, y);
        }
    }
    ROS_INFO("Success build wall!");
    nav_msgs::OccupancyGrid wall_map;
    // nav_msgs::MapMetaData map_info_;
    std::vector<int8_t> map_data_;

    map_data_.resize(map_info_.width * map_info_.height, 0);
    for (const auto& coord : map_generator_.walls) {

        // ROS_INFO("map data: %d, %d", coord.x, coord.y);
        // int index = (coord.x - map_info_.origin.position.x) + (coord.y - map_info_.origin.position.y) 
        //             * map_info_.width;
        int offset_x = (coord.x - map_info_.origin.position.x) / map_info_.resolution;
        int offset_y = (coord.y - map_info_.origin.position.y) / map_info_.resolution;


        int index = offset_x + offset_y * map_info_.width;
        map_data_[index] = 100; 
    }
    nav_msgs::OccupancyGrid walls_msg;
    walls_msg.header.stamp = ros::Time::now();
    walls_msg.header.frame_id = "world"; 
    walls_msg.info = map_info_;
    walls_msg.data = map_data_;


    pub_walls_.publish(walls_msg);

    ROS_INFO("Success build map!");
    map_exsit_ = true;
}

void PathGenerator::navGoalHandler(const geometry_msgs::PoseStamped &goal_msg)
{
    if(!map_exsit_) return;

    ROS_INFO("\033[1;32mGoal received!\033[0m");

    // Round goal coordinate
    int goal_x = static_cast<int>(round(goal_msg.pose.position.x));
    int goal_y = static_cast<int>(round(goal_msg.pose.position.y));

    // Remmaping coordinate
    AStar::Vec2i target;
    // target.x = (goal_x) / map_info_.resolution;
    // target.y = (goal_y) / map_info_.resolution;
    target.x = goal_x;
    target.y = goal_y;

    AStar::Vec2i source;
    // source.x = (uav_x) / map_info_.resolution;
    // source.y = (uav_y) / map_info_.resolution;
    source.x = uav_x;
    source.y = uav_y;
    ROS_INFO("target:%f, %f, cur_state: %f, %f", target.x, target.y, source.x, source.y);

    // Find Path
    auto path = map_generator_.findPath(source, target);

    nav_msgs::Path path_msg;
    if(path.empty())
    {
        ROS_INFO("\033[1;31mFail generate path!\033[0m");
        return;
    }

    for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
    {
        geometry_msgs::PoseStamped point_pose;

        // Remmaping coordinate
        point_pose.pose.position.x = (coordinate->x) * map_info_.resolution;
        point_pose.pose.position.y = (coordinate->y) * map_info_.resolution;
        path_msg.poses.push_back(point_pose);
    }

    path_msg.header.frame_id = "world";
    pub_robot_path_.publish(path_msg);
   
    ROS_INFO("\033[1;36mSuccess generate path!\033[0m");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator");

    ROS_INFO("\033[1;32m----> Path Generator Node is Started.\033[0m");

    PathGenerator PG;

    ros::spin();
    return 0;
}
