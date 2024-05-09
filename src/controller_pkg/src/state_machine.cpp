#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

class stateMachine
{

public:
    bool is_up = false, is_down = false;

    stateMachine(){
        sub_map_overhead_ = nh.subscribe("/projected_map_overhead", 5, &stateMachine::overhead_callback, this);
        sub_map_ground_ = nh.subscribe("/projected_map_ground", 5, &stateMachine::ground_callback, this);
        current_state = nh.subscribe("/current_state_est", 1, &stateMachine::onCurrentState, this);
        pub_up = nh.advertise<std_msgs::Bool>("/up", 1);
        pub_down = nh.advertise<std_msgs::Bool>("/down", 1);
        pub_walls_ = nh.advertise<nav_msgs::OccupancyGrid>("test", 1, true);
        timer = nh.createTimer(ros::Duration(0.1), &stateMachine::timerCallback, this);
        ros::Rate rate(1);  // Adjust the rate (in Hz) based on your requirements

        while (ros::ok()) {
            // Process callbacks
            ros::spinOnce();

            // Sleep to control the message receiving rate
            rate.sleep();
        }

    };
    ~stateMachine(){};

    // int x, y, z;
    double x, y, z;

    void overhead_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void ground_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void onCurrentState(const nav_msgs::Odometry& cur_state);
    void timerCallback(const ros::TimerEvent& event);


private:
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Subscriber sub_map_overhead_, sub_map_ground_, current_state;
    ros::Publisher pub_up, pub_down, pub_walls_;
};

void stateMachine::onCurrentState(const nav_msgs::Odometry& cur_state){
    x = cur_state.pose.pose.position.x;
    y = cur_state.pose.pose.position.y;
    // z = cur_state.pose.pose.position.z;

    // x = static_cast<int>(x);
    // y = static_cast<int>(y);

 
}

void stateMachine::overhead_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    // int map_index = (y - map_msg->info.origin.position.y) / map_msg->info.resolution  +
    //             (x - map_msg->info.origin.position.x) / map_msg->info.resolution * map_msg->info.height;
    // int cell_value = map_msg->data[map_index];
 
    // // ROS_INFO("x: %f, y: %f, index: %d", x, y, map_index);
    // // ROS_INFO("origin x: %f, origin y: %f", map_msg->info.origin.position.x, map_msg->info.origin.position.y);
    // bool is_up = true;
    // std_msgs::Bool bool_msg;
    // bool_msg.data = is_up;
    // pub_up.publish(bool_msg);
    is_up = false;
    for(size_t i = map_msg->data.size() - 5000; i < map_msg->data.size(); ++i)
    {

        if(map_msg->data[i] > 0)
        {
            is_up = true;
            
            
            break;
        }
    }


}



void stateMachine::ground_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    is_down = false;
    for(size_t i = map_msg->data.size() - 5000; i < map_msg->data.size(); ++i)
    {

        if(map_msg->data[i] > 0)
        {
            is_down = true;
            
            
            
            break;
        }
    }
    std_msgs::Bool bool_msg;
    bool_msg.data = is_down;
    pub_down.publish(bool_msg);
    

}

void stateMachine::timerCallback(const ros::TimerEvent& event){
    std_msgs::Bool bool_msg_up;
    bool_msg_up.data = is_up;
    pub_up.publish(bool_msg_up);

    std_msgs::Bool bool_msg_down;
    bool_msg_down.data = is_down;
    pub_up.publish(bool_msg_down);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "state_machine_node");
    stateMachine node;
    ros::spin();
    return 0;
}