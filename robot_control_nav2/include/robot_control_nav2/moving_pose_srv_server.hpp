#ifndef MOVING_POSE_SRV_SERVER_HPP_
#define MOVING_POSE_SRV_SERVER_HPP_

#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <yaml-cpp/yaml.h>
#include "robot_msgs/srv/operator_two_ints.hpp"
#include "robot_msgs/srv/navigate_pose.hpp"
#include "std_msgs/msg/int32.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MovingPoseSrvServer: public rclcpp::Node
{
public: 
    MovingPoseSrvServer();

    using NavigateToPose_ = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose_= rclcpp_action::ClientGoalHandle<NavigateToPose_>;
private:

    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _response_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr next_point_res_pub;

    // Service_Server
    rclcpp::Service<robot_msgs::srv::OperatorTwoInts>::SharedPtr _server;
    rclcpp::Service<robot_msgs::srv::NavigatePose>::SharedPtr navigate_pose_server;
    
    rclcpp_action::Client<NavigateToPose_>::SharedPtr client_ptr_;
    
    // Service Client
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_cost_map_cl;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_cost_map_cl;
    
    // Subcriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_robot_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_sub;

    // TF Linsener 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Serviiec Seercer Callback
    void navigate_pose_server_cb(const robot_msgs::srv::NavigatePose::Request::SharedPtr req,
                                    const robot_msgs::srv::NavigatePose::Response::SharedPtr res);

    void operator_two_ints_callback(const robot_msgs::srv::OperatorTwoInts::Request::SharedPtr req,
                                    const robot_msgs::srv::OperatorTwoInts::Response::SharedPtr res);

    // Action Client CallBack_;
    void goal_response_callback(const GoalHandleNavigateToPose_::SharedPtr & goal_handle);

    void cancel_callback(const GoalHandleNavigateToPose_::SharedPtr & goal_handle);

    void feedback_callback(GoalHandleNavigateToPose_::SharedPtr,
                    const std::shared_ptr<const NavigateToPose_::Feedback> feedback);

    void result_callback(const GoalHandleNavigateToPose_::WrappedResult & result);
    
    // Sub - Callback 
    void curent_pose_robot_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void next_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    bool get_pose_robot();
    void robot_node_sleep(); 
    void timer_callback();
    void clear_all_costmap();
    void send_goal_pose();

    double quat_to_yaw(geometry_msgs::msg::PoseStamped pose);

    nav2_msgs::action::NavigateToPose::Goal goal_data;

    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped curent_pose_robot;
    geometry_msgs::msg::PoseStamped target_pose_robot;

    double curent_theta = 0.0;
    double target_theta = 0.0;

    float error_theta = 0.2;
    float pi_ = 3.14159265359;

    bool next_point_req_ = false;

    // Change the name ????????????????????
    int next_point_res = 0; // uint32 mode
                            // uint32 MODE_IDLE=0
                            // uint32 MODE_CHARGING=1
                            // uint32 MODE_MOVING=2
                            // uint32 MODE_PAUSED=3
                            // uint32 MODE_WAITING=4
                            // uint32 MODE_EMERGENCY=5
                            // uint32 MODE_GOING_HOME=6
                            // uint32 MODE_DOCKING=7

    std_msgs::msg::Int32 next_point_res_data;
    
    // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions()
    // YAML::Node config ;
};


#endif  







