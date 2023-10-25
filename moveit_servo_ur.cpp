#include "moveit_servo_ur.hpp"
#include <memory>

#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


moveit_servo_ur::moveit_servo_ur() : Node("moveit_servo_ur")
{    /////////////////////// Nazar added Moveit part ///////////////////   
// Create a subscriber for the "bottle_pose" topic
    bottle_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "bottle_pos", 10, std::bind(&moveit_servo_ur::bottlePoseCallback, this, std::placeholders::_1));
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_marker", 10); // Create a marker publisher
    

    // Create the MoveIt MoveGroup Interface
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);

    std::string frame_id = move_group_interface->getPlanningFrame(); 
    // Generate a table collision object based on the lab task
      auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall");
      auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall");
      auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");
      auto col_object_ceiling = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, 1.0, frame_id, "ceiling");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Apply table as a collision object
      planning_scene_interface.applyCollisionObject(col_object_backWall);
      planning_scene_interface.applyCollisionObject(col_object_sideWall);
      planning_scene_interface.applyCollisionObject(col_object_table);
      planning_scene_interface.applyCollisionObject(col_object_ceiling);

    
    /////////////////////// Nazar added Moveit part Ends //////////////////////

    ////////////////////// This is from demo_moveit_servo /////////////
    client_servo_trigger_ = create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    client_switch_controller_ = create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    joint_cmd_pub_ = create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
    twist_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);
    
    timer_routine_ = this->create_wall_timer(std::chrono::milliseconds(34), std::bind(&moveit_servo_ur::routine_callback, this));
    timer_routine_->cancel();    
}

void moveit_servo_ur::request_activate_servo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_servo_trigger_->async_send_request(request, std::bind(&moveit_servo_ur::client_servo_response_callback, this, std::placeholders::_1));
}

void moveit_servo_ur::request_switch_controllers() {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers.push_back("forward_position_controller");
    request->deactivate_controllers.push_back("joint_trajectory_controller");
    request->strictness = 2;
    request->activate_asap = true;
    request->timeout.sec = 2.0;
    auto result = client_switch_controller_->async_send_request(request, std::bind(&moveit_servo_ur::client_switch_controller_response_callback, this, std::placeholders::_1));
}

void moveit_servo_ur::client_servo_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
    if (future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Servo Service call succeeded");
        actived_servo_flag = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Servo Service call failed");
    }
}

void moveit_servo_ur::client_switch_controller_response_callback(rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) 
{
    if (future.get()->ok) {
        RCLCPP_INFO(this->get_logger(), "Switch controller Service call succeeded");
        switched_controllers_flag = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Switch controller Service call failed");
    }
}



void moveit_servo_ur::routine_callback()
{   
    if ( switched_controllers_flag && actived_servo_flag) {
        auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "tool0";
        msg->twist.linear.x = 0.1;
        msg->twist.linear.y = 0.0;
        msg->twist.angular.z = 0.0;
        twist_cmd_pub_->publish(std::move(msg));
        count++;
        if (count >= 200) {
            msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            msg->header.stamp = this->now();
            msg->header.frame_id = "tool0";
            msg->twist.linear.x = 0;
            msg->twist.linear.y = 0;
            msg->twist.angular.z = 0;
            twist_cmd_pub_->publish(std::move(msg));
            std::cout<<"finished moving to point 2" << std::endl;
            auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            request->deactivate_controllers.push_back("forward_position_controller");
            request->activate_controllers.push_back("joint_trajectory_controller");
            request->timeout.sec = 2.0;
            auto result = client_switch_controller_->async_send_request(request, std::bind(&moveit_servo_ur::client_switch_controller_response_callback, this, std::placeholders::_1));
            std::cout<<"finished switch back controller" << std::endl;
            rclcpp::shutdown();
            return;
        }
    }
}
void moveit_servo_ur::bottlePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (goal_reached) {
        // Stop processing new messages if the goal has been reached
        return;
    }
    geometry_msgs::msg::Pose tar_msg;
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI , M_PI);
    tar_msg.orientation.x = q.x();
    tar_msg.orientation.y = q.y();
    tar_msg.orientation.z = q.z();
    tar_msg.orientation.w = q.w();
    tar_msg.position.x = msg->pose.position.x;//0.3;
    tar_msg.position.y = msg->pose.position.y;//0.2;
    tar_msg.position.z = msg->pose.position.z;//0.3;
    auto const target_pose_1 = tar_msg;
    auto success = false;
    moveit::planning_interface::MoveGroupInterface::Plan planMessage;     

    //Plan movement to point 1
    move_group_interface->setPoseTarget(target_pose_1);
    success = static_cast<bool>(move_group_interface->plan(planMessage));

    //Execute movement to point 1
    if (success) {
    move_group_interface->execute(planMessage);    
    } else {
    std::cout<<"fail moving to point 1" << std::endl;
    return;
    }
    
    std::cout<<"finished moving to point 1" << std::endl;
    goal_reached = true;
    wait_for_services();    
    request_switch_controllers();    
    request_activate_servo();    
    
    timer_routine_->reset();
    return;       
    }



void moveit_servo_ur::wait_for_services(){
    while (!client_servo_trigger_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!client_switch_controller_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<moveit_servo_ur>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
