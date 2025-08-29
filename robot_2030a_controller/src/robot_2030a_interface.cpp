#include "robot_2030a_controller/robot_2030a_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iomanip>  
#include <sstream> 
#include <cmath>
#include <std_msgs/msg/string.hpp>




namespace robot_2030a_controller
{

  double degreesToRadians(double degrees) {
  const double pi = 3.14159265358979323846; // Value of π
  return degrees * (pi / 180.0);
  }


  
Robot_2030aInterface::Robot_2030aInterface() 
{
    node_ = std::make_shared<rclcpp::Node>("robot_2030a_interface");
}

Robot_2030aInterface::Robot_2030aInterface(const rclcpp::Node::SharedPtr &node)
    : node_(node)
{
}

Robot_2030aInterface::~Robot_2030aInterface()
{
}


CallbackReturn Robot_2030aInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    robot_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("robot_cmd_pub_", rclcpp::QoS(10));

    query_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/query_response", rclcpp::QoS(10), 
    std::bind(&Robot_2030aInterface::query_callback, this, std::placeholders::_1));

    position_commands_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size());
    prev_position_commands_.resize(info_.joints.size());
    start_time_ = node_->get_clock()->now();

    RCLCPP_INFO(node_->get_logger(), "Robot 2030A Interface initialized successfully!");
    return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> Robot_2030aInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> Robot_2030aInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn Robot_2030aInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("Robot_2030aInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  prev_position_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  
  

  RCLCPP_INFO(rclcpp::get_logger("Robot_2030aInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn Robot_2030aInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("Robot_2030aInterface"), "Stopping robot hardware ...");


  RCLCPP_INFO(rclcpp::get_logger("Robot_2030aInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type Robot_2030aInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  rclcpp::spin_some(node_);

  if ((query_subscriber_->get_publisher_count() == 0) &&  
    ((node_->get_clock()->now() - start_time_).seconds() > 30.0))
{
    if (!warning_printed_)
    {
        RCLCPP_WARN(node_->get_logger(), "\033[1;31m--------------------------- System is not Closeloop !!! ---------------------------\033[0m");
        warning_printed_ = true;
    }
    position_states_ = position_commands_;
    return hardware_interface::return_type::OK;
}
  return hardware_interface::return_type::OK;
}



hardware_interface::return_type Robot_2030aInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
    // Only send new commands if they differ from the previous ones
    // if (position_commands_ == prev_position_commands_)
    // {
    //     return hardware_interface::return_type::OK;
    // }

    auto int_msg = std_msgs::msg::Int32MultiArray();
    int_msg.data.clear();

    // Publish commands for each joint, converting from radians to degrees
    for (size_t i = 0; i < position_commands_.size(); ++i)
    {
        int angle = static_cast<int>((((position_commands_.at(i) - (M_PI / 2)) * 180) / M_PI) + 90);
        int rounded_angle = std::round(angle * 100.0f) / 100.0f;

        if (std::fabs(rounded_angle) < std::numeric_limits<int>::epsilon())
        {
            rounded_angle = 0.0f; 
        }

        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << rounded_angle;
        int formatted_angle = std::stof(stream.str());

        // Multiply the value by 1000
        formatted_angle *= 1000;

        int_msg.data.push_back(formatted_angle);
    }

    // Publish the ODrive command
    robot_command_publisher_->publish(int_msg);

    prev_position_commands_ = position_commands_;

   

    return hardware_interface::return_type::OK;
}

void Robot_2030aInterface::query_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg)
{
    if (msg->data.size() < 6)
    {
        RCLCPP_ERROR(node_->get_logger(), "Received message with insufficient data size.");
        return;
    }
    // RCLCPP_WARN(node_->get_logger(), "\033[1;31m--------------------------- Received query response message !!! ---------------------------\033[0m");

    for (size_t i = 0; i < 6; ++i)
    {
      // RCLCPP_INFO(node_->get_logger(), "Processing value %zu: %.2f degrees -> %.2f radians", i, msg->data.at(i), degreesToRadians(msg->data.at(i)));
      position_states_.at(i) = degreesToRadians(msg->data.at(i));
    }
}

}  

PLUGINLIB_EXPORT_CLASS(robot_2030a_controller::Robot_2030aInterface, hardware_interface::SystemInterface)

