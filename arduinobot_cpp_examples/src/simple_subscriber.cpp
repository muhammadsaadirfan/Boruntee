#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

namespace simple_subscriber_ns {

  // Subscriber callback function
  void msgCallback(const std_msgs::msg::String &msg)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("simple_subscriber"), "I heard: " << msg.data.c_str());
  }

  // Function to initialize the subscriber
  void startSubscriber()
  {
    auto node = rclcpp::Node::make_shared("simple_subscriber");
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&msgCallback, std::placeholders::_1));

    rclcpp::spin(node);
  }

}  // namespace simple_subscriber_ns

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  simple_subscriber_ns::startSubscriber();
  rclcpp::shutdown();
  return 0;
}
