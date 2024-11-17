#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber"), count_(0), new_message_(std::make_shared<std_msgs::msg::String>())
  {
    rclcpp::QoS qos_settings(rclcpp::KeepLast(10));
    qos_settings.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos_settings.history(rclcpp::HistoryPolicy::KeepAll);
    qos_settings.reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "ping", qos_settings, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("pong", qos_settings);
  }

private:
  void topic_callback(const std_msgs::msg::String & msg)
  {
    std::string received_message = msg.data;
    count_++;

    if (count_ == 100) {
      publisher_->publish(*new_message_);
      printf("Received 100 messages\n");
      count_ = 0;
    } else {
      new_message_->data = msg.data;
      publisher_->publish(*new_message_);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::shared_ptr<std_msgs::msg::String> new_message_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

