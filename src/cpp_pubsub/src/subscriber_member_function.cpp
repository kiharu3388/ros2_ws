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
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "ping", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>("pong", 10);
  }

private:
  void topic_callback(const std_msgs::msg::String & msg)
  {
    std::string received_message = msg.data;
    count_ = std::stoi(received_message.substr(received_message.find_last_of(' ') + 1));

    if (count_ == 100) {
      printf("Received 100 messages\n");
      // exit(0);
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

