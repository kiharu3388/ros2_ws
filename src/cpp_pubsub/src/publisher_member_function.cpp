#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define GET_TIME(tp, err_msg) \
do { \
        if (clock_gettime(CLOCK_REALTIME, &tp)){ \
                perror(err_msg); \
                return; \
        } \
} while(0)

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0), message_(std::make_shared<std_msgs::msg::String>())
  {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    publisher_ = this->create_publisher<std_msgs::msg::String>("ping", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "pong", 10, std::bind(&MinimalPublisher::topic_callback, this, std::placeholders::_1));
    GET_TIME(start_time_, "Failed to get publish time");
    publish_message();
  }

private:
  void publish_message() {
    message_->data = "Hello, world! " + std::to_string(count_);
    publisher_->publish(*message_);
  }

  void topic_callback(const std_msgs::msg::String & )
  {
    if(count_ == 100) {
      GET_TIME(end_time_, "Failed to get subscribe time");
      double elapsed_time = ((double)end_time_.tv_sec + (double)end_time_.tv_nsec / (double)1000000000l)
                             - ((double)start_time_.tv_sec + (double)start_time_.tv_nsec / (double)1000000000l);
      RCLCPP_INFO(this->get_logger(), "Total elapsed time: %.9f seconds", elapsed_time);
      exit(0);
    } else {
      count_++;
      publish_message();
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
  std::shared_ptr<std_msgs::msg::String> message_;
  struct timespec start_time_, end_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

