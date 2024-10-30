#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
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
    csv_file_path_ = "/root/ros2_ws/measurement_results.csv";

    this->declare_parameter<int>("message_size", 256);
    this->get_parameter("message_size", message_size_);

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
    message_->data = std::string(message_size_, 'x') + " " + std::to_string(count_);
    publisher_->publish(*message_);
  }

  void topic_callback(const std_msgs::msg::String &)
  {
    if(count_ == 99) { 
      GET_TIME(end_time_, "Failed to get subscribe time");
      double elapsed_time = ((double)end_time_.tv_sec + (double)end_time_.tv_nsec / (double)1000000000l)
                             - ((double)start_time_.tv_sec + (double)start_time_.tv_nsec / (double)1000000000l);
      RCLCPP_INFO(this->get_logger(), "Total elapsed time: %.9f seconds", elapsed_time);

      double average = elapsed_time / 100;
      RCLCPP_INFO(this->get_logger(), "Average elapsed time: %.9f seconds", average);

      std::ofstream csv_file;
      csv_file.open(csv_file_path_, std::ios_base::app);
      if (csv_file.is_open()) {
        csv_file << message_size_ << "," << average << "\n";
        csv_file.close();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
      }
      exit(0);
    } else {
      count_++;
      publish_message();
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
  int message_size_;
  std::shared_ptr<std_msgs::msg::String> message_;
  struct timespec start_time_, end_time_;
  std::string csv_file_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

