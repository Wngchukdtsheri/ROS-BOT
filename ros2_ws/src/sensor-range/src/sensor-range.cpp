#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0), min_range_(10), max_range_(20.0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Range>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::Range();
      message.range = static_cast<float>(count_++);
      message.min_range = static_cast<float>(min_range_);
      message.max_range = static_cast<float>(max_range_);
      RCLCPP_INFO(this->get_logger(), "Publishing: Range=%f, Min Range=%f, Max Range=%f",message.range, message.min_range, message.max_range);
      publisher_->publish(message);

       count_++;
        if (count_ > 100)  
        {
            count_ = 0;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    size_t count_;
    double min_range_;
    double max_range_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
