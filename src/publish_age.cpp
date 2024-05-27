#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/age.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AgePublisher : public rclcpp::Node {
public:
  AgePublisher() : Node("age_publisher") {
    age_.years = 49;
    age_.months = 3;
    age_.days = 27;
    publisher_ = this->create_publisher<custom_interfaces::msg::Age>("age", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&AgePublisher::age_callback, this));
  }

private:
  void age_callback() {
    publisher_->publish(age_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;
  custom_interfaces::msg::Age age_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgePublisher>());
  rclcpp::shutdown();
  return 0;
}