#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
 public:
  MinimalPublisher()
  : Node("numeros_aleatorios"), num_(0)
  {
   publisher_ = this->create_publisher<std_msgs::msg::String>("number_stream", 10);
   timer_ = this->create_wall_timer(
    1000ms, std::bind(&MinimalPublisher::timer_callback, this));
  } 
  
  private:
   void timer_callback()
   {
    // int num=0;
    num_=rand() % 10000;
    auto message = std_msgs::msg::String();
    message.data = "Numero aleatorio: " + std::to_string(num_);
    RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
    publisher_->publish(message);
   }
   rclcpp::TimerBase::SharedPtr timer_;
   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   size_t num_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
