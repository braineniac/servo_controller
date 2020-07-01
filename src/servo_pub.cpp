#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ServoPublisher: public rclcpp::Node
{
    public:
        ServoPublisher(int servo_number): Node("servo_publisher"), count_(0), number(servo_number)
        {
            std::string topic = "servo";
            topic += std::to_string(number);
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 5);
            timer_ = this->create_wall_timer(2s, std::bind(&ServoPublisher::timer_callback, this));
        }
    private:
        void timer_callback()
        {
            double angle = 0;
            auto twist = geometry_msgs::msg::Twist();
            twist.angular.z = angle;
            RCLCPP_INFO(this->get_logger(), "Setting servo %d to %f degrees!", this->number, angle);
            publisher_->publish(twist);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        size_t count_;
        int number;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int number = 0;
    rclcpp::spin(std::make_shared<ServoPublisher>(number));
    rclcpp::shutdown();
    return 0;
}