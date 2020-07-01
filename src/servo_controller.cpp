#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sg90/sg90/sg90.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class ServoSub: public rclcpp::Node
{
    public:
        ServoSub(int servo_number): Node("servoSub"), number(servo_number)
        {
            sg90_ = new sg90::sg90(1, 0x40, servo_number);
            std::string topic = "servo";
            topic += std::to_string(number);
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>
            (topic, 5, std::bind(&ServoSub::topic_callback, this, _1));
        }
    private:
        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr twist);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        int number;
        sg90::sg90* sg90_;
};

void ServoSub::topic_callback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
    RCLCPP_INFO(this->get_logger(), "Move servo to angle: %f", twist->angular.z);
    this->sg90_->setPos(twist->angular.z);
}  

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int number = 0;
    rclcpp::spin(std::make_shared<ServoSub>(number));
    rclcpp::shutdown();
    return 0;
}