#include "rclcpp/rclcpp.hpp"

#include "vex_message/msg/vexrotationsensor.hpp"
#include "std_msgs/msg/float32.hpp"

#include "template.cpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node("down");
    auto publisher_motor = node.create_publisher<std_msgs::msg::Float32>("SpinVolt_motor_1", 10);

    auto message = std_msgs::msg::Float32();
    message.data = 0;
    publisher_motor->publish(message);

    rclcpp::shutdown();

    return 0;
}