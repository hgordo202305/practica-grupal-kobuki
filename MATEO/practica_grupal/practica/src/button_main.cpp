#include "rclcpp/rclcpp.hpp"
#include "practica/ButtonNode.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<practica::ButtonNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
