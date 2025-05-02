#include "image_saver.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}
