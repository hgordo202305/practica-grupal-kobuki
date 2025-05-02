#ifndef IMAGE_SAVER_HPP
#define IMAGE_SAVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <string>

class ImageSaver : public rclcpp::Node {
public:
    ImageSaver();
    ~ImageSaver();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    std::string getTimestamp();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    bool image_saved_;
};

#endif // IMAGE_SAVER_HPP
