#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class YUV2ToRGBConverter : public rclcpp::Node
{
public:
    YUV2ToRGBConverter() : Node("yuv2_to_rgb_converter")
    {
        // 订阅YUV2图像话题  
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/cam0/image_raw",
            10,
            std::bind(&YUV2ToRGBConverter::image_callback, this, std::placeholders::_1));
        
        // 发布RGB图像话题
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/cam0/image_rgb", 10);
        
        RCLCPP_INFO(this->get_logger(), "YUV2 to RGB converter node started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat rgb_image;
            
            cv::Mat yuv_image(msg->height, msg->width, CV_8UC2, (void*)msg->data.data());
            cv::cvtColor(yuv_image, rgb_image, cv::COLOR_YUV2RGB_YUYV);
            
            // 创建输出消息
            auto rgb_msg = cv_bridge::CvImage(msg->header, "rgb8", rgb_image).toImageMsg();
            
            // 发布RGB图像
            publisher_->publish(*rgb_msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YUV2ToRGBConverter>());
    rclcpp::shutdown();
    return 0;
}
