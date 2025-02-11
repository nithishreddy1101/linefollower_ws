#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using std::placeholders::_1;

class LineFollower :public rclcpp::Node 
{
public:
    LineFollower() : Node("line_follower_node")
    {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,std::bind(&LineFollower::cameraCallback,this,_1));

        RCLCPP_INFO_STREAM(this->get_logger(), "Line Follower Robot");
        
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    void cameraCallback(const sensor_msgs::msg::Image &image){
         try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
            cv::Mat gray, blur, threshold;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
            cv::threshold(blur, threshold, 60, 255, cv::THRESH_BINARY_INV);

            int height = threshold.rows;
            int width = threshold.cols;
            cv::Mat roi = threshold(cv::Rect(0, height / 2, width, height / 2));

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                auto largest_contour = *std::max_element(contours.begin(), contours.end(),
                                                         [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                                                         { return cv::contourArea(a) < cv::contourArea(b); });

                cv::Moments M = cv::moments(largest_contour);
                if (M.m00 != 0)
                {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int deviation = cx - (width / 2);
                    RCLCPP_INFO(this->get_logger(), "Deviation: %d", deviation);
                }
            }

            cv::imshow("Camera Feed", cv_ptr->image);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollower>());
    rclcpp::shutdown();
    return 0;
}