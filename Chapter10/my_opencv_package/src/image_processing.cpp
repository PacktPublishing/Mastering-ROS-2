#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.hpp> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {
        // Subscribe to the input image topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

        // Publisher for processed image
        _contours_imag_pub      = this->create_publisher<sensor_msgs::msg::Image>( "contours/image_raw", 1);
        _gray_img_pub           = this->create_publisher<sensor_msgs::msg::Image>( "gray/image_raw", 1);
        _blur_img_pub           = this->create_publisher<sensor_msgs::msg::Image>( "blur/image_raw", 1);
        _thresholded_img_pub    = this->create_publisher<sensor_msgs::msg::Image>( "thresholded/image_raw", 1);
        _prespective_img_pub    = this->create_publisher<sensor_msgs::msg::Image>( "prespective/image_raw", 1);
        _corner_img_pub         = this->create_publisher<sensor_msgs::msg::Image>( "corner/image_raw", 1);
        _keypoints_img_pub      = this->create_publisher<sensor_msgs::msg::Image>( "keypoints/image_raw", 1);
    
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)  {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat processed_image = cv_ptr->image.clone();
        cv::Mat contours_image = cv_ptr->image.clone();  
        cv::Mat gray_image;
        cv::Mat blur_image;
        cv::Mat thresholded_image;
        cv::Mat prespective_img;
        cv::Mat corner_image;
        cv::Mat keypoints_img;

        // 0. Convert to grayscale if needed
        cv::cvtColor(processed_image, gray_image, cv::COLOR_BGR2GRAY);

        // 1. Blurring        
        cv::blur(processed_image, blur_image, cv::Size(15, 15));
            
        // 2. Thresholding        
        cv::threshold(gray_image, thresholded_image, 128, 255, cv::THRESH_BINARY);

        // 3. Contour Detection
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholded_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::drawContours(contours_image, contours, -1, cv::Scalar(0, 255, 0), 2);
        
        // 4. Feature Detection and Description
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(gray_image, cv::noArray(), keypoints, descriptors);
        cv::drawKeypoints(processed_image, keypoints, keypoints_img, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // corners
        cv::Mat dst, dst_norm, dst_norm_scaled;
        dst = cv::Mat::zeros(gray_image.size(), CV_32FC1);
        // Parameters for Harris corner detection
        int blockSize = 2;      // Neighborhood size
        int apertureSize = 3;    // Aperture parameter for the Sobel operator
        double k = 0.04;         // Harris detector free parameter
        // Detecting corners
        cv::cornerHarris(gray_image, dst, blockSize, apertureSize, k);
        // Normalize the result
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        // Convert to an 8-bit image to draw on it
        cv::convertScaleAbs(dst_norm, dst_norm_scaled);
        corner_image = processed_image.clone();
        for (int i = 0; i < dst_norm.rows; i++) {
            for (int j = 0; j < dst_norm.cols; j++) {
                if ((int)dst_norm.at<float>(i, j) > 200) {
                    cv::circle(corner_image, cv::Point(j, i), 5, cv::Scalar(0, 0, 255), 2, 8, 0);
                }
            }
        }
    
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", contours_image).toImageMsg();
        _contours_imag_pub->publish(*msg_.get());
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_image).toImageMsg();
        _gray_img_pub->publish(*msg_.get());
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blur_image).toImageMsg();
        _blur_img_pub->publish(*msg_.get());
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", thresholded_image).toImageMsg();
        _thresholded_img_pub->publish(*msg_.get());
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", corner_image).toImageMsg();
        _corner_img_pub->publish(*msg_.get());
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", prespective_img).toImageMsg();
        _prespective_img_pub->publish(*msg_.get());
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", keypoints_img).toImageMsg();
        _keypoints_img_pub->publish(*msg_.get());
    }

   
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _contours_imag_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _gray_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _blur_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _thresholded_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _prespective_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _corner_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _keypoints_img_pub;
    sensor_msgs::msg::Image::SharedPtr msg_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}

