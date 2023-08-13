#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <string>
#include <limits>

#include "sensor_msgs/msg/laser_scan.hpp"

class FilteredScanPublisher : public rclcpp::Node
{
    public:

        FilteredScanPublisher() : Node("filtered_scan_publisher")
        {
            // ポールとして認識する最小距離(m)
            this->declare_parameter("scan_min", 0.3);
            scan_min = this->get_parameter("scan_min").as_double();

            // ポールとして認識する最大距離(m)
            this->declare_parameter("scan_max", 1.0);
            scan_max = this->get_parameter("scan_max").as_double();

            // Publisherの設定
            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);

            // Subscriberの設定
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FilteredScanPublisher::scan_callback, this, std::placeholders::_1));
        }

    private:

        void scan_callback(sensor_msgs::msg::LaserScan msg)
        {
            
            for(int i = 0; i < int(msg.ranges.size()); i++)
            {
                if (msg.ranges[i] < scan_min || scan_max < msg.ranges[i])
                {
                    msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                }
            }
            publisher_->publish(msg);
            
            //RCLCPP_INFO(this->get_logger(), "ranges[%ld]: %f\n", msg.ranges.size()-1, msg.ranges[msg.ranges.size()-1]);
        }

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

        double scan_min;
        double scan_max;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilteredScanPublisher>());
    rclcpp::shutdown();
    return 0;
}