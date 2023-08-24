#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <memory>
#include <string>
#include <limits>

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class PoleDetector : public rclcpp::Node
{
    public:

        PoleDetector() : Node("pole_detector")
        {
            // ポールとして認識する最小距離(m)
            this->declare_parameter("scan_min", 0.3);
            scan_min = this->get_parameter("scan_min").as_double();

            // ポールとして認識する最大距離(m)
            this->declare_parameter("scan_max", 1.0);
            scan_max = this->get_parameter("scan_max").as_double();

            // Publisherの設定
            publisher_ = this->create_publisher<std_msgs::msg::Float32>("/pole_distance", 10);

            // PublishするデータをNaNで初期化
            distance_msg.data = std::numeric_limits<double>::quiet_NaN();

            // Subscriberの設定
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PoleDetector::scan_callback, this, std::placeholders::_1));
        }

    private:

        void scan_callback(sensor_msgs::msg::LaserScan msg)
        {
            // 将来的には１周期前のポール位置から現在周期のポール位置に制限を与えるといいかも
            // 例：１周期前は-60度の位置にポールがあったから現在周期では-60度から-50度ぐらいの範囲内にあるはず

            // 最大クラスタの端の添字
            int start_index = 0;
            int last_index = 0;

            // 最大クラスタの長さ
            int index_range = 0;

            // クラスタの端の添字（一時保存）
            int temp_start_index = 0;
            int temp_last_index = 0;

            if (msg.ranges[0] < scan_min || scan_max < msg.ranges[0]){
                // NaNに置き換え
                msg.ranges[0] = std::numeric_limits<float>::quiet_NaN();
            }else{
                // クラスタの端として一時保存
                temp_start_index = 0;
            }

            // 指定範囲外のデータをNaNに置き換えながら最も大きいクラスタを探す
            for(int i = 1; i < int(msg.ranges.size()); i++)
            {
                if (msg.ranges[i] < scan_min || scan_max < msg.ranges[i]){

                    // NaNに置き換え
                    msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();

                    // もし一つ前のデータがNaNでないなら
                    if(!std::isnan(msg.ranges[i-1])){
                        temp_last_index = i;
                        // もし保存されているクラスタより現在のクラスタが長ければ更新
                        if (index_range < temp_last_index - temp_start_index){
                            start_index = temp_start_index;
                            last_index = temp_last_index;
                            index_range = temp_last_index - temp_start_index;
                        }
                    }
                }else{
                    // もし一つ前のデータがNaNならクラスタの端として一時保存
                    if (std::isnan(msg.ranges[i-1])){
                        temp_start_index = i;
                    }
                }
            }

            // もし最後のデータがNaNでないならクラスタの端として処理
            if (!std::isnan(msg.ranges[msg.ranges.size()-1])){

                temp_last_index = msg.ranges.size();

                // もし保存されているクラスタより現在のクラスタが長ければ更新
                if (index_range < temp_last_index - temp_start_index)
                {
                    start_index = temp_start_index;
                    last_index = temp_last_index;
                    index_range = temp_last_index - temp_start_index;
                }
            }

            // 最大クラスタの長さが0でなければクラスタ中心とそこまでのロボット進行方向距離を計算
            if (index_range != 0){

                // クラスタ中心の角度(rad)
                float center_angle = msg.angle_min + (start_index + last_index)/2 * msg.angle_increment;

                // クラスタ中心の距離(m)
                float sum = 0;
                for(int i = start_index; i < last_index; i++)
                {
                    sum += msg.ranges[i];
                }
                float center_range = sum / index_range;

                // クラスタ中心距離 * sin(クラスタ中心角度) 角度の基準が単位円のpi/2なのでcosではなくsin
                distance_msg.data = center_range * std::sin(center_angle);

                publisher_->publish(distance_msg);

                RCLCPP_INFO(this->get_logger(), "rad:'%f' range:'%f' distance:'%f'", center_angle, center_range, distance_msg.data);
            }else{
                RCLCPP_INFO(this->get_logger(), "Cluster not found.");
            }
        }

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

        std_msgs::msg::Float32 distance_msg;

        double scan_min;
        double scan_max;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoleDetector>());
    rclcpp::shutdown();
    return 0;
}