#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class pointcloud_to_laserscan : public rclcpp::Node {
public:
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_,
        range_max_;

    pointcloud_to_laserscan() : Node("pointcloud_to_laserscan") {
        // --ros-args -p min_height:=0.5 ...
        declare_parameter("min_height", std::numeric_limits<double>::min());
        declare_parameter("max_height", 0.5);
        declare_parameter("angle_min", -M_PI);
        declare_parameter("angle_max", M_PI);
        declare_parameter("angle_increment", M_PI / 180.0);
        declare_parameter("scan_time", 1.0 / 30.0);
        declare_parameter("range_min", 0.0);
        declare_parameter("range_max", std::numeric_limits<double>::max());
        
        get_parameter("min_height", min_height_);
        get_parameter("max_height", max_height_);
        get_parameter("angle_min", angle_min_);
        get_parameter("angle_max", angle_max_);
        get_parameter("angle_increment", angle_increment_);
        get_parameter("scan_time", scan_time_);
        get_parameter("range_min", range_min_);
        get_parameter("range_max", range_max_);
        
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));
        laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan", qos_profile);

        point_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/roi_lidar", qos_profile, std::bind(&pointcloud_to_laserscan::subscribe_cloud, this, _1));
    }
private:
    void subscribe_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) const {
        //cloud 가공 후 LaserScan 발행
        auto twoD_msg = make_twoD(cloud);
        publish_laserscan(std::move(twoD_msg));
    }
    //pointcloud2 데이터 가공
    std::unique_ptr<sensor_msgs::msg::LaserScan> make_twoD(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) const {
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header = cloud->header;
        scan_msg->header.frame_id = "livox_frame";
        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;

        //할당
        uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"),
            iter_y(*cloud, "y"), iter_z(*cloud, "z");
            iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            //ROI 적용
            double range = hypot(*iter_x, *iter_y);
            double angle = atan2(*iter_y, *iter_x);
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z) ||    // nan point
                *iter_z > max_height_ || *iter_z < min_height_ ||                       // 고도
                range < range_min_ || range > range_max_ ||                             // 거리
                angle < scan_msg->angle_min || angle > scan_msg->angle_max              // 각도
            ) continue;

            // LaserScan->ranges 초기화; 각도 비슷=>overwrite
            int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
            if (range < scan_msg->ranges[index]) {
                scan_msg->ranges[index] = range;
            }
        }

        return scan_msg;
    }
    //msg 발행
    void publish_laserscan(std::unique_ptr<sensor_msgs::msg::LaserScan> msg) const {
        laser_publisher_->publish(std::move(msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_to_laserscan>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
