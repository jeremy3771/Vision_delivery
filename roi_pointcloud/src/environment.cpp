 #include "roi_pointcloud/environment.hpp"

using std::placeholders::_1;

environment::environment() : Node("lidar_environment") {
    RCLCPP_INFO(get_logger(), "Start Lidar Environment Node");

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5));

    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/lidar", qos_profile, std::bind(&environment::pointcloud_cb, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/roi_lidar", 2);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr environment::FilterCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);

    // Filter Points on the Robot Roof Top
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(Eigen::Vector4f(-1.0, -0.5, -0.5, 1));
    roof.setMax(Eigen::Vector4f(0.0, 0.5, 0.5, 1));
    roof.setInputCloud(cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    // Remove Roof Points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudFiltered);

    return cloudFiltered;
}

void environment::pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 cloud_out;

    pcl::fromROSMsg(*cloud_msg, *cloud_dst);

    cloud_filtered = FilterCloud(cloud_dst);
    pcl::toROSMsg(*cloud_filtered,cloud_out);

    cloud_out.header.frame_id = cloud_msg->header.frame_id;
    cloud_out.header.stamp = cloud_msg->header.stamp;

    publisher_->publish(cloud_out);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<environment>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
