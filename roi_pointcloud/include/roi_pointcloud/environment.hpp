#ifndef ROI_POINTCLOUD__ENVIRONMENT_HPP_
#define ROI_POINTCLOUD__ENVIRONMENT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <iterator>
#include <algorithm>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

struct Plane {
    Eigen::Vector3f normal;
    float d;
};

class environment : public rclcpp::Node {
public:
    // Constructor
    explicit environment();

private:
    /// @brief Apply voxel grid & ROI filtering to the point cloud
    /// @param[in] cloud PCL PointCloud2 Data
    /// @param[in] filterRes XYZ Leaf Size
    /// @param[in] minPoint ROI Min Point
    /// @param[in] maxPoint ROI Max Points
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr      cloud,
        double                                   filterRes,
        Eigen::Vector4f                          minPoint,
        Eigen::Vector4f                          maxPoint);

    /// @brief Detect object using PointCloud2
    /// @param[in] cloud_msg ROS2 PointCloud2 data
    void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif
