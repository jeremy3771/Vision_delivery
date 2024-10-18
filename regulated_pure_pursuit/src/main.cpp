#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "regulated_pure_pursuit/regulated_pure_pursuit.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto RPP_node = std::make_shared<RegulatedPurePursuitController>();
    rclcpp::spin(RPP_node);
    rclcpp::shutdown();

    return 0;
}