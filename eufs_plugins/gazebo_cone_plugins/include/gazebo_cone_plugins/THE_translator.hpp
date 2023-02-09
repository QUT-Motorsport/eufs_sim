#include <cmath>
#include <vector>

#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/cone.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"

const float MAX_ANGLE = 0.15;

std::vector<driverless_msgs::msg::Cone> cone_array_with_covariance_to_cone_list(const eufs_msgs::msg::ConeArrayWithCovariance& eufs_cones)
{
    std::vector<driverless_msgs::msg::Cone> internal_cones;
    for (const auto& eufs_cone : eufs_cones.blue_cones) {
        driverless_msgs::msg::Cone internal_cone;
        internal_cone.location = eufs_cone.point;
        internal_cone.color = driverless_msgs::msg::Cone::BLUE;
        internal_cones.push_back(internal_cone);
    }
    for (const auto& eufs_cone : eufs_cones.yellow_cones) {
        driverless_msgs::msg::Cone internal_cone;
        internal_cone.location = eufs_cone.point;
        internal_cone.color = driverless_msgs::msg::Cone::YELLOW;
        internal_cones.push_back(internal_cone);
    }
    for (const auto& eufs_cone : eufs_cones.orange_cones) {
        driverless_msgs::msg::Cone internal_cone;
        internal_cone.location = eufs_cone.point;
        internal_cone.color = driverless_msgs::msg::Cone::ORANGE_SMALL;
        internal_cones.push_back(internal_cone);
    }
    for (const auto& eufs_cone : eufs_cones.big_orange_cones) {
        driverless_msgs::msg::Cone internal_cone;
        internal_cone.location = eufs_cone.point;
        internal_cone.color = driverless_msgs::msg::Cone::ORANGE_BIG;
        internal_cones.push_back(internal_cone);
    }

    return internal_cones;
}