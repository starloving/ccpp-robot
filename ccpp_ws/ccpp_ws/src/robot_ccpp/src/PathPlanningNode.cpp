#include "robot_ccpp/CleaningPathPlanner.h"
#include <boost/shared_ptr.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <memory>

namespace cm = nav2_costmap_2d;
namespace rm = geometry_msgs::msg;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;


int main(int argc, char** argv) {
    rclcpp::init(argc, argv) ;
    auto clr_node = std::make_shared<CleaningPathPlanning>();
    clr_node->GetPathInROS() ;
    rclcpp::Rate rate(5) ;  // 1 Hz
    while(rclcpp::ok()) {
        clr_node->PublishCoveragePath();
        clr_node->PublishGrid();
        rclcpp::spin_some(clr_node) ;
        // rclcpp::spin_some(clr_node.costmap2d_ros_->) ;
        rate.sleep() ;
    }
    rclcpp::shutdown() ;   
    
    return 0 ;
}

