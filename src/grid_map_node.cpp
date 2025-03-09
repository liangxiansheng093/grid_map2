#include "grid_map.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("grid_map_node");

    GridMap grid_map;

    grid_map.init_map(node);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Eigen::Vector3d pos(0, 0, 0);
        double dist;
        grid_map.evaluateEDT(pos, dist);
        Eigen::Vector3d dist_grad;
        grid_map.evaluateFirstGrad(pos, dist_grad);
    }
    rclcpp::shutdown();

    return 0;
}
