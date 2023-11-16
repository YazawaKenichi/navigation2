#include "core.hpp"

int main(int argc, char * argv[])
{
    std::cout << " init" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << " exe " << std::endl;
    rclcpp::executors::SingleThreadedExecutor exe;
    std::cout << " make_shared " << std::endl;
    auto node = std::make_shared<Nav2Keyboard>(rclcpp::NodeOptions());
    std::cout << " add_node " << std::endl;
    exe.add_node(node->get_node_base_interface());
    std::cout << " spin " << std::endl;
    exe.spin();
    std::cout << " shutdown " << std::endl;
    rclcpp::shutdown();
    std::cout << " return " << std::endl;
    return 0;
}

