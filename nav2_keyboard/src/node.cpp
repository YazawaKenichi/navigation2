#include "core.hpp"

int main(int argc, char * argv[])
{
    char key = ' ';
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yazaop");
    auto service_client = node->create_client<std_srvs::srv::SetBool>("/wait_waypoint");
    auto service_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    while(rclcpp::ok())
    {
        key = getch();
        if(key)
        {
            service_request->data = true;
            auto service_future = service_client->async_send_request(service_request);
            service_request->data = false;
            std::cout << "----- Debug Line (" << key << ") -----" << std::endl;
        }
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

