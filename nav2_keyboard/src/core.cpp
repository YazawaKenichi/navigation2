#include "core.hpp"

using namespace std::chrono_literals;

Nav2Keyboard::Nav2Keyboard(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("keyboard_node", options), key_(' ')
{
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Nav2Keyboard::on_configure(const rclcpp_lifecycle::State &)
{
    // トピック初期化 サービス作成
    RCLCPP_INFO(get_logger(), "SOIYA");
    publisher_ = create_publisher<std_msgs::msg::String>("/nav2_key", 10);
    subscriber_ = create_subscription<std_msgs::msg::String>("/nav2_key", 10, std::bind(&Nav2Keyboard::subscriber_callback, this, std::placeholders::_1));
    service_client_ = this->create_client<std_srvs::srv::SetBool>("/wait_waypoint");
    RCLCPP_INFO(get_logger(), "\x1b[34m on_configure \x1b[0m");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Nav2Keyboard::on_activate(const rclcpp_lifecycle::State &)
{
    // サブスクライバ・サービスクライアント初期化
    publish_message();
    RCLCPP_INFO(get_logger(), "\x1b[34m on_activate \x1b[0m");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Nav2Keyboard::on_active(const rclcpp_lifecycle::State &)
{
    auto service_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    // メイン処理
    /*
    key_ = getch();
    RCLCPP_INFO(get_logger(), "\x1b[34m on_active \x1b[0m");
    if(key_)
    {
        service_request->data = false;
        auto service_future = service_client_->async_send_request(service_request);
        RCLCPP_INFO(get_logger(), "\x1b[34m key_(%c) \x1b[0m", key_);
    }
    */
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Nav2Keyboard::on_deactivate(const rclcpp_lifecycle::State &)
{
    // サブスクライバ・サービスクライアント解放
    RCLCPP_INFO(get_logger(), "\x1b[34m on_deactivate \x1b[0m");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Nav2Keyboard::on_cleanup(const rclcpp_lifecycle::State &)
{
    // リソース解放
    service_client_.reset();
    RCLCPP_INFO(get_logger(), "\x1b[34m on_cleanup \x1b[0m");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Nav2Keyboard::publish_message()
{
    auto message = std_msgs::msg::String();
    message.data = "";
    publisher_->publish(message);
}

void Nav2Keyboard::subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Received Message %s", msg->data.c_str());
}

int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

