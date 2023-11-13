# WayPoint YAML Load

## YAML Load のボタンを押されたときのコールバック登録
[navigation2/nav2_rviz_plugins/src/nav2_panel.cpp#L380-L384](https://github.com/YazawaKenichi/navigation2/blob/feat/tsukuba-challenge-2023-ex/nav2_rviz_plugins/src/nav2_panel.cpp#L380-L384)

``` C++
QObject::connect(
    load_waypoints_button_,
    &QPushButton::released,
    this,
    &Nav2Panel::handleGoalLoader);
```

## YAML Load のボタンが押されたときのコールバック関数
[navigation2/nav2_rviz_plugins/src/nav2_panel.cpp#L673-L704](https://github.com/YazawaKenichi/navigation2/blob/feat/tsukuba-challenge-2023-ex/nav2_rviz_plugins/src/nav2_panel.cpp#L673-L704)
``` C++
void Nav2Panel::handleGoalLoader()
{
    acummulated_poses_.clear();

    std::cout << "Loading Waypoints!" << std::endl;

    //! ファイルを開くダイアログを表示
    QString file = QFileDialog::getOpenFileName(
        this,
        tr("Open File"), "",
        tr("yaml(*.yaml);;All Files (*)"));

    YAML::Node available_waypoints;

    //! YAML ファイルの内容を格納
    try {
        available_waypoints = YAML::LoadFile(file.toStdString());
    } catch (const std::exception & ex) {
        std::cout << ex.what() << ", please select a valid file" << std::endl;
        updateWpNavigationMarkers();
        return;
    }

    /*
     * `waypoint.yaml`
     * waypoints:
     *     waypoint0:
     *         pose:
     *             - ****
     *             - ****
     *             - ****
     *         orientation:
     *             - ****
     *             - ****
     *             - ****
     *     waypoint1:
     *         pose:
     *             - ****
     *             - ****
     *             - ****
     *         orientation:
     *             - ****
     *             - ****
     *             - ****
     *     waypoint3:
     *         pose:
     *             - ****
     *             - ****
     *             - ****
     *         orientation:
     *             - ****
     *             - ****
     *             - ****
     *         ...
     */

    //! YAML 中の "waypoints" キーにアクセス
    const YAML::Node & waypoint_iter = available_waypoints["waypoints"];
    for (YAML::const_iterator it = waypoint_iter.begin(); it != waypoint_iter.end(); ++it) {
        auto waypoint = waypoint_iter[it->first.as<std::string>()];
        //! waypoints: waypoint*: pose: の内容を取得
        auto pose = waypoint["pose"].as<std::vector<double>>();
        //! waypoints: waypoint*: orientation: の内容を取得
        auto orientation = waypoint["orientation"].as<std::vector<double>>();
        //! acummulated_poses_ の配列末尾に追加
        acummulated_poses_.push_back(convert_to_msg(pose, orientation));
    }

    //! ↓ この英文読む感じ Publish もしてるっぽ？
    // Publishing Waypoint Navigation marker after loading wp's
    updateWpNavigationMarkers();
}
```

## acummulated_poses_ の正体
[navigation2/nav2_rviz_plugins/include/nav2_rviz_plugins/nav2_panel.hpp#L196](https://github.com/YazawaKenichi/navigation2/blob/feat/tsukuba-challenge-2023-ex/nav2_rviz_plugins/include/nav2_rviz_plugins//nav2_panel.hpp#L196)
``` C++
std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
```

[geometry_msgs/PoseStamped Message](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)
``` Python
# A Pose with reference coordinate frame and timestamp
std_msgs/Header header
geometry_msgs/Pose pose
```
## Publish してる関数
[navigation2/nav2_rviz_plugins/src/nav2_panel.cpp#L1369-L1441](https://github.com/YazawaKenichi/navigation2/blob/feat/tsukuba-challenge-2023-ex/nav2_rviz_plugins/src/nav2_panel.cpp#L1369-L1441)

``` C++
void
Nav2Panel::updateWpNavigationMarkers()
{
    resetUniqueId();

    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    for (size_t i = 0; i < acummulated_poses_.size(); i++) {
        // Draw a green arrow at the waypoint pose
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header = acummulated_poses_[i].header;
        arrow_marker.id = getUniqueId();
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose = acummulated_poses_[i].pose;
        arrow_marker.scale.x = 0.3;
        arrow_marker.scale.y = 0.05;
        arrow_marker.scale.z = 0.02;
        arrow_marker.color.r = 0;
        arrow_marker.color.g = 255;
        arrow_marker.color.b = 0;
        arrow_marker.color.a = 1.0f;
        arrow_marker.lifetime = rclcpp::Duration(0s);
        arrow_marker.frame_locked = false;
        marker_array->markers.push_back(arrow_marker);

        // Draw a red circle at the waypoint pose
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header = acummulated_poses_[i].header;
        circle_marker.id = getUniqueId();
        circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose = acummulated_poses_[i].pose;
        circle_marker.scale.x = 0.05;
        circle_marker.scale.y = 0.05;
        circle_marker.scale.z = 0.05;
        circle_marker.color.r = 255;
        circle_marker.color.g = 0;
        circle_marker.color.b = 0;
        circle_marker.color.a = 1.0f;
        circle_marker.lifetime = rclcpp::Duration(0s);
        circle_marker.frame_locked = false;
        marker_array->markers.push_back(circle_marker);

        // Draw the waypoint number
        visualization_msgs::msg::Marker marker_text;
        marker_text.header = acummulated_poses_[i].header;
        marker_text.id = getUniqueId();
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;
        marker_text.pose = acummulated_poses_[i].pose;
        marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
        marker_text.scale.x = 0.07;
        marker_text.scale.y = 0.07;
        marker_text.scale.z = 0.07;
        marker_text.color.r = 0;
        marker_text.color.g = 255;
        marker_text.color.b = 0;
        marker_text.color.a = 1.0f;
        marker_text.lifetime = rclcpp::Duration(0s);
        marker_text.frame_locked = false;
        marker_text.text = "wp_" + std::to_string(i + 1);
        marker_array->markers.push_back(marker_text);
    }

    if (marker_array->markers.empty()) {
        visualization_msgs::msg::Marker clear_all_marker;
        clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.push_back(clear_all_marker);
    }

    wp_navigation_markers_pub_->publish(std::move(marker_array));
}
```

## Publish してるインスタンス
[navigation2/nav2_rviz_plugins/src/nav2_panel.cpp#L590-L593](https://github.com/YazawaKenichi/navigation2/blob/feat/tsukuba-challenge-2023-ex/nav2_rviz_plugins/src/nav2_panel.cpp#L590-L503)

``` C++
wp_navigation_markers_pub_ = client_node_->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS(1).transient_local());
```

# waypoint_follower.cpp

WP の情報を読み込んでると思われる部分

``` C++
//! L150
void WaypointFollower::followWaypoints()
{
    auto goal = acthion_server_->get_current_goal();
    uint32_t goal_index = goal->goal_index;
    bool new_goal = true;

    //! L202
    if(new_goal)
    {
        new_goal = false;
        ClientT::Goal client_goal;
        client_goal.pose = goal->poses[goal_index];
    }

    //! L240
    if(current_goal_status_.status == ActionStatus::SUCCEEDED)
    {
        bool is_task_executed = waypoint_task_executor_->processAtWaypoint(goal->poses[goal_index], goal_index);
    }

    //! L274
    if(current_goal_status_.status != ActionStatus::PROCESSING && current_goal_status_.status != ActionStatus::UNKNOWN)
    {
        goal_index++;
        new_goal = true;
        if(goal_index >= goal->poses.size())
        {
            if(current_loop_no == no_of_loops)
            {
                action_server_->succeeded_current(result);
                current_goal_status_error_code = 0;
                return;
            }
            goal_index = 0;
            current_loop_no++;
        }
    }

    callback_group_executor_.spin_some();
    r.sleep();
}
```

# WP に到達したときの標準出力の内容
```
[controller_server-5] [INFO] [1699895899.739644078] [controller_server]: Passing new path to controller.
[controller_server-5] [INFO] [1699895899.751342308] [controller_server]: Reached the goal!
[bt_navigator-9] [INFO] [1699895899.798471035] [bt_navigator]: Goal succeeded
[waypoint_follower-10] [INFO] [1699895899.812896064] [waypoint_follower]: Succeeded processing waypoint 1, processing waypoint task execution
[waypoint_follower-10] [INFO] [1699895899.812961367] [waypoint_follower]: Arrived at 1'th waypoint, sleeping for 200 milliseconds
[waypoint_follower-10] [INFO] [1699895900.013070438] [waypoint_follower]: Task execution at waypoint 1 succeeded
[waypoint_follower-10] [INFO] [1699895900.013188961] [waypoint_follower]: Handled task execution on waypoint 1, moving to next.
[bt_navigator-9] [INFO] [1699895900.013991663] [bt_navigator]: Begin navigating from current location (2.54, 7.81) to (-5.59, -2.26)
[controller_server-5] [INFO] [1699895900.035233758] [controller_server]: Received a goal, begin computing control effort.
```

# waypoint_follower
`Succeeded processing waypoint` を出力している関数内でウェイト処理をするべきなのでは？

ということでこの関数内をトレースデバッグ。

## waypoint_follower/src/waypoint_follower.cpp
``` C++
WaypointFollower::WaypointFollower(const rclcpp::NodeOptions & options) : nav2_util::LifecycleNode("waypoint_follower", "", options), waypoint_task_executor_loader_("nav2_waypoint_follower", "nav2_core::WaypointTaskExecutor")
{
    nav2_util::declare_parameter_if_not_declared(this, std::string("waypoint_task_executor_plugin", rclcpp::ParameterValue(std::string("wait_at_waypoint")));
    nav2_util::declare_parameter_if_not_declared(this, std::string("wait_at_waypoint.plugin", rclcpp::ParameterValue(std::string("nav2_waypoint_follower::WaitAtWaypoint")));
}

nav2_util::CallbackReturn WaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    try
    {
        waypoint_task_executor_id_ = get_parameter("wawypoint_task_executor_plugin").as_string();
        waypoint_task_executor_type = nav2_util::get_plugin_type_param(this, waypoint_task_executor_id_);
        waypoint_task_executor_ = waypoint_task_executor_loader_.createUniqueInstance(waypoint_task_executor_tpye_);
        waypoint_task_executor_->initialize(node, waypoint_task_executor_id_);
    }
}

void WaypointFollower::followWaypoints()
{
    RCLCPP_INFO(get_logger(), "Succeeded processing waypoint %i, processing waypoint task execution", goal_index);
    bool is_task_executed = waypoint_task_executor_->processAtWaypoint(goal->poses[goal_index], goal_index);
    RCLCPP_INFO(get_logger(), "Task execution at waypoint %i %s", goal_index, is_task_executed ? "succeeded" : "failed!");
}
```

## raspicat_slam_navigation/raspicat_navigation/config/param/nav2.param.yaml
``` YAML
waypoint_follower:
    ros__parameters:
        loop_rate : 200
        stop_on_failure: false
        waypoint_task_executor_plugin: "wait_at_waypoint"
        wait_at_waypoint:
            plugin: "nav2_waypoint_follower::WaitAtWaypoint"
            enabled: True
            waypoint_pause_duration: 200
```



