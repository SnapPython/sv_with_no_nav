// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/send_serial.hpp"
#include "auto_aim_interfaces/msg/receive_serial.hpp"
#include "auto_aim_interfaces/msg/nuc.hpp"

//导航的自定义消息类型
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/receive_serial.hpp"

namespace rm_serial_driver
{
    class RMSerialDriver : public rclcpp::Node
    {
    public:
        explicit RMSerialDriver(const rclcpp::NodeOptions & options);

        ~RMSerialDriver() override;

    private:
        void getParams();

        void receiveData();

        void sendData();

        void aimsetData(auto_aim_interfaces::msg::SendSerial msg);

        void navsetData(const geometry_msgs::msg::Twist& cmd_vel);

        void reopenPort();

        void setParam(const rclcpp::Parameter & param);

        void resetTracker();

        void nuchandle(const auto_aim_interfaces::msg::SendSerial msg);

        // Serial port
        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

        // Param client to set detect_colr
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        bool initial_set_param_ = false;
        uint8_t previous_receive_color_ = 0;
        rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
        ResultFuturePtr set_param_future_;

        // Service client to reset tracker
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

        // 声明变量
        visualization_msgs::msg::Marker aiming_point_;
        auto_aim_interfaces::msg::ReceiveSerial aim_receive_serial_msg_;
        rm_decision_interfaces::msg::ReceiveSerial nav_receive_serial_msg_;

        // Broadcast tf from odom to gimbal_link
        double timestamp_offset_ = 0;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::SendSerial>::SharedPtr result_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::SendSerial>::SharedPtr nuc_sub_;

        // 创建发布者
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::ReceiveSerial>::SharedPtr aim_serial_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::ReceiveSerial>::SharedPtr nav_serial_pub_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Nuc>::SharedPtr nuc_pub_;


        std::thread receive_thread_;
        std::thread send_thread_;
        std::chrono::steady_clock::time_point lastTime_;
        int closecount=0;
    };
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
