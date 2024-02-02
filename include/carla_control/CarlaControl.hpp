#pragma once

//C++
/*#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <vector>
#include <sys/time.h>
#include <condition_variable>
#include <functional>
#include <memory>
#include <fstream>
#include <string>
*/
//ROS2
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/int32.hpp"
//#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/AckermannDrive.hpp"

//custom msgs
#include "ros2_msg/msg/lrc2ocr.hpp"
#include "ros2_msg/msg/ocr2lrc.hpp"

using namespace std::chrono_literals;

namespace scale_truck_control{

class CarlaControl : public rclcpp::Node
{
public:
    CarlaControl();

    ~CarlaControl();

private:
    void init();

    //Subscriber
    rclcpp::Subscription<ros2_msg::msg::Lrc2ocr>::SharedPtr LrcSubscriber_;

    //Publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr CarlaPublisher_;

    //Callback
    void LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg);

    float tar_vel_ = 0;
    float steer_angle_ = 0;
    std::mutex data_mutex_;
};

}
