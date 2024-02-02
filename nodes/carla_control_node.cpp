#include "carla_control/CarlaControl.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scale_truck_control::CarlaControl>());
    rclcpp::shutdown();
    return 0;
}

