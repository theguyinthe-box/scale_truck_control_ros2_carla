gg#include "carla_control/CarlaControl.hpp"

using namespace std::chrono_literals;

namespace scale_truck_control{

CarlaControl::CarlaControl(void) 
  : Node("carla_control", rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
{
  init();

}

CarlaControl::~CarlaControl(){
}

void CarlaControl::init(){
  /******************************/
  /* ROS Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("OcrSub/lrc_to_ocr/topic", LrcSubTopicName, std::string("lrc2ocr_msg"));
  this->get_parameter_or("OcrSub/lrc_to_ocr/queue_size", LrcSubQueueSize, 1);

  /******************************/
  /* ROS Topic Publish Option */
  /******************************/
  this->get_parameter_or("OcrPub/ocr_to_carla/topic", CarlaPubTopicName, std::string("carla_ackermann_msg"));
  this->get_parameter_or("OcrPub/ocr_to_carla/queue_size", CarlaPubQueueSize, 1);

  /************************/
  /* ROS Topic Subscriber */
  /************************/
  LrcSubscriber_ = this->create_subscription<ros2_msg::msg::Lrc2ocr>(LrcSubTopicName, LrcSubQueueSize, std::bind(&CarlaControl::LrcCallback, this, std::placeholders::_1));

  /************************/
  /* ROS Topic Publisher */
  /************************/
  CarlaPublisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(CarlaPubTopicName, CarlaPubQueueSize);  

}

void CarlaControl::LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg)   
{
  std::scoped_lock lock(data_mutex_);
  tar_vel_ = msg->tar_vel;
  steer_angle_ = msg->steer_angle;

  ackermann_msgs::msg::AckermannDrive carla_msg;
  carla_msg.steering_angle = steer_angle_;
  carla_msg.steer.steering_angle_velocity = 0;
  carla_msg.speed = tar_vel_;
  carla_msg.acceleration = 0;
  carla_msg.jerk = 0;

  CarlaPublisher_->publish(carla_msg);
}



} /* namespace scale_truck_control */

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scale_truck_control::CarlaControl>());
    rclcpp::shutdown();
    return 0;
}

