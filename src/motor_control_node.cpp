#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ethercat_driver_ros2/ethercat_device.hpp>
#include <ethercat_driver_ros2/ethercat_master.hpp>

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode()
  : Node("motor_control_node")
  {
    // EtherCAT master 초기화
    master_ = std::make_shared<ethercat::EthercatMaster>(this, "config/ethercat_config.yaml");

    // 토크 온/오프 명령을 구독하는 토픽 생성
    torque_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "torque_command", 10, std::bind(&MotorControlNode::torque_command_callback, this, std::placeholders::_1));
    
    // EtherCAT 주기적 작업 타이머 생성
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MotorControlNode::update, this));
  }

private:
  void torque_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Torque ON command received");
      set_torque(true);
    } else {
      RCLCPP_INFO(this->get_logger(), "Torque OFF command received");
      set_torque(false);
    }
  }

  void set_torque(bool on)
  {
    uint16_t controlword = on ? 0x000F : 0x0006;  // Example controlword values for torque on/off
    master_->write_pdo("MotorDriver", 0x6040, 0, controlword);
  }

  void update()
  {
    master_->update();  // EtherCAT 주기적 업데이트
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ethercat::EthercatMaster> master_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}

