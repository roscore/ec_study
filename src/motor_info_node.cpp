#include <rclcpp/rclcpp.hpp>
#include <ethercat_driver_ros2/ethercat_master.hpp>

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode()
  : Node("motor_control_node")
  {
    master_ = std::make_shared<ethercat::EthercatMaster>(this, "config/ethercat_config.yaml");

    // 주기적으로 slave 정보를 확인하기 위한 타이머 설정
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MotorControlNode::check_slaves, this));
  }

private:
  void check_slaves()
  {
    RCLCPP_INFO(this->get_logger(), "EtherCAT slave 정보를 확인 중...");
    auto slaves_info = master_->get_slave_info();
    for (const auto& slave : slaves_info) {
      RCLCPP_INFO(this->get_logger(), "Slave ID: %d, Name: %s, State: %s",
                  slave.id, slave.name.c_str(), slave.state.c_str());
    }
  }

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

