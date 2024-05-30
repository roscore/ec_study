# ec_study

## Usage
```
# 모터 컨트롤 노드 실행
ros2 run ethercat_driver_ros2 motor_control_node

# 토크 온/오프 테스트
source ~/ros2_ws/install/setup.bash
ros2 topic pub /torque_command std_msgs/msg/Bool "{data: true}"  # 토크 온
ros2 topic pub /torque_command std_msgs/msg/Bool "{data: false}"  # 토크 오프


```


