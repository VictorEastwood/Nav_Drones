#include <stdint.h>

#include <chrono>
#include <iostream>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros_com/frame_transforms.h>
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("nav_drone_control_srv_node")
  {
    offboard_control_mode_publisher_ =
      this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
      this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
      this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;
    flight_state_ = FlightState::INIT;
    hover_start_time_ = std::chrono::steady_clock::now();

    auto timer_callback = [this]() -> void {
      switch (flight_state_) {
        case FlightState::INIT:
          if (offboard_setpoint_counter_ == 10) {
            // Change to Offboard mode after 10 setpoints
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            // Arm the vehicle
            this->arm();
            flight_state_ = FlightState::TAKEOFF;
            RCLCPP_INFO(this->get_logger(), "切换到起飞模式");
          }
          break;

        case FlightState::TAKEOFF:
          // 继续发送2米高度指令直到稳定
          if (offboard_setpoint_counter_ > 50) { // 5秒后认为已到达目标高度
            flight_state_ = FlightState::HOVER;
            hover_start_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "开始悬停20秒");
          }
          break;

        case FlightState::HOVER:
          {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - hover_start_time_);
            if (elapsed.count() >= 20) {
              flight_state_ = FlightState::LANDING;
              RCLCPP_INFO(this->get_logger(), "开始降落");
            }
          }
          break;

        case FlightState::LANDING:
          // 降落完成后进入等待状态
          if (offboard_setpoint_counter_ > 100) { // 简单的降落时间估计
            flight_state_ = FlightState::WAITING;
            this->disarm();
            RCLCPP_INFO(this->get_logger(), "降落完成，进入等待状态");
          }
          break;

        case FlightState::WAITING:
          // 保持等待状态，不执行任何动作
          return;
      }

      // offboard_control_mode needs to be paired with trajectory_setpoint
      publish_offboard_control_mode();
      publish_trajectory_setpoint();

      offboard_setpoint_counter_++;
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

  void arm();
  void disarm();

private:
  enum class FlightState {
    INIT,
    TAKEOFF,
    HOVER,
    LANDING,
    WAITING
  };

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;

  std::atomic<uint64_t> timestamp_;  //!< common synced timestamped

  uint64_t offboard_setpoint_counter_;  //!< counter for the number of setpoints sent
  FlightState flight_state_;
  std::chrono::steady_clock::time_point hover_start_time_;

  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        根据飞行状态发送相应的轨迹设定点
 */
void OffboardControl::publish_trajectory_setpoint()
{
  TrajectorySetpoint msg{};
  
  switch (flight_state_) {
    case FlightState::INIT:
    case FlightState::TAKEOFF:
    case FlightState::HOVER:
      // 在2米高度悬停
      msg.position = {0.0, 0.0, -2.0};
      msg.yaw = 0.0;
      break;
      
    case FlightState::LANDING:
      // 降落到地面
      msg.position = {0.0, 0.0, 0.0};
      msg.yaw = 0.0;
      break;
      
    case FlightState::WAITING:
      // 保持在地面
      msg.position = {0.0, 0.0, 0.0};
      msg.yaw = 0.0;
      break;
  }
  
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
  std::cout << "Starting offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());

  rclcpp::shutdown();
  return 0;
}
