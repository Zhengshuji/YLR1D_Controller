#include "ylr1d_position_simulate/pid.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ylr1d_position_simulate {

/// 单个关节的模拟状态
struct JointSimState {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double desired{0.0};
  PID pid;
  bool initialized{false};
};

/// 控制器组：一组关节输出到同一个 ForwardCommandController topic
struct ControllerGroup {
  std::string topic;
  std::vector<size_t> joint_indices;
};

/// PositionSimulateNode
/// 输入期望关节坐标 -> PID -> 加速度限幅 -> 速度限幅 -> 位置输出
class PositionSimulateNode : public rclcpp::Node {
public:
  PositionSimulateNode() : Node("position_simulate") {
    double loop_hz = declare_parameter("loop_hz", 100.0);
    double kp = declare_parameter("pid/kp", 5.0);
    double ki = declare_parameter("pid/ki", 0.0);
    double kd = declare_parameter("pid/kd", 0.1);
    double max_accel = declare_parameter("pid/max_accel", 50.0);
    double max_vel = declare_parameter("pid/max_vel", 3.0);

    setup_joints(kp, ki, kd, max_accel, max_vel);
    setup_controller_groups();

    desired_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "desired_joint_positions", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        desired_callback(msg);
      });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        init_callback(msg);
      });

    for (auto & g : groups_) {
      g.pub = create_publisher<std_msgs::msg::Float64MultiArray>(g.topic, 10);
    }

    sim_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("simulated_joint_states", 10);

    dt_ = 1.0 / loop_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      [this]() { update(); });

    RCLCPP_INFO(get_logger(), "position_simulate started (%.1f Hz, %zu joints, %zu groups)",
                loop_hz, joints_.size(), groups_.size());
  }

private:
  void setup_joints(double kp, double ki, double kd, double max_accel, double max_vel) {
    const std::vector<std::string> names = {
      // chassis_steering_controller (4)
      "Joint_Base_to_RFWheelF",
      "Joint_Base_to_LFWheelF",
      "Joint_Base_to_RBWheelF",
      "Joint_Base_to_LBWheelF",
      // torso_controller (4)
      "Joint_Base_to_Body1",
      "Joint_Body1_to_Body2",
      "Joint_Body2_to_Body3",
      "Joint_Body3_to_Body4",
      // left_arm_controller (9)
      "Joint_Body2_to_LeftArm1",
      "Joint_LeftArm1_to_LeftArm2",
      "Joint_LeftArm2_to_LeftArm3",
      "Joint_LeftArm3_to_LeftArm4",
      "Joint_LeftArm4_to_LeftArm5",
      "Joint_LeftArm5_to_LeftArm6",
      "Joint_LeftArm6_to_LeftArm7",
      "Joint_LeftArm7_to_LeftFinger1",
      "Joint_LeftArm7_to_LeftFinger2",
      // right_arm_controller (9)
      "Joint_Body2_RightArm1",
      "Joint_RightArm1_to_RightArm2",
      "Joint_RightArm2_to_RightArm3",
      "Joint_RightArm3_to_RightArm4",
      "Joint_RightArm4_to_RightArm5",
      "Joint_RightArm5_to_RightArm6",
      "Joint_RightArm6_to_RightArm7",
      "Joint_RightArm7_to_RightFinger1",
      "Joint_RightArm7_to_RightFinger2",
    };

    joints_.reserve(names.size());
    name_to_idx_.reserve(names.size());
    for (size_t i = 0; i < names.size(); ++i) {
      JointSimState js;
      js.name = names[i];
      js.pid = PID(kp, ki, kd, max_accel, max_vel);
      joints_.push_back(js);
      name_to_idx_[names[i]] = i;
    }
  }

  void setup_controller_groups() {
    groups_.push_back({"/chassis_steering_controller/commands", {0, 1, 2, 3}});
    groups_.push_back({"/torso_controller/commands",           {4, 5, 6, 7}});
    groups_.push_back({"/left_arm_controller/commands",        {8, 9, 10, 11, 12, 13, 14, 15, 16}});
    groups_.push_back({"/right_arm_controller/commands",       {17, 18, 19, 20, 21, 22, 23, 24, 25}});
  }

  /// 从 /joint_states 初始化内部位置（仅一次）
  void init_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    bool need_init = false;
    for (auto & js : joints_) {
      if (!js.initialized) { need_init = true; break; }
    }
    if (!need_init) return;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = name_to_idx_.find(msg->name[i]);
      if (it != name_to_idx_.end()) {
        auto & js = joints_[it->second];
        if (!js.initialized && i < msg->position.size()) {
          double pos = msg->position[i];
          js.position = std::isnan(pos) ? 0.0 : pos;
          js.desired = js.position;
          js.initialized = true;
        }
      }
    }

    bool all_init = true;
    for (auto & js : joints_) {
      if (!js.initialized) { all_init = false; break; }
    }
    if (all_init) {
      RCLCPP_INFO(get_logger(), "All joints initialized from /joint_states");
      joint_state_sub_.reset();
    }
  }

  /// 接收期望关节坐标
  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = name_to_idx_.find(msg->name[i]);
      if (it != name_to_idx_.end()) {
        auto & js = joints_[it->second];
        if (i < msg->position.size()) {
          js.desired = msg->position[i];
        }
      }
    }
  }

  /// 主控制循环：PID -> 加速度限幅 -> 速度 -> 速度限幅 -> 位置
  void update() {
    for (auto & js : joints_) {
      if (!js.initialized) return;
    }

    for (auto & js : joints_) {
      double error = js.desired - js.position;
      double accel = js.pid.compute(error, dt_);
      js.velocity += accel * dt_;
      js.velocity = std::clamp(js.velocity, -js.pid.max_vel_, js.pid.max_vel_);
      js.position += js.velocity * dt_;
    }

    for (auto & g : groups_) {
      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data.reserve(g.joint_indices.size());
      for (size_t idx : g.joint_indices) {
        msg.data.push_back(joints_[idx].position);
      }
      g.pub->publish(msg);
    }

    publish_sim_state();
  }

  void publish_sim_state() {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name.reserve(joints_.size());
    msg.position.reserve(joints_.size());
    msg.velocity.reserve(joints_.size());

    for (auto & js : joints_) {
      msg.name.push_back(js.name);
      msg.position.push_back(js.position);
      msg.velocity.push_back(js.velocity);
      msg.effort.push_back(0.0);
    }
    sim_state_pub_->publish(msg);
  }

  double dt_{0.01};
  std::vector<JointSimState> joints_;
  std::map<std::string, size_t> name_to_idx_;

  struct ControllerGroupPub : ControllerGroup {
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
  };
  std::vector<ControllerGroupPub> groups_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ylr1d_position_simulate

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ylr1d_position_simulate::PositionSimulateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
