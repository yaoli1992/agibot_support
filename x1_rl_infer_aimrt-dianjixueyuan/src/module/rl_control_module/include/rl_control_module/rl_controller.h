#pragma once
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "rl_control_module/control_mode.h"
#include "rl_control_module/rotation_tools.h"
#include "rl_control_module/types.h"
#include "rl_control_module/utilities.h"

#include <fstream>  // 处理文件输入输出的头文件
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "my_ros2_proto/msg/joint_command.hpp"

namespace xyber_x1_infer::rl_control_module {

struct ControlCfg {
  struct WalkStepCfg {
    double action_scale;
    int32_t decimation;
    double cycle_time;
    bool sw_mode;
    double cmd_threshold;
  };

  struct ObsScales {
    double lin_vel;
    double ang_vel;
    double dof_pos;
    double dof_vel;
    double quat;
  };

  struct OnnxCfg {
    std::string policy_file;
    int32_t actions_size;
    int32_t lumbar_size;
    int32_t arm_size;
    int32_t obs_size;
    int32_t num_hist;
    double obs_clip;
    double actions_clip;
  };

  // joint_conf["init_state"/"stiffness"/"damping"][joint_name]
  std::map<std::string, std::map<std::string, double>> joint_conf;
  std::map<std::string, std::map<std::string, double>> joy_conf;

  // in order of yaml declared
  std::vector<std::string> ordered_joint_name;
  WalkStepCfg walk_step_conf;
  ObsScales obs_scales;
  OnnxCfg onnx_conf;
};

struct Proprioception {
  vector_t joint_pos;
  vector_t joint_vel;
  vector3_t base_ang_vel;
  vector3_t base_euler_xyz;
  vector3_t projected_gravity;
};

class RLController {
  using tensor_element_t = float;

 public:
  // RLController() {}
  RLController(const ControlCfg &control_conf, bool use_sim_handles);
  ~RLController() = default;

  void SetMode(const ControlMode control_mode);
  void SetCmdData(const geometry_msgs::msg::Twist joy_data);
  void SetImuData(const sensor_msgs::msg::Imu imu_data);
  void SetJointStateData(const sensor_msgs::msg::JointState joint_state_data);
  ControlMode GetMode();
  bool IsReady();
  void GetJointCmdData(std_msgs::msg::Float64MultiArray &joint_cmd);
  void GetJointCmdData(my_ros2_proto::msg::JointCommand *joint_cmd);
  void start_wave_hand(int mode);
  void jawctrl();

 private:
  bool LoadModel();
  void Update();

  void HandleIdleMode();
  void HandleZeroMode();
  void HandleStandMode();
  void HandleSquatMode();
  void HandleWalkMode();
  void HandleKeepMode();

  void HandleWaveHandMode();
  void wave_hand();
  std::vector<std::vector<double>> readData(const std::string &file_path);

  void UpdateStateEstimation();
  void ComputeObservation();
  double ComputeSwingArm(double phase, double delta_q);
  void ComputeActions();
  void clamping_jaw_ctl();
  void clamping_jaw_ctl(bool mode);

  void push_rod_ctl(); 
 private:
  bool use_sim_handles_;
  int mode_ = 0;
  // from  yaml
   ControlCfg control_conf_;
  vector_t init_joint_angles_;

  // onnx
  std::unique_ptr<Ort::Session> session_ptr_;
  Ort::MemoryInfo memory_info_;
  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;
  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<std::vector<int64_t>> output_shapes_;

  // from ros2 topic
  ControlMode control_mode_;
  geometry_msgs::msg::Twist joy_data_;
  sensor_msgs::msg::Imu imu_data_;
  sensor_msgs::msg::JointState joint_state_data_;
  std::unordered_map<std::string, int32_t> joint_name_index_;

  // compute in algorithm
  std::vector<tensor_element_t> actions_;
  std::vector<tensor_element_t> observations_;
  Proprioception propri_;
  vector_t last_actions_;
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> propri_history_buffer_;

  bool is_first_rec_obs_{true};
  int64_t loop_count_;
  std::vector<digital_lp_filter<double>> low_pass_filters_;
  std::vector<digital_lp_filter<double>> cmd_vel_low_pass_filters_;
  std::vector<digital_lp_filter<double>> imu_angle_vel_filters_;
  std::vector<digital_lp_filter<double>> imu_line_acc_filters_;
  std::vector<digital_lp_filter<double>> joint_vel_filters_;


  // output
  std_msgs::msg::Float64MultiArray sim_joint_cmd_;
  my_ros2_proto::msg::JointCommand real_joint_cmd_;

  // PD stand
  double trans_mode_percent_ = 0.0;
  double trans_mode_duration_ms_ = 2000.0;
  vector_t current_joint_angles_;
  vector_t last_tg_joint_angles_;
  vector_t move_start_joint_angles_;
  vector_t current_joint_effort_;

  std::vector<std::vector<std::vector<double>>> wave_hand_txt_data_;
  std::vector<std::vector<double>> wave_hand_data_;

  // std::vector<std::vector<double>> wave_hand_data_banner1;
  // std::vector<std::vector<double>> wave_hand_data_banner2;
  // std::vector<std::vector<double>> wave_hand_data_lantern;
  // std::vector<std::vector<double>> wave_hand_data_fu;
  // std::vector<std::vector<double>> wave_hand_data_snake;
  // std::vector<std::vector<double>> wave_hand_data_red_envelope;

  int wave_hand_step_ = 0;
  std::vector<int>wave_hand_type_;


  int wave_hand_data_mode_ = 0;
  int last_wave_hand_data_mode_ = 0;
  int wave_hand_ctl_num_ = 0;
  bool to_first_pos_ok = false;
  bool first_wave_hand_ = true;
  bool start_wave_hand_ = false;
  double wave_hand_percent_ = 0;

  double ctl_cycle = 0;
  geometry_msgs::msg::Twist joy_data_filter;
  double max_lin_vel_x = 0.6;
  double max_lin_vel_y = 0.4;
  double max_angle_vel_z = 0.4;

  double left_elbow_pitch = 0.0;
  double right_elbow_pitch = 0.0;
  bool get_wave_hand_data_ = true;
  bool ctrl_clamping_jaw_ = false;
  bool hand_open_ = true;
};

}  // namespace xyber_x1_infer::rl_control_module
