#include "rl_control_module/rl_controller.h"
#include <chrono>
#include <iostream>
#include <filesystem>
using namespace std::chrono;
namespace xyber_x1_infer::rl_control_module {

RLController::RLController(const ControlCfg& control_conf, bool use_sim_handles)
    : control_conf_(control_conf),
      use_sim_handles_(use_sim_handles),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {

  LoadModel();
  wave_hand_txt_data_.resize(4);
  std::filesystem::path currentPath = std::filesystem::current_path();
  std::cout << "workspace: " << currentPath << std::endl;


  wave_hand_txt_data_[0] = readData(currentPath.string()+"/data/grasp.txt");  
  wave_hand_txt_data_[1] = readData(currentPath.string()+"/data/shake_hand.txt");
  wave_hand_txt_data_[2] = readData(currentPath.string()+"/data/waist_wave_hand.txt");
  // wave_hand_txt_data_[3] = readData(currentPath.string()+"/data/grasp.txt");

  wave_hand_data_ = wave_hand_txt_data_[0];


  wave_hand_type_.resize(4);
  wave_hand_type_[0] = 1;
  wave_hand_type_[1] = 1;
  wave_hand_type_[2] = 0;

  for (int i = 0; i < 3;i++){
    if (wave_hand_txt_data_[i].size() < 10)
    {
      get_wave_hand_data_ = false;
    }
  }
  std::cerr << "get_wave_hand_data_ = " << get_wave_hand_data_ << std::endl;
  // wave_hand_type_[3] = 0;
  //   for (const auto& row : wave_hand_data_) {
  //   for (const auto& val : row) {
  //     std::cout << val << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << " INIT ###########################" << std::endl;
  // std::cout << "control_conf_.ordered_joint_name.size() = " <<
  // control_conf_.ordered_joint_name.size() << std::endl; std::cout <<
  // "control_conf_.onnx_conf.arm_size = " << control_conf_.onnx_conf.arm_size << std::endl;
  // std::cout << "control_conf_.onnx_conf.lumbar_size = " << control_conf_.onnx_conf.lumbar_size <<std::endl;

  actions_.resize(control_conf_.onnx_conf.actions_size);
  observations_.resize(control_conf_.onnx_conf.obs_size * control_conf_.onnx_conf.num_hist);
  last_actions_.resize(control_conf_.onnx_conf.actions_size);
  last_actions_.setZero();

  propri_history_buffer_.resize(control_conf_.onnx_conf.obs_size *
                                control_conf_.onnx_conf.num_hist);

  // init_joint_angles_     current_joint_angles_    fix
  init_joint_angles_.resize(control_conf_.onnx_conf.actions_size);
  int index = 0;
  for (size_t i = 0; i < control_conf_.onnx_conf.actions_size; ++i) {
    if (i < 6)
    {
      index = i;
    }else if (i == 6)
    {
      index = 15;
    }else if (i < 13)
    {
      index = i - 1;
    }else if(i == 13){
      index = 23;
    }else{
      std::cerr << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;
      index = 0;
    }

    init_joint_angles_(i) =
        control_conf_.joint_conf["init_state"][control_conf_.ordered_joint_name[index]];
  }

  current_joint_angles_.resize(control_conf_.ordered_joint_name.size());
  last_tg_joint_angles_.resize(control_conf_.ordered_joint_name.size());
  move_start_joint_angles_.resize(control_conf_.ordered_joint_name.size());
  current_joint_effort_.resize(control_conf_.ordered_joint_name.size());
  if (use_sim_handles_) {
    control_mode_ = ControlMode::ZERO;
  } else {
    control_mode_ = ControlMode::IDLE;
  }
  joint_name_index_.clear();
  loop_count_ = 0;
  low_pass_filters_.clear();
  for (size_t i = 0; i < control_conf_.onnx_conf.actions_size; ++i) {
    low_pass_filters_.emplace_back(100, 0.001);
  }
  imu_angle_vel_filters_.clear();
  for (size_t i = 0; i < 3; ++i) {
    imu_angle_vel_filters_.emplace_back(100, 1);
    imu_angle_vel_filters_[i].init(0);
  }

  imu_line_acc_filters_.clear();
  for (size_t i = 0; i < 3; ++i) {
    imu_line_acc_filters_.emplace_back(100, 1);
    imu_line_acc_filters_[i].init(0);

  }

  joint_vel_filters_.clear();
  for (size_t i = 0; i < 30; ++i) {
    joint_vel_filters_.emplace_back(100, 1);
    joint_vel_filters_[i].init(0);
  }

  cmd_vel_low_pass_filters_.clear();
  for (size_t i = 0; i < 3; ++i) {
    cmd_vel_low_pass_filters_.emplace_back(100, 0.01);
    cmd_vel_low_pass_filters_[i].init(0);
  }
  sim_joint_cmd_.data.resize(control_conf_.ordered_joint_name.size());

  // fix
  real_joint_cmd_.name = control_conf_.ordered_joint_name;
  real_joint_cmd_.position.resize(control_conf_.ordered_joint_name.size());
  real_joint_cmd_.velocity.resize(control_conf_.ordered_joint_name.size());
  real_joint_cmd_.effort.resize(control_conf_.ordered_joint_name.size());
  real_joint_cmd_.damping.resize(control_conf_.ordered_joint_name.size());
  real_joint_cmd_.damping.resize(control_conf_.ordered_joint_name.size());
  real_joint_cmd_.stiffness.resize(control_conf_.ordered_joint_name.size());
  ctl_cycle = control_conf_.walk_step_conf.cycle_time;
  max_lin_vel_x = control_conf_.joy_conf["cmd_vel"]["max_line_vel_x"];
  max_lin_vel_y = control_conf_.joy_conf["cmd_vel"]["max_line_vel_y"];
  max_angle_vel_z = control_conf_.joy_conf["cmd_vel"]["max_angle_vel_z"];
  // std::cerr << "!!!!!!!!!!!!!"<<max_lin_vel_x << " " << max_lin_vel_y << " " << max_angle_vel_z <<std::endl;
}

void RLController::SetMode(const ControlMode control_mode) {
  // std::cout << " SetMode ###########################" << std::endl;
  control_mode_ = control_mode;
  trans_mode_percent_ = 0.0;
  wave_hand_percent_ = 0.0;
  wave_hand_ctl_num_ = 0;
  to_first_pos_ok = false;
  for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
    current_joint_angles_(j) =
        joint_state_data_.position[joint_name_index_[control_conf_.ordered_joint_name[j]]];
  }
  for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
    current_joint_effort_(j) =
        joint_state_data_.effort[joint_name_index_[control_conf_.ordered_joint_name[j]]];
  }
  start_wave_hand_ = false;
  first_wave_hand_ = true;

}

void RLController::SetCmdData(const geometry_msgs::msg::Twist joy_data) { joy_data_ = joy_data; }

void RLController::SetImuData(const sensor_msgs::msg::Imu imu_data) { imu_data_ = imu_data; }

void RLController::SetJointStateData(const sensor_msgs::msg::JointState joint_state_data) {
  joint_state_data_ = joint_state_data;

  if (joint_name_index_.empty()) {
    for (size_t i = 0; i < joint_state_data_.name.size(); ++i) {
      joint_name_index_[joint_state_data_.name[i]] = i;
    }
  }

  // int left_ankle_index = joint_name_index_["left_ankle_pitch_joint"];
  // joint_state_data_.position[left_ankle_index] += -0.04;
  // int right_ankle_index = joint_name_index_["right_ankle_pitch_joint"];
  // joint_state_data_.position[right_ankle_index] += -0.04;

  // int right_hip_index = joint_name_index_["right_hip_roll_joint"];
  // joint_state_data_.position[right_hip_index] += -0.03;
  // int left_hip_index = joint_name_index_["left_hip_roll_joint"];
  // joint_state_data_.position[left_hip_index] += 0.04;
}

ControlMode RLController::GetMode() { return control_mode_; }

bool RLController::IsReady() {
  if (joint_name_index_.empty()) {
    return false;
  }
  return true;
}
void RLController::GetJointCmdData(std_msgs::msg::Float64MultiArray& joint_cmd) {
  Update();
  joint_cmd = sim_joint_cmd_;
}

void RLController::GetJointCmdData(my_ros2_proto::msg::JointCommand *joint_cmd) {
  Update();
  *joint_cmd = real_joint_cmd_;

  // int left_ankle_index = joint_name_index_["left_ankle_pitch_joint"];
  // joint_cmd->position[left_ankle_index] += 0.04;
  // int right_ankle_index = joint_name_index_["right_ankle_pitch_joint"];
  // joint_cmd->position[right_ankle_index] += 0.04;

  // int right_hip_index = joint_name_index_["right_hip_roll_joint"];
  // joint_cmd->position[right_hip_index] += 0.03;
  // int left_hip_index = joint_name_index_["left_hip_roll_joint"];
  // joint_cmd->position[left_hip_index] += -0.04;


}

bool RLController::LoadModel() {
  std::cout << " LoadModel ###########################" << std::endl;
  // create env
  std::shared_ptr<Ort::Env> onnxEnvPrt(
      new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetInterOpNumThreads(1);
  session_ptr_ = std::make_unique<Ort::Session>(
      *onnxEnvPrt, control_conf_.onnx_conf.policy_file.c_str(), sessionOptions);  // TODO
  std::cout << control_conf_.onnx_conf.policy_file.c_str() << std::endl;
  // get input and output info
  input_names_.clear();
  output_names_.clear();
  input_shapes_.clear();
  output_shapes_.clear();
  Ort::AllocatorWithDefaultOptions allocator;

  for (size_t i = 0; i < session_ptr_->GetInputCount(); ++i) {
    char* tempstring =
        new char[strlen(session_ptr_->GetInputNameAllocated(i, allocator).get()) + 1];
    strcpy(tempstring, session_ptr_->GetInputNameAllocated(i, allocator).get());
    input_names_.push_back(tempstring);
    input_shapes_.push_back(
        session_ptr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "Output count: " <<
  // session_ptr_->GetOutputCount());
  for (size_t i = 0; i < session_ptr_->GetOutputCount(); ++i) {
    char* tempstring =
        new char[strlen(session_ptr_->GetOutputNameAllocated(i, allocator).get()) + 1];
    strcpy(tempstring, session_ptr_->GetOutputNameAllocated(i, allocator).get());
    output_names_.push_back(tempstring);
    output_shapes_.push_back(
        session_ptr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }

  // for (const char* name : input_names_) {
  //   // RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "Input Names: " << name);
  // }
  // for (const char* name : output_names_) {
  //   // RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "Output Names: " << name);
  // }
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rlccpp"), "Load Onnx model successfully !!!");
  // printf("Load Onnx model successfully !!!\n");
  // std::cout << "Load Onnx model successfully !!!" << std::endl;
  return true;
}

void RLController::Update() {
  UpdateStateEstimation();

  switch (control_mode_) {
    case ControlMode::IDLE:
      HandleIdleMode();
      break;
    case ControlMode::ZERO:
      HandleZeroMode();
      break;
    case ControlMode::STAND:
      HandleStandMode();
      break;
    case ControlMode::WALK:
      HandleWalkMode();
      break;
    case ControlMode::KEEP:
      HandleKeepMode();
      break;
    case ControlMode::SQUAT:
      HandleSquatMode();
      break;
    default:
      break;
  }
}
void RLController::jawctrl(){
  ctrl_clamping_jaw_ = true;
  std::cerr << "ctrl_clamping_jaw_ = true" << std::endl; 
}

void RLController::push_rod_ctl() {
    real_joint_cmd_.position[28] =2;
    real_joint_cmd_.velocity[28] = 0.0;
    real_joint_cmd_.effort[28] = 0.0;
    real_joint_cmd_.stiffness[28] = 0;
    real_joint_cmd_.damping[28] = 0;


    real_joint_cmd_.position[29] =2;
    real_joint_cmd_.velocity[29] = 0.0;
    real_joint_cmd_.effort[29] = 0.0;
    real_joint_cmd_.stiffness[29] = 0;
    real_joint_cmd_.damping[29] = 0;


    real_joint_cmd_.position[20] =2;
    real_joint_cmd_.velocity[20] = 0.0;
    real_joint_cmd_.effort[20] = 0.0;
    real_joint_cmd_.stiffness[20] = 0;
    real_joint_cmd_.damping[20] = 0;


    real_joint_cmd_.position[21] =2;
    real_joint_cmd_.velocity[21] = 0.0;
    real_joint_cmd_.effort[21] = 0.0;
    real_joint_cmd_.stiffness[21] = 0;
    real_joint_cmd_.damping[21] = 0;
}
void RLController::clamping_jaw_ctl(bool mode){
    if (mode == true)
    {
      real_joint_cmd_.position[22] =1;
      real_joint_cmd_.velocity[22] = 0.0;
      real_joint_cmd_.effort[22] = 1.0;
      real_joint_cmd_.stiffness[22] = 0;
      real_joint_cmd_.damping[22] = 0;
      real_joint_cmd_.position[30] =1;
      real_joint_cmd_.velocity[30] = 0.0;
      real_joint_cmd_.effort[30] = 1.0;
      real_joint_cmd_.stiffness[30] = 0;
      real_joint_cmd_.damping[30] = 0;
    }
    else
    {
      real_joint_cmd_.position[22] =0.0;
      real_joint_cmd_.velocity[22] = 0.0;
      real_joint_cmd_.effort[22] = 1.0;
      real_joint_cmd_.stiffness[22] = 0;
      real_joint_cmd_.damping[22] = 0;
      real_joint_cmd_.position[30] = 0.02;
      real_joint_cmd_.velocity[30] = 0.0;
      real_joint_cmd_.effort[30] = 1.0;
      real_joint_cmd_.stiffness[30] = 0;
      real_joint_cmd_.damping[30] = 0;
    }
}
void RLController::clamping_jaw_ctl() {
  if (ctrl_clamping_jaw_)
  {
    std::cerr << current_joint_angles_[22] << std::endl;

    if (hand_open_)
    {
      real_joint_cmd_.position[22] =1;
      real_joint_cmd_.velocity[22] = 0.0;
      real_joint_cmd_.effort[22] = 1.0;
      real_joint_cmd_.stiffness[22] = 0;
      real_joint_cmd_.damping[22] = 0;
      real_joint_cmd_.position[30] =1;
      real_joint_cmd_.velocity[30] = 0.0;
      real_joint_cmd_.effort[30] = 1.0;
      real_joint_cmd_.stiffness[30] = 0;
      real_joint_cmd_.damping[30] = 0;
      std::cerr << "clamping_jaw_ctl_open" << std::endl;
      hand_open_ = false;
    }
    else
    { 
      real_joint_cmd_.position[22] =0.0;
      real_joint_cmd_.velocity[22] = 0.0;
      real_joint_cmd_.effort[22] = 1.0;
      real_joint_cmd_.stiffness[22] = 0;
      real_joint_cmd_.damping[22] = 0; 
      real_joint_cmd_.position[30] = 0.02;
      real_joint_cmd_.velocity[30] = 0.0;
      real_joint_cmd_.effort[30] = 1.0;
      real_joint_cmd_.stiffness[30] = 0;
      real_joint_cmd_.damping[30] = 0;
      hand_open_ = true;
      std::cerr << "clamping_jaw_ctl_closes" << std::endl;
    }
    ctrl_clamping_jaw_ = false;
  }
}
void RLController::HandleIdleMode() {
  // std::cout << " HandleIdleMode ###########################" << std::endl;
  if (use_sim_handles_) {
    sim_joint_cmd_.data = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);
  } else {
    // std::cout << " HandleIdleMode  ::::control_conf_.ordered_joint_name.size() = " <<
    // control_conf_.ordered_joint_name.size() << std::endl;

    // set leg + lumbar

    real_joint_cmd_.position = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);
    real_joint_cmd_.velocity = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);
    real_joint_cmd_.effort = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);
    real_joint_cmd_.damping = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);
    real_joint_cmd_.damping = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);
    real_joint_cmd_.stiffness = std::vector<double>(control_conf_.ordered_joint_name.size(), 0.0);

    // real_joint_cmd_.effort[22] = 1.0;
    // real_joint_cmd_.effort[30] = 1.0;

    
    // for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
    //   std::cerr << j << " real_joint_cmd_.position " << real_joint_cmd_.position[j] << std::endl; 
    //   std::cerr << j << " real_joint_cmd_.velocity " << real_joint_cmd_.velocity[j] << std::endl; 
    //   std::cerr << j << " real_joint_cmd_.effort " << real_joint_cmd_.effort[j] << std::endl; 
    //   std::cerr << j << " real_joint_cmd_.stiffness " << real_joint_cmd_.stiffness[j] << std::endl; 
    //   std::cerr << j << " real_joint_cmd_.damping " << real_joint_cmd_.damping[j] << std::endl; 
    //   std::cerr << j << " real_joint_cmd_.name " << real_joint_cmd_.name[j] << std::endl; 
    // }
  }
}

void RLController::HandleZeroMode() {
  // std::cout << "control_conf_.ordered_joint_name.size() = " <<
  // control_conf_.ordered_joint_name.size() << std::endl;
  // std::cerr << "zero before " << real_joint_cmd_.name[15] << " " << real_joint_cmd_.name[16] << std::endl; 
  static bool flag = true;
  if (flag) {
    for (int i = 0; i < 25; i++) {
      std::cout << current_joint_angles_(i);
    }
    std::cout << std::endl;
    flag = false;
  }

  if (trans_mode_percent_ <= 1) {
    // set leg + lumbar + arm
    for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
      int index = joint_name_index_[control_conf_.ordered_joint_name[j]];

      // pos_des
      double pos_des = current_joint_angles_(j);
      pos_des = current_joint_angles_(j) * (1 - trans_mode_percent_);
      std::string joint_name = control_conf_.ordered_joint_name[j];
      double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
      double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];

      if (j == 4 || j == 5 || j == 10 || j == 11) {
        // // sim
        sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                 0.0 * (0.0 - joint_state_data_.velocity[index]);
        // real
        real_joint_cmd_.position[j] = pos_des;
        real_joint_cmd_.velocity[j] = 0.0;
        real_joint_cmd_.effort[j] = 0.0;
        real_joint_cmd_.stiffness[j] = stand_stiffness;
        real_joint_cmd_.damping[j] = stand_damping;
      } 
      // else if (j == 20 || j == 21 || j == 28 || j == 29){
      //   real_joint_cmd_.position[j] = 0;
      //   real_joint_cmd_.velocity[j] = 0.0;
      //   real_joint_cmd_.effort[j] = 0.0;
      //   real_joint_cmd_.stiffness[j] = 0;
      //   real_joint_cmd_.damping[j] = 0;
      // }else if (j == 22 || j == 30){
      //   real_joint_cmd_.position[j] = 0;
      //   real_joint_cmd_.velocity[j] = 0.0;
      //   real_joint_cmd_.effort[j] = 1.0;
      //   real_joint_cmd_.stiffness[j] = 0;
      //   real_joint_cmd_.damping[j] = 0;
      // }
      else {
        sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                 0.5 * (0.0 - joint_state_data_.velocity[index]);

        // real
        real_joint_cmd_.position[j] = pos_des;
        real_joint_cmd_.velocity[j] = 0.0;
        real_joint_cmd_.effort[j] = 0.0;
        real_joint_cmd_.stiffness[j] = stand_stiffness;
        real_joint_cmd_.damping[j] = stand_damping;
      }
      // std::cerr << j << " real_joint_cmd_.name " << real_joint_cmd_.name[j] << std::endl; 
    }

    trans_mode_percent_ += 1 / trans_mode_duration_ms_;
    trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
  }
}

void RLController::HandleStandMode() {
  // set leg  + lumbar + arm
  if (trans_mode_percent_ <= 1) {
    for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
      int index = joint_name_index_[control_conf_.ordered_joint_name[j]];
      double pos_des =
          current_joint_angles_(j) * (1 - trans_mode_percent_) +
          trans_mode_percent_ *
              control_conf_.joint_conf["init_state"][control_conf_.ordered_joint_name[j]];

      double effort_des = current_joint_effort_(j) * (1 - trans_mode_percent_);

      std::string joint_name = control_conf_.ordered_joint_name[j];
      double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
      double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
      if (j == 4 || j == 5 || j == 10 || j == 11) {
        // sim
        sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                 0.0 * (0.0 - joint_state_data_.velocity[index]);
        // real
        real_joint_cmd_.position[j] = pos_des;
        real_joint_cmd_.velocity[j] = 0.0;
        real_joint_cmd_.effort[j] = 0.0;
        real_joint_cmd_.stiffness[j] = stand_stiffness;
        real_joint_cmd_.damping[j] = stand_damping;
      } 
      else {
        // if 
        if (((start_wave_hand_) &&  (j>=12))|| (j == 22 || j ==30 || j == 28 || j == 29 || j == 20 || j == 21)){
        }
        else{
          sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                  0.5 * (0.0 - joint_state_data_.velocity[index]);

          // real
          // std::cerr << "j = " << j << std::endl;
          real_joint_cmd_.position[j] = pos_des;
          real_joint_cmd_.velocity[j] = 0.0;
          real_joint_cmd_.effort[j] = effort_des;
          real_joint_cmd_.stiffness[j] = stand_stiffness;
          real_joint_cmd_.damping[j] = stand_damping;
        }
      }
    }
    trans_mode_percent_ += 1 / trans_mode_duration_ms_;
    trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
  }

  if (start_wave_hand_) {
    wave_hand();
  }
  clamping_jaw_ctl();
  push_rod_ctl();
  loop_count_ = 0;
}


void RLController::HandleSquatMode() {
  // set leg  + lumbar + arm
  if (trans_mode_percent_ <= 1) {
    for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
      int index = joint_name_index_[control_conf_.ordered_joint_name[j]];
      
      double pos_des =
          current_joint_angles_(j) * (1 - trans_mode_percent_) +
          trans_mode_percent_ *
              control_conf_.joint_conf["squat_state"][control_conf_.ordered_joint_name[j]];

      double effort_des = current_joint_effort_(j) * (1 - trans_mode_percent_);

      std::string joint_name = control_conf_.ordered_joint_name[j];
      double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
      double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
      if (j == 4 || j == 5 || j == 10 || j == 11) {
        // sim
        sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                 0.0 * (0.0 - joint_state_data_.velocity[index]);
        // real
        real_joint_cmd_.position[j] = pos_des;
        real_joint_cmd_.velocity[j] = 0.0;
        real_joint_cmd_.effort[j] = 0.0;
        real_joint_cmd_.stiffness[j] = stand_stiffness;
        real_joint_cmd_.damping[j] = stand_damping;
      } 
      // else if (j == 20 || j == 21 || j == 28 || j == 29){
      //   real_joint_cmd_.position[j] = 1;
      //   real_joint_cmd_.velocity[j] = 0.0;
      //   real_joint_cmd_.effort[j] = 0.0;
      //   real_joint_cmd_.stiffness[j] = 0;
      //   real_joint_cmd_.damping[j] = 0;
      // }
      // else if (j == 22 || j == 30){
      //   real_joint_cmd_.position[j] = 1;
      //   real_joint_cmd_.velocity[j] = 0.0;
      //   real_joint_cmd_.effort[j] = 1.0;
      //   real_joint_cmd_.stiffness[j] = 0;
      //   real_joint_cmd_.damping[j] = 0;
      // }
      else {
        sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                 0.5 * (0.0 - joint_state_data_.velocity[index]);
        // real
        real_joint_cmd_.position[j] = pos_des;
        real_joint_cmd_.velocity[j] = 0.0;
        real_joint_cmd_.effort[j] = effort_des;
        real_joint_cmd_.stiffness[j] = stand_stiffness;
        real_joint_cmd_.damping[j] = stand_damping;
      }
    }
    trans_mode_percent_ += 1 / (trans_mode_duration_ms_ * 1.0);
    trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
  }


}

void RLController::HandleKeepMode() {
  // std::cout << "control_conf_.ordered_joint_name.size() = " <<
  // control_conf_.ordered_joint_name.size() << std::endl;
  static bool flag = true;
  if (flag) {
    for (int i = 0; i < 25; i++) {
      std::cout << current_joint_angles_(i);
    }
    std::cout << std::endl;
    flag = false;
  }

  if (trans_mode_percent_ <= 1) {
    // set leg + lumbar + arm
    for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
      int index = joint_name_index_[control_conf_.ordered_joint_name[j]];

      // pos_des
      double pos_des = current_joint_angles_(j);
      // pos_des = current_joint_angles_(j) * (1 - trans_mode_percent_);
      if(j == 3 || j == 9){
        pos_des = current_joint_angles_(j) - 0.03;
      }
      
      // if(j == 1){
      //   pos_des = current_joint_angles_(j) - 0.1;
      // }
      // if(j == 7){
      //   // pos_des = current_joint_angles_(j) - 0.05;
      // }

      std::string joint_name = control_conf_.ordered_joint_name[j];
      double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
      double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];

      if (j == 4 || j == 5 || j == 10 || j == 11) {
        // sim
        sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                 0.0 * (0.0 - joint_state_data_.velocity[index]);
        // real
        real_joint_cmd_.position[j] = pos_des;
        real_joint_cmd_.velocity[j] = 0.0;
        real_joint_cmd_.effort[j] = 0.0;
        real_joint_cmd_.stiffness[j] = stand_stiffness;
        real_joint_cmd_.damping[j] = stand_damping;
      } else {
          if ((start_wave_hand_) &&  (j>=15)){
          }
          else{
            sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                  0.5 * (0.0 - joint_state_data_.velocity[index]);
          // real
            real_joint_cmd_.position[j] = pos_des;
            real_joint_cmd_.velocity[j] = 0.0;
            real_joint_cmd_.effort[j] = 0.0;
            real_joint_cmd_.stiffness[j] = stand_stiffness;
            real_joint_cmd_.damping[j] = stand_damping;
          }
       }
      }
    }
    trans_mode_percent_ += 1 / trans_mode_duration_ms_;
    trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
  
  if (start_wave_hand_) {
    // wave_hand();
  }

}

void RLController::wave_hand() {
  double pos_des = 0;

  if (!to_first_pos_ok) {
    for (int i = 12; i <= 30; i++) {
      if (i>=12 &&  i <= 14){
        std::string joint_name = control_conf_.ordered_joint_name[i];
        double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
        double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
        pos_des = move_start_joint_angles_(i) * (1 - wave_hand_percent_) +
                  wave_hand_percent_ * wave_hand_data_[0][i-12];
        last_tg_joint_angles_[i] =  pos_des;               
        real_joint_cmd_.position[i] = pos_des;
        real_joint_cmd_.velocity[i] = 0.0;
        real_joint_cmd_.effort[i] = 0.0;
        // real_joint_cmd_.stiffness[i] = 0;  // stand_stiffness
        // real_joint_cmd_.damping[i] = 0;
        real_joint_cmd_.stiffness[i] = stand_stiffness;  // stand_stiffness
        real_joint_cmd_.damping[i] = stand_damping;
      }
      else if (i >=15 &&  i <= 19){
        std::string joint_name = control_conf_.ordered_joint_name[i];
        double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
        double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
        pos_des = move_start_joint_angles_(i) * (1 - wave_hand_percent_) +
                  wave_hand_percent_ * wave_hand_data_[0][i - 15+3];
        last_tg_joint_angles_[i] =  pos_des;               
        real_joint_cmd_.position[i] = pos_des;
        real_joint_cmd_.velocity[i] = 0.0;
        real_joint_cmd_.effort[i] = 0.0;
        // real_joint_cmd_.stiffness[i] = 0;  // stand_stiffness
        // real_joint_cmd_.damping[i] = 0;
        real_joint_cmd_.stiffness[i] = stand_stiffness;  // stand_stiffness
        real_joint_cmd_.damping[i] = stand_damping;
      }
      else if (i >=23 &&  i <= 27){
        std::string joint_name = control_conf_.ordered_joint_name[i];
        double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
        double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
        pos_des = move_start_joint_angles_(i) * (1 - wave_hand_percent_) +
                  wave_hand_percent_ * wave_hand_data_[0][i - 23 + 5+3];
        last_tg_joint_angles_[i] =  pos_des;        
        
        real_joint_cmd_.position[i] = pos_des;
        real_joint_cmd_.velocity[i] = 0.0;
        real_joint_cmd_.effort[i] = 0.0;
        // real_joint_cmd_.stiffness[i] = 0;  // stand_stiffness
        // real_joint_cmd_.damping[i] = 0;
        real_joint_cmd_.stiffness[i] = stand_stiffness;  // stand_stiffness
        real_joint_cmd_.damping[i] = stand_damping;
      }
    }

    wave_hand_percent_ += 1 / 2000.0;
    if (wave_hand_percent_ > 1) {
      wave_hand_percent_ = 1;
      to_first_pos_ok = true;
    }
    wave_hand_percent_ = std::min(wave_hand_percent_, scalar_t(1));
  } else if (wave_hand_step_ == 2){
    if (wave_hand_ctl_num_ < wave_hand_data_.size() - 5) {
      for (size_t j = 12; j <= 30; ++j) {
        if (j >=12 &&  j <= 14){
          std::string joint_name = control_conf_.ordered_joint_name[j];
          double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
          double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
          pos_des = wave_hand_data_[wave_hand_ctl_num_][j - 12];
          last_tg_joint_angles_[j] =  pos_des;        

          real_joint_cmd_.position[j] = pos_des;
          real_joint_cmd_.velocity[j] = 0.0;
          real_joint_cmd_.effort[j] = 0.0;
          real_joint_cmd_.stiffness[j] = stand_stiffness;  // 80
          real_joint_cmd_.damping[j] = stand_damping;
        }
        else if (j >=15 &&  j <= 19){
          std::string joint_name = control_conf_.ordered_joint_name[j];
          double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
          double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
          pos_des = wave_hand_data_[wave_hand_ctl_num_][j - 15+3];
          last_tg_joint_angles_[j] =  pos_des;        

          real_joint_cmd_.position[j] = pos_des;
          real_joint_cmd_.velocity[j] = 0.0;
          real_joint_cmd_.effort[j] = 0.0;
          real_joint_cmd_.stiffness[j] = stand_stiffness;  // 80
          real_joint_cmd_.damping[j] = stand_damping;
        }
        else if (j >=23 &&  j <= 27){
          std::string joint_name = control_conf_.ordered_joint_name[j];
          double stand_stiffness = control_conf_.joint_conf["stand_stiffness"][joint_name];
          double stand_damping = control_conf_.joint_conf["stand_damping"][joint_name];
          pos_des = wave_hand_data_[wave_hand_ctl_num_][j - 23+5+3];
          last_tg_joint_angles_[j] =  pos_des;        
          real_joint_cmd_.position[j] = pos_des;
          real_joint_cmd_.velocity[j] = 0.0;
          real_joint_cmd_.effort[j] = 0.0;
          real_joint_cmd_.stiffness[j] = stand_stiffness;  // 80
          real_joint_cmd_.damping[j] = stand_damping;
        }
      }
    }
    else{
      start_wave_hand_ = false;
      std::cerr << "stop wave_hand"<<std::endl;
    }
    std::cerr << real_joint_cmd_.position[23] <<std::endl;
    wave_hand_ctl_num_++;
  }
}
// void RLController::grasp(){



// }
// void RLController::move_box(){


// }
void RLController::HandleWaveHandMode() {
  double pos_des;
  if (!to_first_pos_ok) {
    if (trans_mode_percent_ <= 1) {
      for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
        int index = joint_name_index_[control_conf_.ordered_joint_name[j]];
        if (index >= 20 && index <= 24) {
          pos_des = current_joint_angles_(j) * (1 - trans_mode_percent_) +
                    trans_mode_percent_ * wave_hand_data_[0][index - 20];
          real_joint_cmd_.position[j] = pos_des;
          real_joint_cmd_.velocity[j] = 0.0;
          real_joint_cmd_.effort[j] = 0.0;
          real_joint_cmd_.stiffness[j] = 80;  // 80
          real_joint_cmd_.damping[j] = 1.5;
          // std::cerr << pos_des << " ";
        } else {
          pos_des = current_joint_angles_(j) * (1 - trans_mode_percent_) +
                    trans_mode_percent_ *
                        control_conf_.joint_conf["init_state"][control_conf_.ordered_joint_name[j]];
          if (j == 4 || j == 5 || j == 10 || j == 11) {
            // sim
            sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                     0.0 * (0.0 - joint_state_data_.velocity[index]);
            // real
            real_joint_cmd_.position[j] = pos_des;
            real_joint_cmd_.velocity[j] = 0.0;
            real_joint_cmd_.effort[j] = 0.0;
            real_joint_cmd_.stiffness[j] = 80.0;
            real_joint_cmd_.damping[j] = 1.5;
          } else {
            sim_joint_cmd_.data[j] = 0.0 + 30.0 * (pos_des - joint_state_data_.position[index]) +
                                     0.5 * (0.0 - joint_state_data_.velocity[index]);
            // real
            real_joint_cmd_.position[j] = pos_des;
            real_joint_cmd_.velocity[j] = 0.0;
            real_joint_cmd_.effort[j] = 0.0;
            real_joint_cmd_.stiffness[j] = 200.0;
            real_joint_cmd_.damping[j] = 1.5;
          }
        }
      }
      // std::cerr << std::endl;
      trans_mode_percent_ += 1 / trans_mode_duration_ms_;
      if (trans_mode_percent_ > 1) {
        to_first_pos_ok = true;
      } else {
        to_first_pos_ok = false;
      }
      trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
    }
  } else {
    if (wave_hand_ctl_num_++ < wave_hand_data_.size() - 1) {
      for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
        int index = joint_name_index_[control_conf_.ordered_joint_name[j]];
        if (index >= 20 && index <= 24) {
          pos_des = wave_hand_data_[wave_hand_ctl_num_ - 1][index - 20];
          real_joint_cmd_.position[j] = pos_des;
          real_joint_cmd_.velocity[j] = 0.0;
          real_joint_cmd_.effort[j] = 0.0;
          real_joint_cmd_.stiffness[j] = 80;  // 80
          real_joint_cmd_.damping[j] = 1.5;
        }
      }
    } else {
      start_wave_hand_ = false;
    }
  }
  loop_count_ = 0;
}


void RLController::HandleWalkMode() {
  // compute observation & actions
  if (loop_count_ % control_conf_.walk_step_conf.decimation == 0) {
    ComputeObservation();
    ComputeActions();
  }
  loop_count_++;

  int index = 0;
  // set action  rl
  for (size_t i = 0; i < control_conf_.onnx_conf.actions_size; i++) {
    if (i < 6)
    {
      index = i;
    }else if (i == 6)
    {
      index = 15;
    }else if (i < 13)
    {
      index = i - 1;
    }else if(i == 13){
      index = 23;
    }else{
      std::cerr << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;
      index = 0;
    }

    std::string joint_name = control_conf_.ordered_joint_name[index];

    scalar_t pos_des = actions_[i] * control_conf_.walk_step_conf.action_scale +
                       control_conf_.joint_conf["init_state"][joint_name];
    double stiffness = control_conf_.joint_conf["stiffness"][joint_name];
    double damping = control_conf_.joint_conf["damping"][joint_name];

    // sim
    sim_joint_cmd_.data[index] =
        0.0 + stiffness * (pos_des - joint_state_data_.position[joint_name_index_[joint_name]]) +
        damping * (0.0 - joint_state_data_.velocity[joint_name_index_[joint_name]]);
    // real
    // int index = joint_name_index_[control_conf_.ordered_joint_name[index]];
    if (index == 4 || index == 5 || index == 10 || index == 11) {
      double rate = 0.0;

      double tau_des = (1 - rate) * stiffness * (pos_des - propri_.joint_pos[i]) +
                       (1 - rate) * damping * (0 - propri_.joint_vel[i]);
      low_pass_filters_[i].input(tau_des);
      double tau_des_lp = low_pass_filters_[i].output();
      real_joint_cmd_.position[index] = pos_des;
      real_joint_cmd_.velocity[index] = 0.0;
      real_joint_cmd_.effort[index] = tau_des_lp;
      real_joint_cmd_.stiffness[index] = rate * stiffness;
      real_joint_cmd_.damping[index] = 2;
    } else {
      if ((start_wave_hand_) &&  (index==15 || index == 23)){
        continue;
      }
      double rate = 1;

      double tau_des = (1 - rate) * stiffness * (pos_des - propri_.joint_pos[i]) +
                       (1 - rate) * damping * (0 - propri_.joint_vel[i]);

      low_pass_filters_[i].input(pos_des);
      double pos_des_lp = low_pass_filters_[i].output();
      real_joint_cmd_.position[index] = pos_des_lp;
      real_joint_cmd_.velocity[index] = 0.0;
      real_joint_cmd_.effort[index] = tau_des;
      real_joint_cmd_.stiffness[index] = rate * stiffness;
      real_joint_cmd_.damping[index] = rate * damping;
    }
    last_actions_(i, 0) = actions_[i];
  }

  // arm
  // for (size_t i = 15; i < control_conf_.ordered_joint_name.size(); i++) {
  //     std::string joint_name = control_conf_.ordered_joint_name[i];
  //     double joint_init = control_conf_.joint_conf["init_state"][joint_name];
  //     if (i == 15 || i == 23)
  //     {
  //       continue;
  //     }
  //     if ((start_wave_hand_) &&  (i>=15)){
  //     }
  //     else {
  //       if (i== 18)// set elbow pitch    index = 18 26
  //       {
  //         joint_init = joint_init + left_elbow_pitch;
  //       }else if (i == 26)
  //       {
  //         joint_init = joint_init + right_elbow_pitch;
  //       }
        
  //       double stiffness = control_conf_.joint_conf["stiffness"][joint_name];
  //       double damping = control_conf_.joint_conf["damping"][joint_name];

  //       real_joint_cmd_.position[i] = joint_init;
  //       real_joint_cmd_.velocity[i] = 0.0;
  //       real_joint_cmd_.effort[i] = 0.0;
  //       real_joint_cmd_.stiffness[i] = stiffness;
  //       real_joint_cmd_.damping[i] = damping;
  //     }
  // }
  if (start_wave_hand_) {
    wave_hand();
  }
  clamping_jaw_ctl();
  push_rod_ctl();
}

void RLController::UpdateStateEstimation() {
  // std::cout << " UpdateStateEstimation ###########################" << std::endl;
  propri_.joint_pos.resize(control_conf_.onnx_conf.actions_size);
  propri_.joint_vel.resize(control_conf_.onnx_conf.actions_size);

  int index = 0;
  for (size_t i = 0; i < control_conf_.onnx_conf.actions_size; ++i) {
    if (i < 6)
    {
      index = i;
    }else if (i == 6)
    {
      index = 15;
    }else if (i < 13)
    {
      index = i - 1;
    }else if(i == 13){
      index = 23;
    }else{
      std::cerr << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;
      index = 0;
    }

    std::string joint_name = control_conf_.ordered_joint_name[index];
    if (start_wave_hand_ && (index == 15 || index == 23)){ 
      propri_.joint_pos(i) = 0;
      joint_vel_filters_[i].input(0);
      propri_.joint_vel(i) = joint_vel_filters_[i].output();
    }
    else{ 
      propri_.joint_pos(i) = joint_state_data_.position[joint_name_index_[joint_name]];
      joint_vel_filters_[i].input(joint_state_data_.velocity[joint_name_index_[joint_name]]);
      propri_.joint_vel(i) = joint_vel_filters_[i].output();
    }
  }

  vector3_t angular_vel;

  imu_angle_vel_filters_[0].input(imu_data_.angular_velocity.x);
  imu_angle_vel_filters_[1].input(imu_data_.angular_velocity.y);
  imu_angle_vel_filters_[2].input(imu_data_.angular_velocity.z);

  angular_vel(0) = imu_angle_vel_filters_[0].output();
  angular_vel(1) = imu_angle_vel_filters_[1].output();
  angular_vel(2) = imu_angle_vel_filters_[2].output();

  
  propri_.base_ang_vel = angular_vel;

  vector3_t gravity_vector(0, 0, -1);
  quaternion_t quat;
  quat.x() = imu_data_.orientation.x;
  quat.y() = imu_data_.orientation.y;
  quat.z() = imu_data_.orientation.z;
  quat.w() = imu_data_.orientation.w;
  matrix_t inverse_rot = GetRotationMatrixFromZyxEulerAngles(QuatToZyx(quat)).inverse();
  propri_.projected_gravity = inverse_rot * gravity_vector;
  propri_.base_euler_xyz = QuatToXyz(quat);
  propri_.base_euler_xyz[2] = 0;
}

void RLController::ComputeObservation() {
  // std::cout << " ComputeObservation ###########################" << std::endl;
  double phase = duration<double>(high_resolution_clock::now().time_since_epoch()).count();

  static int stop_count = 301;
  if (control_conf_.walk_step_conf.sw_mode) {
    double cmd_norm = std::sqrt(Square(joy_data_.linear.x) + Square(joy_data_.linear.y) +
                                Square(joy_data_.angular.z));
    if (cmd_norm <= control_conf_.walk_step_conf.cmd_threshold) {
      stop_count++;
    } else {
      stop_count = 0;
    }

    if (stop_count > 300) {
      phase = 0;
    }
  }
  // static double cycle = control_conf_.walk_step_conf.cycle_time;
  

  // if (joy_data_.linear.x > 0.5 && fabs(sin(2 * M_PI * phase)) < 0.01)
  // {
  //   ctl_cycle = 0.6;

  // }
  // else if (joy_data_.linear.x <= 0.5 && fabs(sin(2 * M_PI * phase)) < 0.01) 
  // {
  //   ctl_cycle = control_conf_.walk_step_conf.cycle_time;
  // }

  // if (joy_data_.linear.x > 0.5 && fabs(sin(2 * M_PI * phase)) < 0.01)
  // {
  //   cycle = 0.6;
  // }
  // else{
  //   cycle =  control_conf_.walk_step_conf.cycle_time;
  // }
  phase = phase / ctl_cycle;

  // actions
  ControlCfg::OnnxCfg& onnx_conf = control_conf_.onnx_conf;
  vector_t propri_obs(onnx_conf.obs_size);
// joy_data_.angular.z

  cmd_vel_low_pass_filters_[0].input(joy_data_.linear.x);
  cmd_vel_low_pass_filters_[1].input(joy_data_.linear.y);
  cmd_vel_low_pass_filters_[2].input(joy_data_.angular.z);

  joy_data_filter.linear.x = cmd_vel_low_pass_filters_[0].output();
  joy_data_filter.linear.y = cmd_vel_low_pass_filters_[1].output();
  joy_data_filter.angular.z = cmd_vel_low_pass_filters_[2].output();


  propri_obs << sin(2 * M_PI * phase), cos(2 * M_PI * phase),
      joy_data_filter.linear.x * max_lin_vel_x * control_conf_.obs_scales.lin_vel,
      joy_data_filter.linear.y * max_lin_vel_y * control_conf_.obs_scales.lin_vel, joy_data_filter.angular.z * max_angle_vel_z,
      (propri_.joint_pos - init_joint_angles_) * control_conf_.obs_scales.dof_pos,
      propri_.joint_vel * control_conf_.obs_scales.dof_vel, last_actions_,
      propri_.base_ang_vel * control_conf_.obs_scales.ang_vel,
      propri_.base_euler_xyz * control_conf_.obs_scales.quat;
  // std::cout << propri_.base_euler_xyz.transpose() << std::endl;
  // std::cout << "joy_data_.linear.x * 0.5" << joy_data_.linear.x * 0.5 << std::endl;
  // std::cout << "joy_data_.linear.x  * 0.5 * control_conf_.obs_scales.lin_vel" << joy_data_.linear.x * 0.5 * control_conf_.obs_scales.lin_vel  << std::endl;


  if (is_first_rec_obs_) {
    int index = 0;
    for (size_t j = 0; j < control_conf_.onnx_conf.actions_size; ++j) {
      if (j < 6)
      {
        index = j;
      }else if (j == 6)
      {
        index = 15;
      }else if (j < 13)
      {
        index = j - 1;
      }else if(j == 13){
        index = 23;
      }else{
        std::cerr << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << std::endl;
        index = 0;
      }

      if (index == 4 || index == 5 || index == 10 || index == 11) {
        low_pass_filters_[j].init(0);
      } else {
        low_pass_filters_[j].init(propri_.joint_pos[j]);
      }
    }

    for (int i = 33; i < 47; ++i) {
      propri_obs(i, 0) = 0.0;
    }
    for (int i = 0; i < onnx_conf.num_hist; ++i) {
      propri_history_buffer_.segment(i * onnx_conf.obs_size, onnx_conf.obs_size) =
          propri_obs.cast<tensor_element_t>();
    }
    is_first_rec_obs_ = false;
  }

  propri_history_buffer_.head(propri_history_buffer_.size() - onnx_conf.obs_size) =
      propri_history_buffer_.tail(propri_history_buffer_.size() - onnx_conf.obs_size);
  propri_history_buffer_.tail(onnx_conf.obs_size) = propri_obs.cast<tensor_element_t>();

  for (int i = 0; i < (onnx_conf.obs_size * onnx_conf.num_hist); ++i) {
    observations_[i] = static_cast<tensor_element_t>(propri_history_buffer_[i]);
  }
  // limit observations range
  scalar_t obs_min = -onnx_conf.obs_clip;
  scalar_t obs_max = onnx_conf.obs_clip;
  std::transform(
      observations_.begin(), observations_.end(), observations_.begin(),
      [obs_min, obs_max](scalar_t x) { return std::max(obs_min, std::min(obs_max, x)); });


    // 计算 swing arm angle
  left_elbow_pitch = ComputeSwingArm(phase, -0.3);
  right_elbow_pitch = ComputeSwingArm(phase, 0.3);
}
std::vector<std::vector<double>> RLController::readData(const std::string& file_path) {
  std::vector<std::vector<double>> data;  // 存储多行数据
  std::ifstream file(file_path);          // 打开文件

  if (!file.is_open()) {
    std::cerr << "Unable to open file: " << file_path << std::endl;
    return data;
  }

  std::string line;
  // 逐行读取文件内容
  while (std::getline(file, line)) {
    std::istringstream iss(line);  // 使用istringstream解析行数据
    std::vector<double> row;       // 存储当前行的数值
    double value;

    // 使用字符串流逐个读取数字，并存入row向量
    while (iss >> value) {
      row.push_back(value);
    }

    // 如果当前行有数据，则存入data向量
    if (!row.empty()) {
      data.push_back(row);
    }
  }

  file.close();  // 关闭文件
  return data;
}

double RLController::ComputeSwingArm(double phase, double delta_q){
  double sin_pos = sin(2 * M_PI * phase);  
  double angle = sin_pos * delta_q;
  return angle;
}

void RLController::ComputeActions() {
  // std::cout << " ComputeActions ###########################" << std::endl;
  // create input tensor object
  std::vector<Ort::Value> input_tensor;
  input_tensor.push_back(Ort::Value::CreateTensor<tensor_element_t>(
      memory_info_, observations_.data(), observations_.size(), input_shapes_[0].data(),
      input_shapes_[0].size()));

  std::vector<Ort::Value> output_values = session_ptr_->Run(
      Ort::RunOptions{}, input_names_.data(), input_tensor.data(), 1, output_names_.data(), 1);

  for (int i = 0; i < control_conf_.onnx_conf.actions_size; ++i) {
    actions_[i] = *(output_values[0].GetTensorMutableData<tensor_element_t>() + i);
  }
  // limit action range
  scalar_t action_min = -control_conf_.onnx_conf.actions_clip;
  scalar_t action_max = control_conf_.onnx_conf.actions_clip;
  std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                 [action_min, action_max](scalar_t x) {
                   return std::max(action_min, std::min(action_max, x));
                 });
}
void RLController::start_wave_hand(int mode) {
  mode_ = mode;
  
  if (mode >=0 && mode <= 2){
    wave_hand_data_mode_ = mode;
    std::cerr << "change mode to " << wave_hand_data_mode_ << std::endl;
  }
  else{
    std::cerr << "wave hand mode err " << std::endl;  
    return ;
  }
  if (!get_wave_hand_data_){
    std::cerr << "not find wave_hand data  !!!!!!!!!" << std::endl;
    return;
  }

  int wave_hand_type = 0;
  switch (wave_hand_data_mode_)
  {
    case 0:
      wave_hand_data_ = wave_hand_txt_data_[0];
      wave_hand_type = wave_hand_type_[0];
      break;
    case 1:
      wave_hand_data_ = wave_hand_txt_data_[1];
      wave_hand_type = wave_hand_type_[1];

      break;
    case 2:
      wave_hand_data_ = wave_hand_txt_data_[2];
      wave_hand_type = wave_hand_type_[2];
      break;
    default:
      break;
  }
  

  if (last_wave_hand_data_mode_!= wave_hand_data_mode_){
      wave_hand_step_ = 0;
      first_wave_hand_ = true;
  }

  last_wave_hand_data_mode_ = wave_hand_data_mode_;

  if (wave_hand_step_ == 0 || wave_hand_step_ == 2){
    wave_hand_percent_ = 0.0;
    wave_hand_ctl_num_ = 0;
    to_first_pos_ok = false;

    for (size_t j = 0; j < control_conf_.ordered_joint_name.size(); ++j) {
      if (first_wave_hand_){
        move_start_joint_angles_(j) =
            joint_state_data_.position[joint_name_index_[control_conf_.ordered_joint_name[j]]];
      }
      else{
        move_start_joint_angles_(j) = last_tg_joint_angles_[j];
      }
    } 
    if (wave_hand_type == 0){
      wave_hand_step_ = 2;
    } else{
      wave_hand_step_ = 1;
    }

    start_wave_hand_ = true;
    first_wave_hand_ = false;
    std::cout << "start wave_hand" << std::endl;
  }
  else if (wave_hand_step_ == 1){
      wave_hand_step_ = 2;
  }
  else{
    std::cerr << "wave hand mode err " << std::endl;
  }
}
// bool RLController::stop_wave_hand(){
//   start_wave_hand_ = false;
//   wave_hand_percent_ = 0.0;
//   wave_hand_ctl_num_ = 0;
//   to_first_pos_ok = false;
//   std::cout << "stop wave_hand" <<std::endl;
// }
}  // namespace xyber_x1_infer::rl_control_module
