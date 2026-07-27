// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
// 4.0 International License (CC BY-NC-SA 4.0).
//
// To view a copy of this license, visit:
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

#pragma once

#include <memory>
#include <map>
#include <string>
#include <vector>

#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

namespace openarmx_kp_kd_panel {

/**
 * @brief RViz面板用于动态调整机器人关节的KP和KD值
 *
 * 该面板包含两个滑轨（KP和KD）和一个确认按钮，
 * 用于实时调整8个电机（7个手臂关节 + 1个夹爪）的刚度和阻尼参数。
 */
class KpKdPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit KpKdPanel(QWidget * parent = nullptr);
  ~KpKdPanel() override;

  // RViz保存/加载面板配置
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  // 在RViz上下文完成后获取ROS节点
  void onInitialize() override;

private Q_SLOTS:
  void onKpSliderChanged(int value);
  void onKdSliderChanged(int value);
  void onGripperKpSliderChanged(int value);
  void onGripperKdSliderChanged(int value);
  void onApplyClicked();
  void onResetArmKp();
  void onResetArmKd();
  void onResetGripperKp();
  void onResetGripperKd();
  void onArmSelectionChanged(int index);
  void onNamespaceChanged(const QString & text);

private:
  void setupUi();
  void setupRos();
  void updateLabels();
  // Build fully-qualified node name with optional namespace prefix
  std::string buildNodeName(const std::string & base_name) const;

  /**
   * @brief 将滑轨值映射到实际电机KP/KD范围
   * @param slider_value 滑轨值 KP: 0-1000, KD: 0-100
   * @param motor_index 电机索引 0-7
   * @param is_kp true为KP，false为KD
   * @return 映射后的实际值
   */
  double mapSliderToMotorValue(int slider_value, size_t motor_index, bool is_kp);

  /**
   * @brief 应用KP/KD值到所有电机
   */
  void applyKpKdValues();

private:
  // ROS2节点和参数客户端
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_right_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_left_;

  // 目标节点名称（硬件参数节点）
  std::string target_node_name_right_ = "/openarmx_right_hardware_params";
  std::string target_node_name_left_ = "/openarmx_left_hardware_params";

  // 命名空间前缀（多机器人时填入，如 "robot1"；单机器人留空）
  std::string ros_namespace_;

  // 当前控制模式：0=右臂, 1=左臂, 2=双臂
  int control_mode_ = 2;  // 默认双臂

  // 电机类型映射 (8个电机: Joint1-7 + Gripper)
  // Joint 1-2: RS04, Joint 3-4: RS03, Joint 5-8: RS00
  struct MotorLimits {
    double kp_min;
    double kp_max;
    double kd_min;
    double kd_max;
  };

  // 各电机类型的KP/KD范围
  std::map<std::string, MotorLimits> motor_limits_ = {
    {"RS04", {0.0, 5000.0, 0.0, 100.0}},  // Joint 1-2
    {"RS03", {0.0, 5000.0, 0.0, 100.0}},  // Joint 3-4
    {"RS00", {0.0, 500.0, 0.0, 5.0}}      // Joint 5-8
  };

  // 电机类型数组 (索引0-7对应Joint1-8)
  std::vector<std::string> motor_types_ = {
    "RS04", "RS04", "RS03", "RS03", "RS00", "RS00", "RS00", "RS00"
  };

  // 默认滑轨值 (参考 v10_simple_hardware.cpp 默认配置)
  static constexpr int DEFAULT_ARM_KP = 10;       // 对应 RS04/RS03: 50.0, RS00: 5.0
  static constexpr int DEFAULT_ARM_KD = 3;        // 对应 RS04/RS03: 3.0, RS00: 0.15
  static constexpr int DEFAULT_GRIPPER_KP = 100;  // 对应 RS00: 50.0
  static constexpr int DEFAULT_GRIPPER_KD = 50;   // 对应 RS00: 2.5

  // UI元素 - 手臂关节
  QSlider * kp_slider_{};
  QSlider * kd_slider_{};
  QLabel * kp_label_{};
  QLabel * kd_label_{};
  QLabel * kp_value_label_{};
  QLabel * kd_value_label_{};
  QPushButton * reset_arm_kp_button_{};
  QPushButton * reset_arm_kd_button_{};

  // UI元素 - 夹爪
  QSlider * gripper_kp_slider_{};
  QSlider * gripper_kd_slider_{};
  QLabel * gripper_kp_label_{};
  QLabel * gripper_kd_label_{};
  QLabel * gripper_kp_value_label_{};
  QLabel * gripper_kd_value_label_{};
  QPushButton * reset_gripper_kp_button_{};
  QPushButton * reset_gripper_kd_button_{};

  // UI元素 - 通用
  QComboBox * arm_selector_{};
  QLineEdit * namespace_input_{};
  QPushButton * apply_button_{};
  QLabel * status_label_{};

  // 当前滑轨值
  int kp_slider_value_{0};         // 手臂KP (0-1000)
  int kd_slider_value_{0};         // 手臂KD (0-100)
  int gripper_kp_slider_value_{0}; // 夹爪KP (0-1000)
  int gripper_kd_slider_value_{0}; // 夹爪KD (0-100)
};

} // namespace openarmx_kp_kd_panel
