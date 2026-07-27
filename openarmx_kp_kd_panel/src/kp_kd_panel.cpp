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

#include "openarmx_kp_kd_panel/kp_kd_panel.hpp"

#include <QGroupBox>
#include <QMessageBox>
#include <QScrollArea>
#include <chrono>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

namespace openarmx_kp_kd_panel {

KpKdPanel::KpKdPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUi();
}

KpKdPanel::~KpKdPanel()
{
  // Qt会自动清理子widget
}

void KpKdPanel::onInitialize()
{
  // 从RViz的DisplayContext获取ROS节点
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (ros_node_abstraction) {
    node_ = ros_node_abstraction->get_raw_node();

    // 根据当前选择的手臂设置控制模式
    if (arm_selector_) {
      control_mode_ = arm_selector_->currentData().toInt();
    }

    setupRos();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("KpKdPanel"), "Failed to get ROS node");
  }
}

void KpKdPanel::setupUi()
{
  // 主布局
  auto * main_layout = new QVBoxLayout;

  // 标题
  auto * title_label = new QLabel("<b>机器人KP/KD控制面板</b>");
  title_label->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(title_label);

  // 手臂选择组
  auto * arm_selection_group = new QGroupBox("手臂选择");
  auto * arm_selection_layout = new QHBoxLayout;
  auto * arm_label = new QLabel("控制手臂:");
  arm_selector_ = new QComboBox;
  arm_selector_->addItem("右臂 (Right)", 0);   // 仅右臂
  arm_selector_->addItem("左臂 (Left)", 1);    // 仅左臂
  arm_selector_->addItem("双臂 (Both)", 2);    // 双臂
  arm_selector_->setCurrentIndex(2);  // 默认选择双臂
  arm_selection_layout->addWidget(arm_label);
  arm_selection_layout->addWidget(arm_selector_);
  arm_selection_layout->addStretch();
  arm_selection_group->setLayout(arm_selection_layout);
  main_layout->addWidget(arm_selection_group);

  // 命名空间输入组（多机器人时使用）
  auto * ns_group = new QGroupBox("机器人命名空间（多机器人时填写，单机器人留空）");
  auto * ns_layout = new QHBoxLayout;
  auto * ns_label = new QLabel("命名空间:");
  namespace_input_ = new QLineEdit;
  namespace_input_->setPlaceholderText("例: robot1（单机器人留空）");
  ns_layout->addWidget(ns_label);
  ns_layout->addWidget(namespace_input_);
  ns_group->setLayout(ns_layout);
  main_layout->addWidget(ns_group);

  // 手臂关节 KP控制组
  auto * kp_group = new QGroupBox("手臂关节 KP 刚度参数 (Joint 1-7)");
  auto * kp_layout = new QVBoxLayout;

  // KP标签和值显示
  auto * kp_label_layout = new QHBoxLayout;
  kp_label_ = new QLabel("KP值:");
  kp_value_label_ = new QLabel("J1-2: 0.0, J3-4: 0.0, J5-7: 0.0");
  kp_label_layout->addWidget(kp_label_);
  kp_label_layout->addWidget(kp_value_label_);
  kp_label_layout->addStretch();
  kp_layout->addLayout(kp_label_layout);

  // KP滑轨和重置按钮
  auto * kp_slider_layout = new QHBoxLayout;
  kp_slider_ = new QSlider(Qt::Horizontal);
  kp_slider_->setRange(0, 1000);
  kp_slider_->setValue(0);
  kp_slider_->setTickPosition(QSlider::TicksBelow);
  kp_slider_->setTickInterval(100);
  kp_slider_layout->addWidget(kp_slider_);

  reset_arm_kp_button_ = new QPushButton("恢复默认");
  reset_arm_kp_button_->setMaximumWidth(80);
  reset_arm_kp_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; }");
  kp_slider_layout->addWidget(reset_arm_kp_button_);

  kp_layout->addLayout(kp_slider_layout);

  // KP范围提示
  auto * kp_hint = new QLabel("范围: 0-1000 (自动映射到各电机实际范围) | 默认值: 10");
  kp_hint->setStyleSheet("color: gray; font-size: 10px;");
  kp_layout->addWidget(kp_hint);

  kp_group->setLayout(kp_layout);
  main_layout->addWidget(kp_group);

  // 手臂关节 KD控制组
  auto * kd_group = new QGroupBox("手臂关节 KD 阻尼参数 (Joint 1-7)");
  auto * kd_layout = new QVBoxLayout;

  // KD标签和值显示
  auto * kd_label_layout = new QHBoxLayout;
  kd_label_ = new QLabel("KD值:");
  kd_value_label_ = new QLabel("J1-2: 0.0, J3-4: 0.0, J5-7: 0.0");
  kd_label_layout->addWidget(kd_label_);
  kd_label_layout->addWidget(kd_value_label_);
  kd_label_layout->addStretch();
  kd_layout->addLayout(kd_label_layout);

  // KD滑轨和重置按钮
  auto * kd_slider_layout = new QHBoxLayout;
  kd_slider_ = new QSlider(Qt::Horizontal);
  kd_slider_->setRange(0, 100);
  kd_slider_->setValue(0);
  kd_slider_->setTickPosition(QSlider::TicksBelow);
  kd_slider_->setTickInterval(10);
  kd_slider_layout->addWidget(kd_slider_);

  reset_arm_kd_button_ = new QPushButton("恢复默认");
  reset_arm_kd_button_->setMaximumWidth(80);
  reset_arm_kd_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; }");
  kd_slider_layout->addWidget(reset_arm_kd_button_);

  kd_layout->addLayout(kd_slider_layout);

  // KD范围提示
  auto * kd_hint = new QLabel("范围: 0-100 (自动映射到各电机实际范围) | 默认值: 3");
  kd_hint->setStyleSheet("color: gray; font-size: 10px;");
  kd_layout->addWidget(kd_hint);

  kd_group->setLayout(kd_layout);
  main_layout->addWidget(kd_group);

  // ========== 夹爪控制组 ==========
  auto * gripper_group = new QGroupBox("夹爪 KP/KD 参数 (Joint 8)");
  gripper_group->setStyleSheet("QGroupBox { font-weight: bold; color: #2196F3; }");
  auto * gripper_main_layout = new QVBoxLayout;

  // 夹爪KP控制
  auto * gripper_kp_layout = new QHBoxLayout;
  gripper_kp_label_ = new QLabel("夹爪 KP:");
  gripper_kp_value_label_ = new QLabel("0.0");
  gripper_kp_layout->addWidget(gripper_kp_label_);
  gripper_kp_layout->addWidget(gripper_kp_value_label_);
  gripper_kp_layout->addStretch();
  gripper_main_layout->addLayout(gripper_kp_layout);

  auto * gripper_kp_slider_layout = new QHBoxLayout;
  gripper_kp_slider_ = new QSlider(Qt::Horizontal);
  gripper_kp_slider_->setRange(0, 1000);
  gripper_kp_slider_->setValue(0);
  gripper_kp_slider_->setTickPosition(QSlider::TicksBelow);
  gripper_kp_slider_->setTickInterval(100);
  gripper_kp_slider_layout->addWidget(gripper_kp_slider_);

  reset_gripper_kp_button_ = new QPushButton("恢复默认");
  reset_gripper_kp_button_->setMaximumWidth(80);
  reset_gripper_kp_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; }");
  gripper_kp_slider_layout->addWidget(reset_gripper_kp_button_);

  gripper_main_layout->addLayout(gripper_kp_slider_layout);

  auto * gripper_kp_hint = new QLabel("范围: 0-1000 → 0-500 (RS00电机) | 默认值: 100");
  gripper_kp_hint->setStyleSheet("color: gray; font-size: 10px;");
  gripper_main_layout->addWidget(gripper_kp_hint);

  // 夹爪KD控制
  auto * gripper_kd_layout = new QHBoxLayout;
  gripper_kd_label_ = new QLabel("夹爪 KD:");
  gripper_kd_value_label_ = new QLabel("0.0");
  gripper_kd_layout->addWidget(gripper_kd_label_);
  gripper_kd_layout->addWidget(gripper_kd_value_label_);
  gripper_kd_layout->addStretch();
  gripper_main_layout->addLayout(gripper_kd_layout);

  auto * gripper_kd_slider_layout = new QHBoxLayout;
  gripper_kd_slider_ = new QSlider(Qt::Horizontal);
  gripper_kd_slider_->setRange(0, 100);
  gripper_kd_slider_->setValue(0);
  gripper_kd_slider_->setTickPosition(QSlider::TicksBelow);
  gripper_kd_slider_->setTickInterval(10);
  gripper_kd_slider_layout->addWidget(gripper_kd_slider_);

  reset_gripper_kd_button_ = new QPushButton("恢复默认");
  reset_gripper_kd_button_->setMaximumWidth(80);
  reset_gripper_kd_button_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; }");
  gripper_kd_slider_layout->addWidget(reset_gripper_kd_button_);

  gripper_main_layout->addLayout(gripper_kd_slider_layout);

  auto * gripper_kd_hint = new QLabel("范围: 0-100 → 0-5 (RS00电机) | 默认值: 50");
  gripper_kd_hint->setStyleSheet("color: gray; font-size: 10px;");
  gripper_main_layout->addWidget(gripper_kd_hint);

  gripper_group->setLayout(gripper_main_layout);
  main_layout->addWidget(gripper_group);

  // 应用按钮
  apply_button_ = new QPushButton("应用KP/KD到所有关节");
  apply_button_->setStyleSheet(
    "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; "
    "padding: 10px; border-radius: 5px; }"
    "QPushButton:hover { background-color: #45a049; }"
    "QPushButton:pressed { background-color: #3d8b40; }"
  );
  main_layout->addWidget(apply_button_);

  // 状态标签
  status_label_ = new QLabel("状态: 等待连接...");
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("background-color: #f0f0f0; padding: 5px; border-radius: 3px;");
  main_layout->addWidget(status_label_);

  // 电机信息提示
  auto * info_label = new QLabel(
    "<small><b>电机配置:</b><br>"
    "• Joint 1-2: RS04 (KP:0-5000, KD:0-100)<br>"
    "• Joint 3-4: RS03 (KP:0-5000, KD:0-100)<br>"
    "• Joint 5-7: RS00 (KP:0-500, KD:0-5)<br>"
    "• Joint 8 (夹爪): RS00 (KP:0-500, KD:0-5)</small>"
  );
  info_label->setStyleSheet("color: #666; padding: 5px;");
  main_layout->addWidget(info_label);

  main_layout->addStretch();

  // 创建容器widget来包含所有内容
  auto * container_widget = new QWidget;
  container_widget->setLayout(main_layout);

  // 创建滚动区域
  auto * scroll_area = new QScrollArea;
  scroll_area->setWidget(container_widget);
  scroll_area->setWidgetResizable(true);  // 允许widget自动调整大小
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);  // 禁用水平滚动条
  scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);     // 需要时显示垂直滚动条
  scroll_area->setFrameShape(QFrame::NoFrame);  // 去掉边框

  // 设置面板的主布局
  auto * panel_layout = new QVBoxLayout;
  panel_layout->setContentsMargins(0, 0, 0, 0);  // 去掉边距
  panel_layout->addWidget(scroll_area);
  setLayout(panel_layout);

  // 连接信号和槽 - 手臂选择
  connect(arm_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &KpKdPanel::onArmSelectionChanged);

  // 连接信号和槽 - 命名空间输入
  connect(namespace_input_, &QLineEdit::textChanged,
          this, &KpKdPanel::onNamespaceChanged);

  // 连接信号和槽 - 手臂关节滑轨
  connect(kp_slider_, &QSlider::valueChanged, this, &KpKdPanel::onKpSliderChanged);
  connect(kd_slider_, &QSlider::valueChanged, this, &KpKdPanel::onKdSliderChanged);

  // 连接信号和槽 - 手臂关节重置按钮
  connect(reset_arm_kp_button_, &QPushButton::clicked, this, &KpKdPanel::onResetArmKp);
  connect(reset_arm_kd_button_, &QPushButton::clicked, this, &KpKdPanel::onResetArmKd);

  // 连接信号和槽 - 夹爪滑轨
  connect(gripper_kp_slider_, &QSlider::valueChanged, this, &KpKdPanel::onGripperKpSliderChanged);
  connect(gripper_kd_slider_, &QSlider::valueChanged, this, &KpKdPanel::onGripperKdSliderChanged);

  // 连接信号和槽 - 夹爪重置按钮
  connect(reset_gripper_kp_button_, &QPushButton::clicked, this, &KpKdPanel::onResetGripperKp);
  connect(reset_gripper_kd_button_, &QPushButton::clicked, this, &KpKdPanel::onResetGripperKd);

  // 应用按钮
  connect(apply_button_, &QPushButton::clicked, this, &KpKdPanel::onApplyClicked);

  // 初始化显示
  updateLabels();
}

void KpKdPanel::setupRos()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("KpKdPanel"), "Node is null, cannot setup ROS");
    status_label_->setText("错误: ROS节点未初始化");
    status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
    return;
  }

  // 创建参数客户端（总是创建两个，但根据模式决定使用哪个）
  param_client_right_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, buildNodeName("openarmx_right_hardware_params"));
  param_client_left_ = std::make_shared<rclcpp::AsyncParametersClient>(
      node_, buildNodeName("openarmx_left_hardware_params"));

  // 等待参数服务器连接（异步），同时检测仿真模式
  auto timer = node_->create_wall_timer(
    100ms,
    [this]() {
      bool right_ready = param_client_right_->service_is_ready();
      bool left_ready = param_client_left_->service_is_ready();

      // 根据控制模式检查连接状态
      bool is_connected = false;
      QString status_msg;

      if (control_mode_ == 0) {  // 仅右臂
        is_connected = right_ready;
        status_msg = right_ready
          ? QString("状态: 已连接到右臂 (真实硬件模式)")
          : QString("状态: 等待连接右臂...");
      } else if (control_mode_ == 1) {  // 仅左臂
        is_connected = left_ready;
        status_msg = left_ready
          ? QString("状态: 已连接到左臂 (真实硬件模式)")
          : QString("状态: 等待连接左臂...");
      } else {  // 双臂
        is_connected = (right_ready && left_ready);
        if (right_ready && left_ready) {
          status_msg = "状态: 已连接到双臂 (真实硬件模式)";
        } else if (right_ready || left_ready) {
          status_msg = QString("状态: 部分连接 (%1已连接)")
            .arg(right_ready ? "右臂" : "左臂");
        } else {
          status_msg = "状态: 等待连接双臂...";
        }
      }

      if (is_connected) {
        status_label_->setText(status_msg);
        status_label_->setStyleSheet("background-color: #ccffcc; padding: 5px; border-radius: 3px;");
        apply_button_->setEnabled(true);
      } else {
        // 检测是否在仿真模式
        auto node_names = node_->get_node_names();
        bool has_controller_manager = false;
        bool has_hardware_params = false;

        for (const auto& name : node_names) {
          if (name == "/controller_manager") {
            has_controller_manager = true;
          }
          if (name.find("hardware_params") != std::string::npos) {
            has_hardware_params = true;
          }
        }

        if (has_controller_manager && !has_hardware_params) {
          // 仿真模式
          status_label_->setText("状态: 仿真模式 - KP/KD控制不可用\n"
                                 "(fake_hardware 不支持动态参数调整)");
          status_label_->setStyleSheet("background-color: #e0e0e0; padding: 5px; border-radius: 3px; color: #666;");
          apply_button_->setEnabled(false);
          apply_button_->setStyleSheet(
            "QPushButton { background-color: #ccc; color: #888; font-weight: bold; "
            "padding: 10px; border-radius: 5px; }"
          );
        } else {
          // 等待连接
          status_label_->setText(status_msg + "\n请确保真实硬件节点正在运行");
          status_label_->setStyleSheet("background-color: #ffffcc; padding: 5px; border-radius: 3px;");
          apply_button_->setEnabled(true);
        }
      }
    }
  );
}

void KpKdPanel::onKpSliderChanged(int value)
{
  kp_slider_value_ = value;
  updateLabels();
}

void KpKdPanel::onKdSliderChanged(int value)
{
  kd_slider_value_ = value;
  updateLabels();
}

void KpKdPanel::onGripperKpSliderChanged(int value)
{
  gripper_kp_slider_value_ = value;
  updateLabels();
}

void KpKdPanel::onGripperKdSliderChanged(int value)
{
  gripper_kd_slider_value_ = value;
  updateLabels();
}

void KpKdPanel::updateLabels()
{
  // 显示手臂关节的映射值范围 (Joint 1-7)
  std::stringstream kp_ss, kd_ss;
  kp_ss << "J1-2: " << std::fixed << std::setprecision(1)
        << mapSliderToMotorValue(kp_slider_value_, 0, true)
        << ", J3-4: " << mapSliderToMotorValue(kp_slider_value_, 2, true)
        << ", J5-7: " << mapSliderToMotorValue(kp_slider_value_, 4, true);

  kd_ss << "J1-2: " << std::fixed << std::setprecision(2)
        << mapSliderToMotorValue(kd_slider_value_, 0, false)
        << ", J3-4: " << mapSliderToMotorValue(kd_slider_value_, 2, false)
        << ", J5-7: " << mapSliderToMotorValue(kd_slider_value_, 4, false);

  kp_value_label_->setText(QString::fromStdString(kp_ss.str()));
  kd_value_label_->setText(QString::fromStdString(kd_ss.str()));

  // 显示夹爪的映射值 (Joint 8, RS00电机)
  double gripper_kp_value = mapSliderToMotorValue(gripper_kp_slider_value_, 7, true);
  double gripper_kd_value = mapSliderToMotorValue(gripper_kd_slider_value_, 7, false);

  gripper_kp_value_label_->setText(QString::number(gripper_kp_value, 'f', 1));
  gripper_kd_value_label_->setText(QString::number(gripper_kd_value, 'f', 2));
}

double KpKdPanel::mapSliderToMotorValue(int slider_value, size_t motor_index, bool is_kp)
{
  // 获取电机类型
  if (motor_index >= motor_types_.size()) {
    return 0.0;
  }

  std::string motor_type = motor_types_[motor_index];
  const auto & limits = motor_limits_[motor_type];

  // 线性映射: KP slider (0-1000) or KD slider (0-100) -> motor range (min-max)
  double min_val = is_kp ? limits.kp_min : limits.kd_min;
  double max_val = is_kp ? limits.kp_max : limits.kd_max;
  double slider_max = is_kp ? 1000.0 : 100.0;

  return min_val + (max_val - min_val) * slider_value / slider_max;
}

void KpKdPanel::onApplyClicked()
{
  if (!node_) {
    QMessageBox::warning(this, "错误", "ROS节点未初始化");
    return;
  }

  // 根据控制模式检查客户端是否就绪
  bool right_ready = param_client_right_ && param_client_right_->service_is_ready();
  bool left_ready = param_client_left_ && param_client_left_->service_is_ready();

  bool can_apply = false;
  if (control_mode_ == 0) {  // 仅右臂
    can_apply = right_ready;
  } else if (control_mode_ == 1) {  // 仅左臂
    can_apply = left_ready;
  } else {  // 双臂
    can_apply = (right_ready && left_ready);
  }

  if (!can_apply) {
    QMessageBox::warning(this, "错误",
      "参数服务器未连接。\n\n请确保硬件节点正在运行并选择正确的手臂模式。");
    return;
  }

  applyKpKdValues();
}

void KpKdPanel::applyKpKdValues()
{
  status_label_->setText("状态: 正在应用参数...");
  status_label_->setStyleSheet("background-color: #ffffcc; padding: 5px; border-radius: 3px;");
  apply_button_->setEnabled(false);  // 禁用按钮防止重复点击

  // 准备参数列表
  std::vector<rclcpp::Parameter> parameters;

  // Joint 1-7: 使用手臂滑轨的KP/KD值
  for (size_t i = 0; i < 7; ++i) {
    double kp_value = mapSliderToMotorValue(kp_slider_value_, i, true);
    double kd_value = mapSliderToMotorValue(kd_slider_value_, i, false);

    std::string kp_name = "kp_joint" + std::to_string(i + 1);
    std::string kd_name = "kd_joint" + std::to_string(i + 1);

    parameters.push_back(rclcpp::Parameter(kp_name, kp_value));
    parameters.push_back(rclcpp::Parameter(kd_name, kd_value));

    RCLCPP_INFO(node_->get_logger(),
                "Setting [Arm] %s=%.2f, %s=%.2f",
                kp_name.c_str(), kp_value, kd_name.c_str(), kd_value);
  }

  // Joint 8 (夹爪): 使用夹爪滑轨的KP/KD值
  {
    double kp_value = mapSliderToMotorValue(gripper_kp_slider_value_, 7, true);
    double kd_value = mapSliderToMotorValue(gripper_kd_slider_value_, 7, false);

    std::string kp_name = "kp_joint8";
    std::string kd_name = "kd_joint8";

    parameters.push_back(rclcpp::Parameter(kp_name, kp_value));
    parameters.push_back(rclcpp::Parameter(kd_name, kd_value));

    RCLCPP_INFO(node_->get_logger(),
                "Setting [Gripper] %s=%.2f, %s=%.2f",
                kp_name.c_str(), kp_value, kd_name.c_str(), kd_value);
  }

  // 根据控制模式异步设置参数 - 使用QTimer避免lambda捕获问题
  if (control_mode_ == 0 && param_client_right_) {
    // 仅右臂
    RCLCPP_INFO(node_->get_logger(), "Applying parameters to RIGHT arm...");
    auto future_ptr = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
      param_client_right_->set_parameters(parameters));

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this, future_ptr, timer]() {
      if (future_ptr->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          auto results = future_ptr->get();
          bool all_success = true;
          for (const auto & result : results) {
            if (!result.successful) {
              all_success = false;
              RCLCPP_ERROR(node_->get_logger(), "Failed: %s", result.reason.c_str());
            }
          }
          if (all_success) {
            status_label_->setText("状态: 参数应用成功！");
            status_label_->setStyleSheet("background-color: #ccffcc; padding: 5px; border-radius: 3px;");

            // 显示成功弹窗 - 右臂模式
            QMessageBox::information(this, "✓ 成功",
              QString("KP/KD参数已成功应用到右臂！\n\n") +
              "【手臂关节】\n" +
              "Joint 1-2 (RS04): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 0, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 0, false), 'f', 2) + "\n" +
              "Joint 3-4 (RS03): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 2, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 2, false), 'f', 2) + "\n" +
              "Joint 5-7 (RS00): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 4, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 4, false), 'f', 2) + "\n\n" +
              "【夹爪】\n" +
              "Joint 8 (RS00): KP=" + QString::number(mapSliderToMotorValue(gripper_kp_slider_value_, 7, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(gripper_kd_slider_value_, 7, false), 'f', 2));
          } else {
            status_label_->setText("状态: 部分参数应用失败");
            status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
            QMessageBox::warning(this, "⚠ 警告", "部分参数应用失败，请查看终端日志");
          }
        } catch (const std::exception & e) {
          status_label_->setText("状态: 参数应用异常");
          status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
          QMessageBox::critical(this, "✗ 错误", QString("参数应用异常:\n") + e.what());
        }
        apply_button_->setEnabled(true);
        timer->stop();
        timer->deleteLater();
      }
    });
    timer->start(50);

  } else if (control_mode_ == 1 && param_client_left_) {
    // 仅左臂
    RCLCPP_INFO(node_->get_logger(), "Applying parameters to LEFT arm...");
    auto future_ptr = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
      param_client_left_->set_parameters(parameters));

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this, future_ptr, timer]() {
      if (future_ptr->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        try {
          auto results = future_ptr->get();
          bool all_success = true;
          for (const auto & result : results) {
            if (!result.successful) {
              all_success = false;
              RCLCPP_ERROR(node_->get_logger(), "Failed: %s", result.reason.c_str());
            }
          }
          if (all_success) {
            status_label_->setText("状态: 参数应用成功！");
            status_label_->setStyleSheet("background-color: #ccffcc; padding: 5px; border-radius: 3px;");

            // 显示成功弹窗 - 左臂模式
            QMessageBox::information(this, "✓ 成功",
              QString("KP/KD参数已成功应用到左臂！\n\n") +
              "【手臂关节】\n" +
              "Joint 1-2 (RS04): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 0, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 0, false), 'f', 2) + "\n" +
              "Joint 3-4 (RS03): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 2, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 2, false), 'f', 2) + "\n" +
              "Joint 5-7 (RS00): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 4, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 4, false), 'f', 2) + "\n\n" +
              "【夹爪】\n" +
              "Joint 8 (RS00): KP=" + QString::number(mapSliderToMotorValue(gripper_kp_slider_value_, 7, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(gripper_kd_slider_value_, 7, false), 'f', 2));
          } else {
            status_label_->setText("状态: 部分参数应用失败");
            status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
            QMessageBox::warning(this, "⚠ 警告", "部分参数应用失败，请查看终端日志");
          }
        } catch (const std::exception & e) {
          status_label_->setText("状态: 参数应用异常");
          status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
          QMessageBox::critical(this, "✗ 错误", QString("参数应用异常:\n") + e.what());
        }
        apply_button_->setEnabled(true);
        timer->stop();
        timer->deleteLater();
      }
    });
    timer->start(50);

  } else if (control_mode_ == 2 && param_client_right_ && param_client_left_) {
    // 双臂 - 并行发送到两个客户端
    RCLCPP_INFO(node_->get_logger(), "Applying parameters to BOTH arms...");
    auto future_right_ptr = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
      param_client_right_->set_parameters(parameters));
    auto future_left_ptr = std::make_shared<std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>>(
      param_client_left_->set_parameters(parameters));

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this, future_right_ptr, future_left_ptr, timer]() {
      bool right_ready = (future_right_ptr->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready);
      bool left_ready = (future_left_ptr->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready);

      if (right_ready && left_ready) {
        try {
          auto results_right = future_right_ptr->get();
          auto results_left = future_left_ptr->get();
          bool all_success = true;

          for (const auto & result : results_right) {
            if (!result.successful) {
              all_success = false;
              RCLCPP_ERROR(node_->get_logger(), "Right arm failed: %s", result.reason.c_str());
            }
          }
          for (const auto & result : results_left) {
            if (!result.successful) {
              all_success = false;
              RCLCPP_ERROR(node_->get_logger(), "Left arm failed: %s", result.reason.c_str());
            }
          }

          if (all_success) {
            status_label_->setText("状态: 双臂参数应用成功！");
            status_label_->setStyleSheet("background-color: #ccffcc; padding: 5px; border-radius: 3px;");

            // 显示成功弹窗 - 双臂模式
            QMessageBox::information(this, "✓ 成功",
              QString("KP/KD参数已成功应用到双臂！\n\n") +
              "【手臂关节 (两个手臂相同参数)】\n" +
              "Joint 1-2 (RS04): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 0, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 0, false), 'f', 2) + "\n" +
              "Joint 3-4 (RS03): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 2, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 2, false), 'f', 2) + "\n" +
              "Joint 5-7 (RS00): KP=" + QString::number(mapSliderToMotorValue(kp_slider_value_, 4, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(kd_slider_value_, 4, false), 'f', 2) + "\n\n" +
              "【夹爪 (两个手臂相同参数)】\n" +
              "Joint 8 (RS00): KP=" + QString::number(mapSliderToMotorValue(gripper_kp_slider_value_, 7, true), 'f', 1) +
              ", KD=" + QString::number(mapSliderToMotorValue(gripper_kd_slider_value_, 7, false), 'f', 2));
          } else {
            status_label_->setText("状态: 部分参数应用失败");
            status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
            QMessageBox::warning(this, "⚠ 警告", "部分参数应用失败，请查看终端日志");
          }
        } catch (const std::exception & e) {
          status_label_->setText("状态: 参数应用异常");
          status_label_->setStyleSheet("background-color: #ffcccc; padding: 5px; border-radius: 3px;");
          QMessageBox::critical(this, "✗ 错误", QString("参数应用异常:\n") + e.what());
        }
        apply_button_->setEnabled(true);
        timer->stop();
        timer->deleteLater();
      }
    });
    timer->start(50);
  }
}

void KpKdPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  // 从配置加载滑轨值 - 手臂关节
  int kp_val, kd_val;
  if (config.mapGetInt("kp_slider", &kp_val)) {
    kp_slider_->setValue(kp_val);
  }
  if (config.mapGetInt("kd_slider", &kd_val)) {
    kd_slider_->setValue(kd_val);
  }

  // 从配置加载滑轨值 - 夹爪
  int gripper_kp_val, gripper_kd_val;
  if (config.mapGetInt("gripper_kp_slider", &gripper_kp_val)) {
    gripper_kp_slider_->setValue(gripper_kp_val);
  }
  if (config.mapGetInt("gripper_kd_slider", &gripper_kd_val)) {
    gripper_kd_slider_->setValue(gripper_kd_val);
  }

  // 加载控制模式
  int control_mode;
  if (config.mapGetInt("control_mode", &control_mode)) {
    control_mode_ = control_mode;
    if (arm_selector_ && control_mode >= 0 && control_mode < arm_selector_->count()) {
      arm_selector_->setCurrentIndex(control_mode);
    }
  }

  // 加载命名空间
  QString ns;
  if (config.mapGetString("ros_namespace", &ns)) {
    ros_namespace_ = ns.toStdString();
    if (namespace_input_) {
      namespace_input_->setText(ns);
    }
  }
}

void KpKdPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  // 保存滑轨值 - 手臂关节
  config.mapSetValue("kp_slider", kp_slider_value_);
  config.mapSetValue("kd_slider", kd_slider_value_);

  // 保存滑轨值 - 夹爪
  config.mapSetValue("gripper_kp_slider", gripper_kp_slider_value_);
  config.mapSetValue("gripper_kd_slider", gripper_kd_slider_value_);

  // 保存控制模式
  config.mapSetValue("control_mode", control_mode_);

  // 保存命名空间
  config.mapSetValue("ros_namespace", QString::fromStdString(ros_namespace_));
}

// 重置按钮槽函数实现
void KpKdPanel::onResetArmKp()
{
  kp_slider_->setValue(DEFAULT_ARM_KP);
  RCLCPP_INFO(node_->get_logger(), "手臂KP值已恢复默认: %d", DEFAULT_ARM_KP);
}

void KpKdPanel::onResetArmKd()
{
  kd_slider_->setValue(DEFAULT_ARM_KD);
  RCLCPP_INFO(node_->get_logger(), "手臂KD值已恢复默认: %d", DEFAULT_ARM_KD);
}

void KpKdPanel::onResetGripperKp()
{
  gripper_kp_slider_->setValue(DEFAULT_GRIPPER_KP);
  RCLCPP_INFO(node_->get_logger(), "夹爪KP值已恢复默认: %d", DEFAULT_GRIPPER_KP);
}

void KpKdPanel::onResetGripperKd()
{
  gripper_kd_slider_->setValue(DEFAULT_GRIPPER_KD);
  RCLCPP_INFO(node_->get_logger(), "夹爪KD值已恢复默认: %d", DEFAULT_GRIPPER_KD);
}

void KpKdPanel::onArmSelectionChanged(int index)
{
  if (!arm_selector_) return;

  // 获取控制模式
  control_mode_ = arm_selector_->currentData().toInt();

  const char* mode_name[] = {"右臂", "左臂", "双臂"};
  RCLCPP_INFO(rclcpp::get_logger("KpKdPanel"),
              "手臂选择已更改: %s",
              mode_name[control_mode_]);

  // 重新设置ROS连接
  if (node_) {
    setupRos();
  }
}

std::string KpKdPanel::buildNodeName(const std::string & base_name) const
{
  if (ros_namespace_.empty()) {
    return "/" + base_name;
  }
  std::string ns = ros_namespace_;
  if (ns.front() != '/') ns = "/" + ns;
  if (ns.back() == '/') ns.pop_back();
  return ns + "/" + base_name;
}

void KpKdPanel::onNamespaceChanged(const QString & text)
{
  ros_namespace_ = text.trimmed().toStdString();
  if (node_) {
    setupRos();
  }
}

} // namespace openarmx_kp_kd_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(openarmx_kp_kd_panel::KpKdPanel, rviz_common::Panel)
