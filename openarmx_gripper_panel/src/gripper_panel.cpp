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

#include "openarmx_gripper_panel/gripper_panel.hpp"

#include <QTimer>
#include <QMessageBox>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include <chrono>

namespace openarmx_gripper_panel
{

GripperPanel::GripperPanel(QWidget* parent)
  : rviz_common::Panel(parent)
{
  createLayout();
}

GripperPanel::~GripperPanel()
{
}

void GripperPanel::onInitialize()
{
  // 创建ROS2节点
  node_ = std::make_shared<rclcpp::Node>("gripper_panel_node");

  // 创建Action客户端
  left_gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    node_, "/left_gripper_controller/gripper_cmd");

  right_gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    node_, "/right_gripper_controller/gripper_cmd");

  // 等待服务器 (非阻塞)
  QTimer::singleShot(1000, this, [this]() {
    bool left_ready = left_gripper_client_->wait_for_action_server(std::chrono::seconds(1));
    bool right_ready = right_gripper_client_->wait_for_action_server(std::chrono::seconds(1));

    if (!left_ready && !right_ready) {
      status_label_->setText("⚠ 警告: 夹爪控制器未连接");
      status_label_->setStyleSheet("padding: 5px; background-color: #ffcccc; color: #cc0000;");
    } else if (!left_ready) {
      status_label_->setText("⚠ 警告: 左夹爪控制器未连接");
      status_label_->setStyleSheet("padding: 5px; background-color: #fff3cd; color: #856404;");
    } else if (!right_ready) {
      status_label_->setText("⚠ 警告: 右夹爪控制器未连接");
      status_label_->setStyleSheet("padding: 5px; background-color: #fff3cd; color: #856404;");
    } else {
      status_label_->setText("✓ 就绪 - 所有夹爪控制器已连接");
      status_label_->setStyleSheet("padding: 5px; background-color: #d4edda; color: #155724;");
    }
  });

  // 启动定时器处理ROS回调
  QTimer* ros_timer = new QTimer(this);
  connect(ros_timer, &QTimer::timeout, this, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_timer->start(50);  // 20Hz
}

void GripperPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);

  // 加载保存的夹爪选择
  int gripper_index;
  if (config.mapGetInt("gripper_selection", &gripper_index))
  {
    gripper_selector_->setCurrentIndex(gripper_index);
  }
}

void GripperPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  // 保存夹爪选择
  config.mapSetValue("gripper_selection", gripper_selector_->currentIndex());
}

void GripperPanel::createLayout()
{
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->setSpacing(8);
  main_layout->setContentsMargins(10, 10, 10, 10);

  // 标题
  QLabel* title = new QLabel("<h2>OpenArm 夹爪控制</h2>");
  title->setAlignment(Qt::AlignCenter);
  title->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  main_layout->addWidget(title);

  // 夹爪选择器
  QGroupBox* selector_group = new QGroupBox("选择夹爪");
  selector_group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  QHBoxLayout* selector_layout = new QHBoxLayout;

  QLabel* selector_label = new QLabel("控制对象:");
  gripper_selector_ = new QComboBox();
  gripper_selector_->addItem("左夹爪", "left");
  gripper_selector_->addItem("右夹爪", "right");
  gripper_selector_->addItem("双夹爪 (同步)", "both");
  gripper_selector_->setCurrentIndex(2);  // 默认选择双夹爪
  gripper_selector_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  selector_layout->addWidget(selector_label);
  selector_layout->addWidget(gripper_selector_, 1);
  selector_group->setLayout(selector_layout);
  main_layout->addWidget(selector_group);

  // 位置控制组
  QGroupBox* control_group = new QGroupBox("位置控制");
  control_group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  QVBoxLayout* control_layout = new QVBoxLayout;
  control_layout->setSpacing(8);

  // 滑块
  QHBoxLayout* slider_layout = new QHBoxLayout;
  QLabel* slider_label = new QLabel("位置:");
  slider_label->setMinimumWidth(50);
  slider_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  position_slider_ = new QSlider(Qt::Horizontal);
  position_slider_->setRange(0, SLIDER_MAX);
  position_slider_->setValue(0);
  position_slider_->setTickPosition(QSlider::TicksBelow);
  position_slider_->setTickInterval(5);
  position_slider_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  value_label_ = new QLabel("0.0 mm");
  value_label_->setMinimumWidth(70);
  value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  value_label_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  QFont value_font = value_label_->font();
  value_font.setPointSize(11);
  value_font.setBold(true);
  value_label_->setFont(value_font);

  slider_layout->addWidget(slider_label);
  slider_layout->addWidget(position_slider_, 1);
  slider_layout->addWidget(value_label_);
  control_layout->addLayout(slider_layout);

  // 快捷按钮
  QHBoxLayout* quick_btn_layout = new QHBoxLayout;

  close_btn_ = new QPushButton("关闭 (0mm)");
  half_btn_ = new QPushButton("半开 (22mm)");
  open_btn_ = new QPushButton("打开 (44mm)");

  close_btn_->setMinimumHeight(35);
  half_btn_->setMinimumHeight(35);
  open_btn_->setMinimumHeight(35);

  close_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  half_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  open_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  quick_btn_layout->addWidget(close_btn_);
  quick_btn_layout->addWidget(half_btn_);
  quick_btn_layout->addWidget(open_btn_);
  control_layout->addLayout(quick_btn_layout);

  // 应用按钮
  apply_btn_ = new QPushButton("应用 - 执行命令");
  apply_btn_->setMinimumHeight(40);
  apply_btn_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  apply_btn_->setStyleSheet(
    "QPushButton {"
    "  background-color: #4CAF50;"
    "  color: white;"
    "  font-weight: bold;"
    "  font-size: 14px;"
    "  border-radius: 5px;"
    "}"
    "QPushButton:hover {"
    "  background-color: #45a049;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #3d8b40;"
    "}"
  );
  control_layout->addWidget(apply_btn_);

  control_group->setLayout(control_layout);
  main_layout->addWidget(control_group);

  // 状态标签
  status_label_ = new QLabel("准备就绪");
  status_label_->setStyleSheet("padding: 8px; background-color: #e0e0e0; border-radius: 3px;");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  status_label_->setWordWrap(true);
  main_layout->addWidget(status_label_);

  // 使用说明
  QLabel* help_label = new QLabel(
    "<small><i>提示: 选择夹爪 → 调节位置 → 点击应用</i></small>"
  );
  help_label->setAlignment(Qt::AlignCenter);
  help_label->setStyleSheet("color: #666; padding: 5px;");
  help_label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  help_label->setWordWrap(true);
  main_layout->addWidget(help_label);

  // 添加弹性空间到底部（而不是顶部），确保内容从上到下排列
  main_layout->addStretch(1);

  // 设置整体布局，确保面板可以正确显示所有内容
  setLayout(main_layout);

  // 设置面板的最小尺寸，确保所有按钮都能显示
  setMinimumSize(280, 400);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

  // 连接信号槽
  connect(position_slider_, &QSlider::valueChanged, this, &GripperPanel::onSliderChanged);
  connect(close_btn_, &QPushButton::clicked, this, &GripperPanel::onCloseClicked);
  connect(half_btn_, &QPushButton::clicked, this, &GripperPanel::onHalfClicked);
  connect(open_btn_, &QPushButton::clicked, this, &GripperPanel::onOpenClicked);
  connect(apply_btn_, &QPushButton::clicked, this, &GripperPanel::onApplyClicked);
  connect(gripper_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &GripperPanel::onGripperSelectionChanged);
}

std::string GripperPanel::getSelectedGripper() const
{
  return gripper_selector_->currentData().toString().toStdString();
}

void GripperPanel::sendGripperCommand(const std::string& side, double position)
{
  auto goal_msg = control_msgs::action::GripperCommand::Goal();
  goal_msg.command.position = position;
  goal_msg.command.max_effort = 10.0;

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();

  QString status_msg;

  // 双夹爪模式：先发送右夹爪，等待50ms，再发送左夹爪
  if (side == "both") {
    // 发送右夹爪命令
    right_gripper_client_->async_send_goal(goal_msg, send_goal_options);

    // 等待 5ms
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // 发送左夹爪命令
    left_gripper_client_->async_send_goal(goal_msg, send_goal_options);

    status_msg = QString("✓ 已发送: 双夹爪 → %1 mm")
                   .arg(position * 1000, 0, 'f', 1);
    status_label_->setText(status_msg);
    status_label_->setStyleSheet("padding: 8px; background-color: #d4edda; color: #155724; border-radius: 3px;");
  }
  // 单个左夹爪控制
  else if (side == "left") {
    left_gripper_client_->async_send_goal(goal_msg, send_goal_options);
    status_msg = QString("✓ 已发送: 左夹爪 → %1 mm")
                   .arg(position * 1000, 0, 'f', 1);
    status_label_->setText(status_msg);
    status_label_->setStyleSheet("padding: 8px; background-color: #d4edda; color: #155724; border-radius: 3px;");
  }
  // 单个右夹爪控制
  else if (side == "right") {
    right_gripper_client_->async_send_goal(goal_msg, send_goal_options);
    status_msg = QString("✓ 已发送: 右夹爪 → %1 mm")
                   .arg(position * 1000, 0, 'f', 1);
    status_label_->setText(status_msg);
    status_label_->setStyleSheet("padding: 8px; background-color: #d4edda; color: #155724; border-radius: 3px;");
  }
}

// 槽函数实现
void GripperPanel::onSliderChanged(int value)
{
  value_label_->setText(QString("%1 mm").arg(value));
}

void GripperPanel::onGripperSelectionChanged(int index)
{
  QString gripper_name = gripper_selector_->itemText(index);
  status_label_->setText(QString("已选择: %1").arg(gripper_name));
  status_label_->setStyleSheet("padding: 8px; background-color: #e0e0e0; border-radius: 3px;");
}

void GripperPanel::onCloseClicked()
{
  position_slider_->setValue(0);
}

void GripperPanel::onHalfClicked()
{
  position_slider_->setValue(22);
}

void GripperPanel::onOpenClicked()
{
  position_slider_->setValue(44);
}

void GripperPanel::onApplyClicked()
{
  double position = position_slider_->value() / 1000.0;
  std::string side = getSelectedGripper();
  sendGripperCommand(side, position);
}

}  // namespace openarmx_gripper_panel

PLUGINLIB_EXPORT_CLASS(openarmx_gripper_panel::GripperPanel, rviz_common::Panel)
