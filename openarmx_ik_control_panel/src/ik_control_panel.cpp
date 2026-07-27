// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
// 4.0 International License (CC BY-NC-SA 4.0).

#include "openarmx_ik_control_panel/ik_control_panel.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

namespace openarmx_ik_control_panel
{

IkControlPanel::IkControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  createLayout();
}

void IkControlPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("openarmx_ik_control_panel_node");
  rclcpp::QoS qos(1);
  qos.transient_local().reliable();
  enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/openarmx_teleop_vr/ik_enable_override", qos);

  auto * ros_timer = new QTimer(this);
  connect(ros_timer, &QTimer::timeout, this, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_timer->start(50);

  publishState();
}

void IkControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  bool enabled = false;
  if (config.mapGetBool("ik_enabled", &enabled)) {
    setEnabledState(enabled);
  }
}

void IkControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ik_enabled", ik_enabled_);
}

void IkControlPanel::createLayout()
{
  auto * main_layout = new QVBoxLayout;
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);

  auto * title = new QLabel("<h2>OpenArmX IK 控制</h2>");
  title->setAlignment(Qt::AlignCenter);
  main_layout->addWidget(title);

  auto * row_layout = new QHBoxLayout;
  row_layout->setSpacing(10);

  indicator_label_ = new QLabel;
  indicator_label_->setFixedSize(24, 24);
  row_layout->addWidget(indicator_label_);

  toggle_button_ = new QPushButton;
  toggle_button_->setMinimumHeight(42);
  row_layout->addWidget(toggle_button_, 1);

  main_layout->addLayout(row_layout);

  status_label_ = new QLabel;
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setWordWrap(true);
  main_layout->addWidget(status_label_);

  connect(toggle_button_, &QPushButton::clicked, this, &IkControlPanel::onToggleClicked);

  setLayout(main_layout);
  updateUi();
}

void IkControlPanel::onToggleClicked()
{
  setEnabledState(!ik_enabled_);
}

void IkControlPanel::setEnabledState(bool enabled)
{
  ik_enabled_ = enabled;
  updateUi();
  publishState();
}

void IkControlPanel::publishState()
{
  if (!enable_pub_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = ik_enabled_;
  enable_pub_->publish(msg);
}

void IkControlPanel::updateUi()
{
  if (!toggle_button_ || !indicator_label_ || !status_label_) {
    return;
  }

  if (ik_enabled_) {
    toggle_button_->setText("ik逆解结束");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; font-size: 16px; }"
      "QPushButton:hover { background-color: #276c2b; }");
    indicator_label_->setStyleSheet(
      "background-color: #18c24a; border-radius: 12px; border: 2px solid #0c7a2b;");
    status_label_->setText("IK override: ON");
    status_label_->setStyleSheet("padding: 6px; background-color: #d4edda; color: #155724;");
  } else {
    toggle_button_->setText("ik逆解开始");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #b71c1c; color: white; font-weight: bold; font-size: 16px; }"
      "QPushButton:hover { background-color: #9f1818; }");
    indicator_label_->setStyleSheet(
      "background-color: #d71920; border-radius: 12px; border: 2px solid #8f1014;");
    status_label_->setText("IK override: OFF");
    status_label_->setStyleSheet("padding: 6px; background-color: #f8d7da; color: #721c24;");
  }
}

}  // namespace openarmx_ik_control_panel

PLUGINLIB_EXPORT_CLASS(openarmx_ik_control_panel::IkControlPanel, rviz_common::Panel)
