// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
// 4.0 International License (CC BY-NC-SA 4.0).

#ifndef OPENARMX_IK_CONTROL_PANEL__IK_CONTROL_PANEL_HPP_
#define OPENARMX_IK_CONTROL_PANEL__IK_CONTROL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>

#include <QLabel>
#include <QPushButton>
#include <QWidget>

namespace openarmx_ik_control_panel
{

class IkControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit IkControlPanel(QWidget * parent = nullptr);
  ~IkControlPanel() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onToggleClicked();

private:
  void createLayout();
  void publishState();
  void setEnabledState(bool enabled);
  void updateUi();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;

  QPushButton * toggle_button_;
  QLabel * indicator_label_;
  QLabel * status_label_;

  bool ik_enabled_{false};
};

}  // namespace openarmx_ik_control_panel

#endif  // OPENARMX_IK_CONTROL_PANEL__IK_CONTROL_PANEL_HPP_
