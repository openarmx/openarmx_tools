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

#ifndef openarmx_gripper_panel__GRIPPER_PANEL_HPP_
#define openarmx_gripper_panel__GRIPPER_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <QWidget>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>

namespace openarmx_gripper_panel
{

class GripperPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GripperPanel(QWidget* parent = nullptr);
  virtual ~GripperPanel();

  virtual void onInitialize() override;
  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onCloseClicked();
  void onHalfClicked();
  void onOpenClicked();
  void onApplyClicked();
  void onSliderChanged(int value);
  void onGripperSelectionChanged(int index);

private:
  void createLayout();
  void sendGripperCommand(const std::string& side, double position);
  std::string getSelectedGripper() const;

  // ROS2 interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr left_gripper_client_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr right_gripper_client_;

  // UI components
  QComboBox* gripper_selector_;
  QSlider* position_slider_;
  QLabel* value_label_;
  QLabel* status_label_;

  QPushButton* close_btn_;
  QPushButton* half_btn_;
  QPushButton* open_btn_;
  QPushButton* apply_btn_;

  // Constants
  static constexpr double GRIPPER_CLOSED = 0.0;
  static constexpr double GRIPPER_OPEN = 0.044;
  static constexpr double GRIPPER_HALF = 0.022;
  static constexpr int SLIDER_MAX = 44;  // 0-44mm
};

}  // namespace openarmx_gripper_panel

#endif  // openarmx_gripper_panel__GRIPPER_PANEL_HPP_
