#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace openarmx_joint_slider_panel {

class JointSliderPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit JointSliderPanel(QWidget *parent = nullptr);
  ~JointSliderPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void onHandsUpClicked();
  void onPreviewClicked();
  void onHomeClicked();
  void onJointStepSliderChanged(int value);
  void onGripperStepSliderChanged(int value);
  void updateStatusText();

 private:
  struct SliderBinding {
    QSlider *slider{nullptr};
    QLabel *value_label{nullptr};
    bool is_gripper{false};
  };

  struct TargetState {
    std::vector<double> left_arm;
    std::vector<double> right_arm;
    double left_gripper{0.0};
    double right_gripper{0.0};
  };

  void setupUi();
  SliderBinding createJointSliderRow(const QString &title, QVBoxLayout *parent_layout,
                                     bool is_gripper, double init_value);
  void updateSliderLabel(const SliderBinding &binding);
  void setStatus(const QString &text, const QString &color_hex);
  void applyDefaultSliderRanges();
  bool loadJointLimitsFromRobotDescription();
  bool applyUrdfJointLimits(const std::string &urdf_xml);
  void scheduleLivePreview();
  void updateDesiredTargetFromSliders();
  void setHomeSliders();
  double jointStepRad() const;
  double gripperStepMeters() const;
  void publishPreviewTransforms();
  void startPreviewHold(std::chrono::milliseconds duration);
  void stopPreviewHold();
  void onPreviewPublishTimer();
  void startCommandWorker();
  void stopCommandWorker();
  void commandWorkerLoop();
  bool stepTowardsTarget(TargetState &current, const TargetState &target, double arm_step_rad,
                         double gripper_step_m) const;
  bool setupPreviewModel(const std::string &urdf_xml);
  void collectPreviewTransforms(const KDL::SegmentMap::const_iterator &segment_it,
                                const std::map<std::string, double> &joint_positions,
                                const rclcpp::Time &stamp, const std::string &frame_prefix,
                                const std::string &frame_separator,
                                std::vector<geometry_msgs::msg::TransformStamped> &out) const;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool hasAllTargetJointStates() const;
  TargetState targetStateFromLatestJointStates() const;
  bool applyJointStateToSliders();
  bool targetStatesApproxEqual(const TargetState &lhs, const TargetState &rhs) const;

  TargetState collectTargetStateFromSliders() const;

  bool sendWithForwardControllers(const TargetState &target);

  std::vector<double> buildForwardCommand(const std::vector<double> &arm_joints,
                                          double gripper) const;

  static constexpr int kArmSliderScale = 1000;      // slider unit: 0.001 rad
  static constexpr int kArmSliderMin = -3141;       // -3.141 rad
  static constexpr int kArmSliderMax = 3141;        // +3.141 rad
  static constexpr int kGripperSliderMinMm = 0;     // 0 mm
  static constexpr int kGripperSliderMaxMm = 44;    // 44 mm

  std::array<SliderBinding, 7> left_arm_sliders_;
  std::array<SliderBinding, 7> right_arm_sliders_;
  SliderBinding left_gripper_slider_;
  SliderBinding right_gripper_slider_;

  QPushButton *hands_up_button_{nullptr};
  QPushButton *home_button_{nullptr};
  QLabel *status_label_{nullptr};
  QSlider *joint_step_slider_{nullptr};
  QLabel *joint_step_value_label_{nullptr};
  QSlider *gripper_step_slider_{nullptr};
  QLabel *gripper_step_value_label_{nullptr};
  QLabel *preview_info_label_{nullptr};

  QTimer *ros_spin_timer_{nullptr};
  QTimer *status_timer_{nullptr};
  QTimer *preview_send_timer_{nullptr};
  QTimer *preview_publish_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_forward_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_forward_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> preview_tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::map<std::string, double> latest_joint_state_map_;
  bool has_joint_state_{false};
  bool auto_sync_done_{false};
  bool slider_interacting_{false};

  bool suppress_slider_events_{false};
  bool joint_limits_loaded_{false};
  std::string joint_limits_source_node_;

  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  std::thread command_worker_;
  bool command_worker_running_{false};
  bool command_state_initialized_{false};
  TargetState desired_target_;
  TargetState command_target_;
  int joint_step_mrad_{10};         // 0.010 rad per cycle
  int gripper_step_tenth_mm_{10};   // 1.0 mm per cycle
  bool live_preview_enabled_{true};
  std::chrono::steady_clock::time_point last_preview_send_time_;
  std::chrono::steady_clock::time_point preview_hold_until_time_;
  std::chrono::steady_clock::time_point preview_model_retry_time_;
  KDL::Tree preview_kdl_tree_;
  std::string preview_root_link_;
  std::string preview_anchor_frame_;
  bool preview_model_ready_{false};
  std::map<std::string, std::pair<std::string, std::pair<double, double>>> preview_mimic_map_;

  const std::vector<std::string> left_arm_joint_names_ = {
      "openarmx_left_joint1", "openarmx_left_joint2", "openarmx_left_joint3",
      "openarmx_left_joint4", "openarmx_left_joint5", "openarmx_left_joint6",
      "openarmx_left_joint7"};

  const std::vector<std::string> right_arm_joint_names_ = {
      "openarmx_right_joint1", "openarmx_right_joint2", "openarmx_right_joint3",
      "openarmx_right_joint4", "openarmx_right_joint5", "openarmx_right_joint6",
      "openarmx_right_joint7"};

  const std::string left_gripper_joint_name_ = "openarmx_left_finger_joint1";
  const std::string right_gripper_joint_name_ = "openarmx_right_finger_joint1";
};

}  // namespace openarmx_joint_slider_panel
