#include "openarmx_joint_slider_panel/joint_slider_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>
#include <QFontDatabase>
#include <QFontMetrics>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <urdf/model.h>

using namespace std::chrono_literals;

namespace openarmx_joint_slider_panel {

JointSliderPanel::JointSliderPanel(QWidget *parent) : rviz_common::Panel(parent) {
  setupUi();
}

JointSliderPanel::~JointSliderPanel() {
  stopCommandWorker();
}

void JointSliderPanel::setupUi() {
  auto *root_layout = new QVBoxLayout;

  auto *title = new QLabel("<h3>OpenArmX 关节滑块面板</h3>");
  title->setAlignment(Qt::AlignCenter);
  root_layout->addWidget(title);

  auto *scroll_area = new QScrollArea;
  scroll_area->setWidgetResizable(true);
  auto *scroll_content = new QWidget;
  auto *scroll_layout = new QVBoxLayout;

  auto *left_group = new QGroupBox("左臂 (7关节 + 夹爪)");
  auto *left_layout = new QVBoxLayout;
  for (size_t i = 0; i < left_arm_sliders_.size(); ++i) {
    const QString name = QString("左关节 %1").arg(static_cast<int>(i + 1));
    left_arm_sliders_[i] = createJointSliderRow(name, left_layout, false, 0.0);
  }
  left_gripper_slider_ = createJointSliderRow("左夹爪 (mm)", left_layout, true, 0.0);
  left_group->setLayout(left_layout);
  scroll_layout->addWidget(left_group);

  auto *right_group = new QGroupBox("右臂 (7关节 + 夹爪)");
  auto *right_layout = new QVBoxLayout;
  for (size_t i = 0; i < right_arm_sliders_.size(); ++i) {
    const QString name = QString("右关节 %1").arg(static_cast<int>(i + 1));
    right_arm_sliders_[i] = createJointSliderRow(name, right_layout, false, 0.0);
  }
  right_gripper_slider_ = createJointSliderRow("右夹爪 (mm)", right_layout, true, 0.0);
  right_group->setLayout(right_layout);
  scroll_layout->addWidget(right_group);

  scroll_layout->addStretch();
  scroll_content->setLayout(scroll_layout);
  scroll_area->setWidget(scroll_content);
  root_layout->addWidget(scroll_area);

  auto *button_row = new QHBoxLayout;
  hands_up_button_ = new QPushButton("Hands Up");
  home_button_ = new QPushButton("Home 回零");

  button_row->addWidget(hands_up_button_);
  button_row->addWidget(home_button_);
  root_layout->addLayout(button_row);

  auto *joint_step_row = new QHBoxLayout;
  joint_step_row->addWidget(new QLabel("关节步长:"));
  joint_step_slider_ = new QSlider(Qt::Horizontal);
  joint_step_slider_->setRange(1, 200);  // 0.001~0.200 rad per cycle
  joint_step_slider_->setValue(joint_step_mrad_);
  joint_step_slider_->setTickInterval(10);
  joint_step_slider_->setTickPosition(QSlider::TicksBelow);
  joint_step_value_label_ = new QLabel(QString("%1 mrad").arg(joint_step_mrad_));
  joint_step_value_label_->setMinimumWidth(90);
  joint_step_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  joint_step_row->addWidget(joint_step_slider_, 1);
  joint_step_row->addWidget(joint_step_value_label_);
  root_layout->addLayout(joint_step_row);

  auto *gripper_step_row = new QHBoxLayout;
  gripper_step_row->addWidget(new QLabel("夹爪步长:"));
  gripper_step_slider_ = new QSlider(Qt::Horizontal);
  gripper_step_slider_->setRange(1, 100);  // 0.1~10.0 mm per cycle
  gripper_step_slider_->setValue(gripper_step_tenth_mm_);
  gripper_step_slider_->setTickInterval(5);
  gripper_step_slider_->setTickPosition(QSlider::TicksBelow);
  gripper_step_value_label_ = new QLabel(
      QString("%1 mm").arg(static_cast<double>(gripper_step_tenth_mm_) / 10.0, 0, 'f', 1));
  gripper_step_value_label_->setMinimumWidth(90);
  gripper_step_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  gripper_step_row->addWidget(gripper_step_slider_, 1);
  gripper_step_row->addWidget(gripper_step_value_label_);
  root_layout->addLayout(gripper_step_row);

  status_label_ = new QLabel("状态: 等待初始化...");
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet("background-color: #f0f0f0; padding: 6px; border-radius: 4px;");
  root_layout->addWidget(status_label_);

  setLayout(root_layout);

  connect(hands_up_button_, &QPushButton::clicked, this, &JointSliderPanel::onHandsUpClicked);
  connect(home_button_, &QPushButton::clicked, this, &JointSliderPanel::onHomeClicked);
  connect(joint_step_slider_, &QSlider::valueChanged, this, &JointSliderPanel::onJointStepSliderChanged);
  connect(gripper_step_slider_, &QSlider::valueChanged, this,
          &JointSliderPanel::onGripperStepSliderChanged);
}

JointSliderPanel::SliderBinding JointSliderPanel::createJointSliderRow(const QString &title,
                                                                       QVBoxLayout *parent_layout,
                                                                       bool is_gripper,
                                                                       double init_value) {
  auto *row_layout = new QHBoxLayout;
  auto *name_label = new QLabel(title);
  name_label->setMinimumWidth(120);

  auto *slider = new QSlider(Qt::Horizontal);
  slider->setTickPosition(QSlider::TicksBelow);

  if (is_gripper) {
    slider->setRange(kGripperSliderMinMm, kGripperSliderMaxMm);
    slider->setTickInterval(2);
    slider->setValue(static_cast<int>(std::round(init_value * 1000.0)));
  } else {
    slider->setRange(kArmSliderMin, kArmSliderMax);
    slider->setTickInterval(300);
    slider->setValue(static_cast<int>(std::round(init_value * kArmSliderScale)));
  }

  auto *value_label = new QLabel;
  value_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  value_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

  // Use monospaced digits and fixed label width to avoid layout jitter while dragging.
  auto mono_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
  value_label->setFont(mono_font);
  QFontMetrics fm(mono_font);
  if (is_gripper) {
    const int width = fm.horizontalAdvance("00 mm (0.044 m)") + 14;
    value_label->setFixedWidth(width);
  } else {
    const int width = fm.horizontalAdvance("+000.0°") + 14;
    value_label->setFixedWidth(width);
  }

  row_layout->addWidget(name_label);
  row_layout->addWidget(slider);
  row_layout->addWidget(value_label);
  parent_layout->addLayout(row_layout);

  SliderBinding binding;
  binding.slider = slider;
  binding.value_label = value_label;
  binding.is_gripper = is_gripper;

  connect(slider, &QSlider::valueChanged, this, [this, binding](int) {
    updateSliderLabel(binding);
    if (!suppress_slider_events_) {
      updateDesiredTargetFromSliders();
    }
  });
  connect(slider, &QSlider::sliderPressed, this, [this]() { slider_interacting_ = true; });
  connect(slider, &QSlider::sliderReleased, this, [this]() { slider_interacting_ = false; });

  updateSliderLabel(binding);
  return binding;
}

void JointSliderPanel::updateSliderLabel(const SliderBinding &binding) {
  if (!binding.slider || !binding.value_label) {
    return;
  }

  if (binding.is_gripper) {
    const int mm = binding.slider->value();
    const double meters = static_cast<double>(mm) / 1000.0;
    binding.value_label->setText(QString::asprintf("%02d mm (%0.3f m)", mm, meters));
  } else {
    const double rad = static_cast<double>(binding.slider->value()) / static_cast<double>(kArmSliderScale);
    const double deg = rad * 57.29577951308232;
    binding.value_label->setText(QString::asprintf("%+0.1f°", deg));
  }
}

void JointSliderPanel::applyDefaultSliderRanges() {
  for (auto &binding : left_arm_sliders_) {
    binding.slider->setRange(kArmSliderMin, kArmSliderMax);
    updateSliderLabel(binding);
  }
  for (auto &binding : right_arm_sliders_) {
    binding.slider->setRange(kArmSliderMin, kArmSliderMax);
    updateSliderLabel(binding);
  }

  left_gripper_slider_.slider->setRange(kGripperSliderMinMm, kGripperSliderMaxMm);
  right_gripper_slider_.slider->setRange(kGripperSliderMinMm, kGripperSliderMaxMm);
  updateSliderLabel(left_gripper_slider_);
  updateSliderLabel(right_gripper_slider_);
}

bool JointSliderPanel::applyUrdfJointLimits(const std::string &urdf_xml) {
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    return false;
  }

  auto apply_arm_joint_limit = [this, &model](SliderBinding &binding, const std::string &joint_name) {
    auto joint = model.getJoint(joint_name);
    if (!joint || !joint->limits) {
      return false;
    }
    const double lower = joint->limits->lower;
    const double upper = joint->limits->upper;
    int min_raw = static_cast<int>(std::lround(lower * kArmSliderScale));
    int max_raw = static_cast<int>(std::lround(upper * kArmSliderScale));
    if (min_raw > max_raw) {
      std::swap(min_raw, max_raw);
    }
    if (min_raw == max_raw) {
      max_raw += 1;
    }
    binding.slider->setRange(min_raw, max_raw);
    return true;
  };

  auto apply_gripper_joint_limit = [this, &model](SliderBinding &binding, const std::string &joint_name) {
    auto joint = model.getJoint(joint_name);
    if (!joint || !joint->limits) {
      return false;
    }
    const double lower = joint->limits->lower;
    const double upper = joint->limits->upper;
    int min_mm = static_cast<int>(std::lround(lower * 1000.0));
    int max_mm = static_cast<int>(std::lround(upper * 1000.0));
    if (min_mm > max_mm) {
      std::swap(min_mm, max_mm);
    }
    if (min_mm == max_mm) {
      max_mm += 1;
    }
    binding.slider->setRange(min_mm, max_mm);
    return true;
  };

  bool all_ok = true;
  for (size_t i = 0; i < left_arm_sliders_.size(); ++i) {
    all_ok = apply_arm_joint_limit(left_arm_sliders_[i], left_arm_joint_names_[i]) && all_ok;
  }
  for (size_t i = 0; i < right_arm_sliders_.size(); ++i) {
    all_ok = apply_arm_joint_limit(right_arm_sliders_[i], right_arm_joint_names_[i]) && all_ok;
  }

  all_ok = apply_gripper_joint_limit(left_gripper_slider_, left_gripper_joint_name_) && all_ok;
  all_ok = apply_gripper_joint_limit(right_gripper_slider_, right_gripper_joint_name_) && all_ok;

  for (const auto &binding : left_arm_sliders_) {
    updateSliderLabel(binding);
  }
  for (const auto &binding : right_arm_sliders_) {
    updateSliderLabel(binding);
  }
  updateSliderLabel(left_gripper_slider_);
  updateSliderLabel(right_gripper_slider_);

  return all_ok;
}

bool JointSliderPanel::loadJointLimitsFromRobotDescription() {
  std::vector<std::string> candidate_nodes = {
      "/controller_manager", "/robot_state_publisher", "/move_group"};

  // Also probe namespaced instances to avoid hard dependency on global node names.
  if (node_) {
    std::set<std::string> candidate_set(candidate_nodes.begin(), candidate_nodes.end());
    const auto graph_nodes = node_->get_node_graph_interface()->get_node_names_and_namespaces();
    for (const auto &entry : graph_nodes) {
      const std::string &name = entry.first;
      const std::string &ns = entry.second;
      if (name.find("controller_manager") == std::string::npos &&
          name.find("robot_state_publisher") == std::string::npos &&
          name.find("move_group") == std::string::npos) {
        continue;
      }

      std::string full_name = ns;
      if (full_name.empty()) {
        full_name = "/";
      }
      if (full_name.back() != '/') {
        full_name.push_back('/');
      }
      full_name += name;
      candidate_set.insert(full_name);
    }
    candidate_nodes.assign(candidate_set.begin(), candidate_set.end());
  }

  for (const auto &node_name : candidate_nodes) {
    try {
      auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node_, node_name);
      if (!param_client->wait_for_service(700ms)) {
        continue;
      }

      const auto params = param_client->get_parameters({"robot_description"});
      if (params.empty()) {
        continue;
      }
      if (params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        continue;
      }

      const std::string urdf_xml = params[0].as_string();
      if (urdf_xml.empty()) {
        continue;
      }

      setupPreviewModel(urdf_xml);

      if (applyUrdfJointLimits(urdf_xml)) {
        joint_limits_loaded_ = true;
        joint_limits_source_node_ = node_name;
        return true;
      }
    } catch (const std::exception &) {
      continue;
    }
  }
  return false;
}

void JointSliderPanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("openarmx_joint_slider_panel");

  left_forward_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/left_forward_position_controller/commands", 10);
  right_forward_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/right_forward_position_controller/commands", 10);

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 30,
      std::bind(&JointSliderPanel::jointStateCallback, this, std::placeholders::_1));

  // Fallback robot description source for cases where parameter services are namespaced
  // or unavailable when panel initializes.
  rclcpp::QoS description_qos(rclcpp::KeepLast(1));
  description_qos.reliable();
  description_qos.transient_local();
  robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", description_qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg || msg->data.empty()) {
          return;
        }

        setupPreviewModel(msg->data);

        if (applyUrdfJointLimits(msg->data)) {
          joint_limits_loaded_ = true;
          joint_limits_source_node_ = "/robot_description(topic)";
        }
      });

  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  ros_spin_timer_->start(50);

  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &JointSliderPanel::updateStatusText);
  status_timer_->start(1000);

  preview_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  preview_send_timer_ = new QTimer(this);
  preview_send_timer_->setSingleShot(true);
  connect(preview_send_timer_, &QTimer::timeout, this, &JointSliderPanel::onPreviewClicked);

  preview_publish_timer_ = new QTimer(this);
  connect(preview_publish_timer_, &QTimer::timeout, this, &JointSliderPanel::onPreviewPublishTimer);
  preview_publish_timer_->start(50);

  live_preview_enabled_ = true;
  last_preview_send_time_ = std::chrono::steady_clock::now();
  preview_hold_until_time_ = std::chrono::steady_clock::time_point::min();
  preview_model_retry_time_ = std::chrono::steady_clock::now();

  applyDefaultSliderRanges();
  if (!loadJointLimitsFromRobotDescription()) {
    joint_limits_loaded_ = false;
    joint_limits_source_node_.clear();
  }

  updateDesiredTargetFromSliders();
  startCommandWorker();
  updateStatusText();
}

void JointSliderPanel::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  latest_joint_state_map_.clear();
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    latest_joint_state_map_[msg->name[i]] = msg->position[i];
  }
  has_joint_state_ = (n > 0);

  if (hasAllTargetJointStates()) {
    const TargetState current_state = targetStateFromLatestJointStates();
    bool should_track_live_state = false;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!command_state_initialized_) {
        command_target_ = current_state;
        desired_target_ = current_state;
        command_state_initialized_ = true;
        should_track_live_state = true;
      } else if (!slider_interacting_ && targetStatesApproxEqual(desired_target_, command_target_)) {
        command_target_ = current_state;
        desired_target_ = current_state;
        should_track_live_state = true;
      }
    }

    if (should_track_live_state) {
      applyJointStateToSliders();
    }

    command_cv_.notify_all();

    if (!auto_sync_done_) {
      auto_sync_done_ = true;
      setStatus("已启用 /joint_states 实时同步显示", "#E3F2FD");
    }
    scheduleLivePreview();
  }
}

bool JointSliderPanel::hasAllTargetJointStates() const {
  auto has_name = [this](const std::string &name) {
    return latest_joint_state_map_.find(name) != latest_joint_state_map_.end();
  };

  for (const auto &j : left_arm_joint_names_) {
    if (!has_name(j)) {
      return false;
    }
  }
  for (const auto &j : right_arm_joint_names_) {
    if (!has_name(j)) {
      return false;
    }
  }
  return has_name(left_gripper_joint_name_) && has_name(right_gripper_joint_name_);
}

JointSliderPanel::TargetState JointSliderPanel::targetStateFromLatestJointStates() const {
  TargetState target;
  target.left_arm.reserve(left_arm_joint_names_.size());
  target.right_arm.reserve(right_arm_joint_names_.size());

  for (const auto &name : left_arm_joint_names_) {
    target.left_arm.push_back(latest_joint_state_map_.at(name));
  }
  for (const auto &name : right_arm_joint_names_) {
    target.right_arm.push_back(latest_joint_state_map_.at(name));
  }

  target.left_gripper = latest_joint_state_map_.at(left_gripper_joint_name_);
  target.right_gripper = latest_joint_state_map_.at(right_gripper_joint_name_);
  return target;
}

bool JointSliderPanel::applyJointStateToSliders() {
  if (!has_joint_state_ || !hasAllTargetJointStates()) {
    return false;
  }

  suppress_slider_events_ = true;

  for (size_t i = 0; i < left_arm_sliders_.size(); ++i) {
    const double rad = latest_joint_state_map_.at(left_arm_joint_names_[i]);
    const int raw = static_cast<int>(std::lround(rad * kArmSliderScale));
    left_arm_sliders_[i].slider->setValue(std::clamp(
        raw, left_arm_sliders_[i].slider->minimum(), left_arm_sliders_[i].slider->maximum()));
  }

  for (size_t i = 0; i < right_arm_sliders_.size(); ++i) {
    const double rad = latest_joint_state_map_.at(right_arm_joint_names_[i]);
    const int raw = static_cast<int>(std::lround(rad * kArmSliderScale));
    right_arm_sliders_[i].slider->setValue(std::clamp(
        raw, right_arm_sliders_[i].slider->minimum(), right_arm_sliders_[i].slider->maximum()));
  }

  const int left_mm = static_cast<int>(std::lround(latest_joint_state_map_.at(left_gripper_joint_name_) * 1000.0));
  const int right_mm = static_cast<int>(std::lround(latest_joint_state_map_.at(right_gripper_joint_name_) * 1000.0));

  left_gripper_slider_.slider->setValue(std::clamp(
      left_mm, left_gripper_slider_.slider->minimum(), left_gripper_slider_.slider->maximum()));
  right_gripper_slider_.slider->setValue(std::clamp(
      right_mm, right_gripper_slider_.slider->minimum(), right_gripper_slider_.slider->maximum()));

  suppress_slider_events_ = false;
  return true;
}

void JointSliderPanel::onHandsUpClicked() {
  if (!node_) {
    setStatus("面板尚未初始化", "#FFCDD2");
    return;
  }

  suppress_slider_events_ = true;

  for (size_t i = 0; i < left_arm_sliders_.size(); ++i) {
    const double target = (i == 3) ? 1.8 : 0.0;
    const int raw = static_cast<int>(std::lround(target * kArmSliderScale));
    left_arm_sliders_[i].slider->setValue(std::clamp(
        raw, left_arm_sliders_[i].slider->minimum(), left_arm_sliders_[i].slider->maximum()));
  }

  for (size_t i = 0; i < right_arm_sliders_.size(); ++i) {
    const double target = (i == 3) ? 1.8 : 0.0;
    const int raw = static_cast<int>(std::lround(target * kArmSliderScale));
    right_arm_sliders_[i].slider->setValue(std::clamp(
        raw, right_arm_sliders_[i].slider->minimum(), right_arm_sliders_[i].slider->maximum()));
  }

  left_gripper_slider_.slider->setValue(
      std::clamp(0, left_gripper_slider_.slider->minimum(), left_gripper_slider_.slider->maximum()));
  right_gripper_slider_.slider->setValue(
      std::clamp(0, right_gripper_slider_.slider->minimum(), right_gripper_slider_.slider->maximum()));

  suppress_slider_events_ = false;
  updateDesiredTargetFromSliders();
  setStatus("已设置 Hands Up 目标位，正在按步长分段执行", "#E3F2FD");
}

void JointSliderPanel::onJointStepSliderChanged(int value) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  joint_step_mrad_ = std::clamp(value, 1, 200);
  if (joint_step_value_label_) {
    joint_step_value_label_->setText(QString("%1 mrad").arg(joint_step_mrad_));
  }
  command_cv_.notify_all();
}

void JointSliderPanel::onGripperStepSliderChanged(int value) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  gripper_step_tenth_mm_ = std::clamp(value, 1, 100);
  if (gripper_step_value_label_) {
    gripper_step_value_label_->setText(
        QString("%1 mm").arg(static_cast<double>(gripper_step_tenth_mm_) / 10.0, 0, 'f', 1));
  }
  command_cv_.notify_all();
}

double JointSliderPanel::jointStepRad() const {
  return static_cast<double>(std::clamp(joint_step_mrad_, 1, 200)) / 1000.0;
}

double JointSliderPanel::gripperStepMeters() const {
  return static_cast<double>(std::clamp(gripper_step_tenth_mm_, 1, 100)) / 10000.0;
}

void JointSliderPanel::setHomeSliders() {
  suppress_slider_events_ = true;

  for (auto &binding : left_arm_sliders_) {
    binding.slider->setValue(std::clamp(0, binding.slider->minimum(), binding.slider->maximum()));
  }
  for (auto &binding : right_arm_sliders_) {
    binding.slider->setValue(std::clamp(0, binding.slider->minimum(), binding.slider->maximum()));
  }
  left_gripper_slider_.slider->setValue(
      std::clamp(0, left_gripper_slider_.slider->minimum(), left_gripper_slider_.slider->maximum()));
  right_gripper_slider_.slider->setValue(
      std::clamp(0, right_gripper_slider_.slider->minimum(), right_gripper_slider_.slider->maximum()));

  suppress_slider_events_ = false;
}

void JointSliderPanel::onHomeClicked() {
  if (!node_) {
    setStatus("面板尚未初始化", "#FFCDD2");
    return;
  }

  setHomeSliders();
  updateDesiredTargetFromSliders();
  setStatus("已设置 Home 目标位，正在按步长分段执行", "#E3F2FD");
}

void JointSliderPanel::scheduleLivePreview() {
  if (!live_preview_enabled_ || !preview_send_timer_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_preview_send_time_ > 40ms && !preview_send_timer_->isActive()) {
    onPreviewClicked();
    return;
  }

  preview_send_timer_->start(40);
}

void JointSliderPanel::onPreviewClicked() {
  publishPreviewTransforms();
}

bool JointSliderPanel::setupPreviewModel(const std::string &urdf_xml) {
  if (urdf_xml.empty()) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(), "setupPreviewModel skipped: empty URDF");
    }
    return false;
  }
  if (!kdl_parser::treeFromString(urdf_xml, preview_kdl_tree_)) {
    preview_model_ready_ = false;
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "setupPreviewModel failed: kdl_parser::treeFromString failed");
    }
    return false;
  }

  const auto root_it = preview_kdl_tree_.getRootSegment();
  if (root_it == preview_kdl_tree_.getSegments().end()) {
    preview_model_ready_ = false;
    if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "setupPreviewModel failed: root segment not found");
    }
    return false;
  }
  preview_root_link_ = root_it->first;
  preview_anchor_frame_ = preview_root_link_;

  preview_mimic_map_.clear();
  urdf::Model model;
  if (model.initString(urdf_xml)) {
    for (const auto &entry : model.joints_) {
      const auto &joint = entry.second;
      if (!joint || !joint->mimic) {
        continue;
      }
      preview_mimic_map_[entry.first] = {
          joint->mimic->joint_name,
          {joint->mimic->multiplier, joint->mimic->offset}};
    }
  }

  preview_model_ready_ = true;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "setupPreviewModel ok: root=%s, segments=%u",
                preview_root_link_.c_str(), preview_kdl_tree_.getNrOfSegments());
  }
  return true;
}

void JointSliderPanel::collectPreviewTransforms(
    const KDL::SegmentMap::const_iterator &segment_it,
    const std::map<std::string, double> &joint_positions, const rclcpp::Time &stamp,
    const std::string &frame_prefix, const std::string &frame_separator,
    std::vector<geometry_msgs::msg::TransformStamped> &out) const {
  for (const auto &child_pair : segment_it->second.children) {
    const auto &child_it = child_pair;
    const std::string &child_name = child_it->first;
    const KDL::Segment &segment = child_it->second.segment;
    const KDL::Joint &joint = segment.getJoint();

    double q = 0.0;
    if (joint.getType() != KDL::Joint::None) {
      const auto pos_it = joint_positions.find(joint.getName());
      if (pos_it != joint_positions.end()) {
        q = pos_it->second;
      }
    }

    const KDL::Frame edge = segment.pose(q);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = frame_prefix + frame_separator + segment_it->first;
    tf.child_frame_id = frame_prefix + frame_separator + child_name;
    tf.transform.translation.x = edge.p.x();
    tf.transform.translation.y = edge.p.y();
    tf.transform.translation.z = edge.p.z();
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    edge.M.GetQuaternion(qx, qy, qz, qw);
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    out.push_back(tf);

    collectPreviewTransforms(child_it, joint_positions, stamp, frame_prefix, frame_separator, out);
  }
}

void JointSliderPanel::publishPreviewTransforms() {
  if (!node_ || !live_preview_enabled_ || !preview_tf_broadcaster_ || !preview_model_ready_) {
    return;
  }

  const auto root_it = preview_kdl_tree_.getRootSegment();
  if (root_it == preview_kdl_tree_.getSegments().end()) {
    return;
  }

  const TargetState preview_target = collectTargetStateFromSliders();
  std::map<std::string, double> joint_positions;

  for (size_t i = 0; i < left_arm_joint_names_.size() && i < preview_target.left_arm.size(); ++i) {
    joint_positions[left_arm_joint_names_[i]] = preview_target.left_arm[i];
  }
  for (size_t i = 0; i < right_arm_joint_names_.size() && i < preview_target.right_arm.size(); ++i) {
    joint_positions[right_arm_joint_names_[i]] = preview_target.right_arm[i];
  }
  joint_positions[left_gripper_joint_name_] = preview_target.left_gripper;
  joint_positions[right_gripper_joint_name_] = preview_target.right_gripper;

  for (const auto &entry : preview_mimic_map_) {
    const auto source_it = joint_positions.find(entry.second.first);
    if (source_it == joint_positions.end()) {
      continue;
    }
    const double multiplier = entry.second.second.first;
    const double offset = entry.second.second.second;
    joint_positions[entry.first] = source_it->second * multiplier + offset;
  }

  const rclcpp::Time stamp = node_->get_clock()->now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve((preview_kdl_tree_.getNrOfSegments() + 1) * 2);

  auto append_preview_set = [&](const std::string &prefix, const std::string &separator) {
    geometry_msgs::msg::TransformStamped anchor_tf;
    anchor_tf.header.stamp = stamp;
    anchor_tf.header.frame_id = "world";
    anchor_tf.child_frame_id = prefix + separator + preview_root_link_;
    anchor_tf.transform.translation.x = 0.0;
    anchor_tf.transform.translation.y = 0.0;
    anchor_tf.transform.translation.z = 0.0;
    anchor_tf.transform.rotation.x = 0.0;
    anchor_tf.transform.rotation.y = 0.0;
    anchor_tf.transform.rotation.z = 0.0;
    anchor_tf.transform.rotation.w = 1.0;
    transforms.push_back(anchor_tf);

    collectPreviewTransforms(root_it, joint_positions, stamp, prefix, separator, transforms);
  };

  // Publish both naming styles to match different TF Prefix concatenation behaviors in RViz.
  append_preview_set("preview", "_");   // preview_world
  append_preview_set("preview_", "/");  // preview_/world

  if (!transforms.empty()) {
    preview_tf_broadcaster_->sendTransform(transforms);
    last_preview_send_time_ = std::chrono::steady_clock::now();
  }
}

void JointSliderPanel::startPreviewHold(std::chrono::milliseconds duration) {
  preview_hold_until_time_ = std::chrono::steady_clock::now() + duration;
}

void JointSliderPanel::stopPreviewHold() {
  preview_hold_until_time_ = std::chrono::steady_clock::time_point::min();
}

void JointSliderPanel::onPreviewPublishTimer() {
  if (!live_preview_enabled_) {
    return;
  }

  if (preview_model_ready_) {
    publishPreviewTransforms();
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < preview_model_retry_time_) {
    return;
  }
  preview_model_retry_time_ = now + 2s;
  loadJointLimitsFromRobotDescription();
}

JointSliderPanel::TargetState JointSliderPanel::collectTargetStateFromSliders() const {
  TargetState target;
  target.left_arm.reserve(7);
  target.right_arm.reserve(7);

  for (const auto &binding : left_arm_sliders_) {
    target.left_arm.push_back(static_cast<double>(binding.slider->value()) / kArmSliderScale);
  }
  for (const auto &binding : right_arm_sliders_) {
    target.right_arm.push_back(static_cast<double>(binding.slider->value()) / kArmSliderScale);
  }

  target.left_gripper = static_cast<double>(left_gripper_slider_.slider->value()) / 1000.0;
  target.right_gripper = static_cast<double>(right_gripper_slider_.slider->value()) / 1000.0;

  return target;
}

void JointSliderPanel::updateDesiredTargetFromSliders() {
  const TargetState target = collectTargetStateFromSliders();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    desired_target_ = target;
  }
  command_cv_.notify_all();
  scheduleLivePreview();
}

void JointSliderPanel::startCommandWorker() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (command_worker_running_) {
    return;
  }
  command_worker_running_ = true;
  command_state_initialized_ = false;
  command_worker_ = std::thread(&JointSliderPanel::commandWorkerLoop, this);
}

void JointSliderPanel::stopCommandWorker() {
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_worker_running_) {
      return;
    }
    command_worker_running_ = false;
  }
  command_cv_.notify_all();
  if (command_worker_.joinable()) {
    command_worker_.join();
  }
}

bool JointSliderPanel::stepTowardsTarget(TargetState &current, const TargetState &target,
                                         double arm_step_rad, double gripper_step_m) const {
  auto step_scalar = [](double &value, double target_value, double step) {
    const double delta = target_value - value;
    if (std::abs(delta) <= step) {
      value = target_value;
      return std::abs(delta) > 1e-9;
    }
    value += (delta > 0.0 ? step : -step);
    return true;
  };

  bool moved = false;
  for (size_t i = 0; i < current.left_arm.size() && i < target.left_arm.size(); ++i) {
    moved = step_scalar(current.left_arm[i], target.left_arm[i], arm_step_rad) || moved;
  }
  for (size_t i = 0; i < current.right_arm.size() && i < target.right_arm.size(); ++i) {
    moved = step_scalar(current.right_arm[i], target.right_arm[i], arm_step_rad) || moved;
  }
  moved = step_scalar(current.left_gripper, target.left_gripper, gripper_step_m) || moved;
  moved = step_scalar(current.right_gripper, target.right_gripper, gripper_step_m) || moved;
  return moved;
}

bool JointSliderPanel::targetStatesApproxEqual(const TargetState &lhs, const TargetState &rhs) const {
  constexpr double kArmEpsilon = 1e-6;
  constexpr double kGripperEpsilon = 1e-6;

  if (lhs.left_arm.size() != rhs.left_arm.size() || lhs.right_arm.size() != rhs.right_arm.size()) {
    return false;
  }

  for (size_t i = 0; i < lhs.left_arm.size(); ++i) {
    if (std::abs(lhs.left_arm[i] - rhs.left_arm[i]) > kArmEpsilon) {
      return false;
    }
  }
  for (size_t i = 0; i < lhs.right_arm.size(); ++i) {
    if (std::abs(lhs.right_arm[i] - rhs.right_arm[i]) > kArmEpsilon) {
      return false;
    }
  }

  return std::abs(lhs.left_gripper - rhs.left_gripper) <= kGripperEpsilon &&
         std::abs(lhs.right_gripper - rhs.right_gripper) <= kGripperEpsilon;
}

void JointSliderPanel::commandWorkerLoop() {
  using clock = std::chrono::steady_clock;
  auto next_tick = clock::now();

  while (true) {
    TargetState desired;
    TargetState command;
    double arm_step = 0.02;
    double gripper_step = 0.001;
    bool initialized = false;
    bool running = false;

    {
      std::unique_lock<std::mutex> lock(command_mutex_);
      command_cv_.wait_until(lock, next_tick);
      running = command_worker_running_;
      if (!running) {
        break;
      }

      desired = desired_target_;
      command = command_target_;
      arm_step = jointStepRad();
      gripper_step = gripperStepMeters();
      initialized = command_state_initialized_;
    }

    bool moved = false;
    if (initialized) {
      moved = stepTowardsTarget(command, desired, arm_step, gripper_step);
    }

    if (moved) {
      sendWithForwardControllers(command);
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_target_ = command;
    }

    next_tick = clock::now() + 20ms;
  }
}

std::vector<double> JointSliderPanel::buildForwardCommand(const std::vector<double> &arm_joints,
                                                          double gripper) const {
  std::vector<double> cmd = arm_joints;
  cmd.push_back(gripper);
  return cmd;
}

bool JointSliderPanel::sendWithForwardControllers(const TargetState &target) {
  if (!left_forward_pub_ || !right_forward_pub_) {
    return false;
  }

  std_msgs::msg::Float64MultiArray left_msg;
  std_msgs::msg::Float64MultiArray right_msg;
  left_msg.data = buildForwardCommand(target.left_arm, target.left_gripper);
  right_msg.data = buildForwardCommand(target.right_arm, target.right_gripper);

  left_forward_pub_->publish(left_msg);
  right_forward_pub_->publish(right_msg);

  return true;
}

void JointSliderPanel::setStatus(const QString &text, const QString &color_hex) {
  if (!status_label_) {
    return;
  }
  status_label_->setText(text);
  status_label_->setStyleSheet(
      QString("background-color: %1; padding: 6px; border-radius: 4px;").arg(color_hex));
}

void JointSliderPanel::updateStatusText() {
  if (!node_) {
    return;
  }

  const QString joint_state_str = has_joint_state_ ? "joint_states: 正常" : "joint_states: 等待中";
  const QString limits_str =
      joint_limits_loaded_
          ? QString("限位: 来自 %1 的 URDF").arg(QString::fromStdString(joint_limits_source_node_))
          : "限位: 使用默认兜底值";
  const QString step_str =
      QString("步长: %1 mrad / 夹爪 %2 mm")
          .arg(joint_step_mrad_)
          .arg(static_cast<double>(gripper_step_tenth_mm_) / 10.0, 0, 'f', 1);
  setStatus(QString("控制: 滑块直控(分段执行) | %1 | %2 | %3 | 预览: 半透明实时(按滑块目标, 不受步长影响)")
                .arg(joint_state_str, limits_str, step_str),
            "#F0F0F0");
}

void JointSliderPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);

  int joint_step_mrad = joint_step_mrad_;
  if (config.mapGetInt("joint_step_mrad", &joint_step_mrad)) {
    joint_step_mrad_ = std::clamp(joint_step_mrad, 1, 200);
    if (joint_step_slider_) {
      joint_step_slider_->setValue(joint_step_mrad_);
    }
    if (joint_step_value_label_) {
      joint_step_value_label_->setText(QString("%1 mrad").arg(joint_step_mrad_));
    }
  }

  int gripper_step_tenth_mm = gripper_step_tenth_mm_;
  if (config.mapGetInt("gripper_step_tenth_mm", &gripper_step_tenth_mm)) {
    gripper_step_tenth_mm_ = std::clamp(gripper_step_tenth_mm, 1, 100);
    if (gripper_step_slider_) {
      gripper_step_slider_->setValue(gripper_step_tenth_mm_);
    }
    if (gripper_step_value_label_) {
      gripper_step_value_label_->setText(
          QString("%1 mm").arg(static_cast<double>(gripper_step_tenth_mm_) / 10.0, 0, 'f', 1));
    }
  }

  live_preview_enabled_ = true;
}

void JointSliderPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("joint_step_mrad", joint_step_mrad_);
  config.mapSetValue("gripper_step_tenth_mm", gripper_step_tenth_mm_);
}

}  // namespace openarmx_joint_slider_panel

PLUGINLIB_EXPORT_CLASS(openarmx_joint_slider_panel::JointSliderPanel, rviz_common::Panel)
