// SPDX-License-Identifier: Apache-2.0
#include "pure_odometry_bringup/localization_status_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/transformation/frame_transformer.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <QColor>
#include <QString>
#include <QVariant>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace pure_odometry_bringup
{
namespace
{
constexpr auto kAdapterDiagnosticName = "localization/localization_interface_adapter";
constexpr auto kRegistrationDiagnosticName = "localization/gyro_odometer";
constexpr auto kFusionDiagnosticName = "localization/gnss_map_odom_fusion";
constexpr double kRadiansToDegrees = 57.295779513082320876;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

QString number(double value, int precision)
{
  if (!std::isfinite(value)) {
    return "INVALID";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return QString::fromStdString(stream.str());
}

QColor toQColor(const Rgba & color)
{
  return QColor::fromRgbF(color.r, color.g, color.b, color.a);
}

std::optional<std::uint64_t> unsignedInteger(const std::string & value)
{
  std::uint64_t result = 0U;
  const char * const begin = value.data();
  const char * const end = begin + value.size();
  const auto parsed = std::from_chars(begin, end, result);
  if (parsed.ec != std::errc{} || parsed.ptr != end) {
    return std::nullopt;
  }
  return result;
}

using StatusLevel = rviz_common::properties::StatusProperty::Level;

StatusLevel interfaceLevel(InterfaceVisualState state)
{
  switch (state) {
    case InterfaceVisualState::ACTIVE:
      return rviz_common::properties::StatusProperty::Ok;
    case InterfaceVisualState::WAITING:
    case InterfaceVisualState::DEGRADED:
      return rviz_common::properties::StatusProperty::Warn;
    case InterfaceVisualState::STALE:
      return rviz_common::properties::StatusProperty::Error;
  }
  return rviz_common::properties::StatusProperty::Error;
}

StatusLevel registrationLevel(RegistrationVisualState state)
{
  switch (state) {
    case RegistrationVisualState::ACCEPTED:
      return rviz_common::properties::StatusProperty::Ok;
    case RegistrationVisualState::WAITING:
    case RegistrationVisualState::REJECTED:
      return rviz_common::properties::StatusProperty::Warn;
    case RegistrationVisualState::STALE:
      return rviz_common::properties::StatusProperty::Error;
  }
  return rviz_common::properties::StatusProperty::Error;
}

StatusLevel gnssLevel(GnssVisualState state)
{
  switch (state) {
    case GnssVisualState::TRACKING:
      return rviz_common::properties::StatusProperty::Ok;
    case GnssVisualState::WAITING:
    case GnssVisualState::OUTAGE:
    case GnssVisualState::REACQUIRING:
    case GnssVisualState::RECOVERING:
      return rviz_common::properties::StatusProperty::Warn;
    case GnssVisualState::ERROR:
    case GnssVisualState::STALE:
      return rviz_common::properties::StatusProperty::Error;
  }
  return rviz_common::properties::StatusProperty::Error;
}

}  // namespace

LocalizationStatusDisplay::LocalizationStatusDisplay()
{
  state_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Kinematic State Topic", "/localization/kinematic_state", "nav_msgs/msg/Odometry",
    "Autoware-compatible localization state used for pose, yaw and speed.",
    this, SLOT(updateSubscriptions()));
  diagnostics_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Diagnostics Topic", "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray",
    "Diagnostics used for interface, registration and GNSS state.",
    this, SLOT(updateSubscriptions()));
  map_frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Map Frame", "map", "Global frame used for the localization TF check.", this);
  base_frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Base Frame", "base_link", "Vehicle frame used for the localization TF check.", this);
  output_stale_property_ = new rviz_common::properties::FloatProperty(
    "Output Stale Time (s)", 1.0F,
    "Maximum localization-state age before output and TF are reported stale.", this);
  output_stale_property_->setMin(0.1F);
  diagnostic_stale_property_ = new rviz_common::properties::FloatProperty(
    "Diagnostic Stale Time (s)", 2.5F,
    "Maximum age of interface, registration and GNSS diagnostics.", this);
  diagnostic_stale_property_->setMin(0.2F);

  live_status_property_ = new rviz_common::properties::Property(
    "Live Status", QVariant(),
    "Read-only values derived from live localization, TF and diagnostic inputs.", this);
  interface_property_ = new rviz_common::properties::StringProperty(
    "Autoware Localization Interface", "WAITING", "Adapter diagnostic state.",
    live_status_property_);
  position_x_property_ = new rviz_common::properties::StringProperty(
    "Position X", "WAITING", "Current map-frame X position.", live_status_property_);
  position_y_property_ = new rviz_common::properties::StringProperty(
    "Position Y", "WAITING", "Current map-frame Y position.", live_status_property_);
  yaw_property_ = new rviz_common::properties::StringProperty(
    "Yaw", "WAITING", "Current map-frame heading.", live_status_property_);
  speed_property_ = new rviz_common::properties::StringProperty(
    "Speed", "WAITING", "Current horizontal speed.", live_status_property_);
  transform_property_ = new rviz_common::properties::StringProperty(
    "map -> base_link", "WAITING", "Availability and freshness of the localization TF.",
    live_status_property_);
  output_rate_property_ = new rviz_common::properties::StringProperty(
    "Output Rate", "WAITING",
    "Publisher rate derived from the adapter diagnostic's cumulative published_count and sim-time.",
    live_status_property_);
  registration_property_ = new rviz_common::properties::StringProperty(
    "Registration", "WAITING", "Latest LiDAR registration result and source/reason.",
    live_status_property_);
  gnss_property_ = new rviz_common::properties::StringProperty(
    "GNSS", "WAITING", "GNSS tracking/recovery state and fusion mode.",
    live_status_property_);
  gnss_color_property_ = new rviz_common::properties::ColorProperty(
    "GNSS State Color", toQColor(gnssColor(GnssVisualState::WAITING)),
    "Green=TRACKING, yellow=OUTAGE/REACQUIRING, blue=RECOVERING.",
    live_status_property_);

  for (auto * property : {
      static_cast<rviz_common::properties::Property *>(interface_property_),
      static_cast<rviz_common::properties::Property *>(position_x_property_),
      static_cast<rviz_common::properties::Property *>(position_y_property_),
      static_cast<rviz_common::properties::Property *>(yaw_property_),
      static_cast<rviz_common::properties::Property *>(speed_property_),
      static_cast<rviz_common::properties::Property *>(transform_property_),
      static_cast<rviz_common::properties::Property *>(output_rate_property_),
      static_cast<rviz_common::properties::Property *>(registration_property_),
      static_cast<rviz_common::properties::Property *>(gnss_property_),
      static_cast<rviz_common::properties::Property *>(gnss_color_property_)})
  {
    property->setReadOnly(true);
  }
}

void LocalizationStatusDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  state_topic_property_->initialize(rviz_ros_node_);
  diagnostics_topic_property_->initialize(rviz_ros_node_);
  map_frame_property_->setFrameManager(context_->getFrameManager());
  base_frame_property_->setFrameManager(context_->getFrameManager());
  showWaitingState();
}

void LocalizationStatusDisplay::onEnable()
{
  subscribe();
}

void LocalizationStatusDisplay::onDisable()
{
  unsubscribe();
  reset();
}

void LocalizationStatusDisplay::updateSubscriptions()
{
  if (!isEnabled()) {
    return;
  }
  unsubscribe();
  reset();
  subscribe();
}

void LocalizationStatusDisplay::subscribe()
{
  const auto node_interface = rviz_ros_node_.lock();
  if (!node_interface) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Subscriptions",
      "RViz ROS node is unavailable");
    return;
  }

  try {
    const auto node = node_interface->get_raw_node();
    state_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_property_->getTopicStd(),
      rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile(),
      std::bind(&LocalizationStatusDisplay::onState, this, std::placeholders::_1));
    diagnostic_subscription_ =
      node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_property_->getTopicStd(),
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile(),
      std::bind(
        &LocalizationStatusDisplay::onDiagnostics, this, std::placeholders::_1));
    setStatus(
      rviz_common::properties::StatusProperty::Ok, "Subscriptions",
      "Kinematic state and diagnostics subscribed");
  } catch (const std::exception & error) {
    state_subscription_.reset();
    diagnostic_subscription_.reset();
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Subscriptions",
      QString("Subscription failed: ") + error.what());
  }
}

void LocalizationStatusDisplay::unsubscribe()
{
  state_subscription_.reset();
  diagnostic_subscription_.reset();
}

void LocalizationStatusDisplay::onState(
  const nav_msgs::msg::Odometry::ConstSharedPtr & message)
{
  const double stamp_sec = stampSeconds(message->header.stamp);
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_state_ = *message;
  latest_state_received_at_ = std::chrono::steady_clock::now();
}

void LocalizationStatusDisplay::onDiagnostics(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & message)
{
  const auto received_at = std::chrono::steady_clock::now();
  const double source_stamp = stampSeconds(message->header.stamp);
  const std::optional<double> source_stamp_sec =
    std::isfinite(source_stamp) && source_stamp > 0.0 ?
    std::make_optional(source_stamp) : std::nullopt;

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & status : message->status) {
    DiagnosticSnapshot snapshot{status, received_at, source_stamp_sec};
    if (status.name == kAdapterDiagnosticName) {
      const auto published_count = diagnosticValue(status, "published_count");
      if (source_stamp_sec && published_count) {
        if (const auto count = unsignedInteger(*published_count)) {
          published_rate_estimator_.observe(*source_stamp_sec, *count);
        }
      }
      adapter_diagnostic_ = std::move(snapshot);
    } else if (status.name == kRegistrationDiagnosticName) {
      registration_diagnostic_ = std::move(snapshot);
    } else if (status.name == kFusionDiagnosticName) {
      fusion_diagnostic_ = std::move(snapshot);
    }
  }
}

bool LocalizationStatusDisplay::diagnosticIsFresh(
  const std::optional<DiagnosticSnapshot> & snapshot,
  const std::chrono::steady_clock::time_point & current,
  double ros_now_sec,
  double stale_limit_sec) const
{
  if (!snapshot) {
    return false;
  }
  if (snapshot->source_stamp_sec && std::isfinite(ros_now_sec) && ros_now_sec > 0.0) {
    return sourceTimeFresh(ros_now_sec, *snapshot->source_stamp_sec, stale_limit_sec);
  }
  return std::chrono::duration<double>(current - snapshot->received_at).count() <=
         stale_limit_sec;
}

void LocalizationStatusDisplay::update(float wall_dt, float)
{
  update_elapsed_sec_ += std::max(0.0F, wall_dt);
  if (update_elapsed_sec_ < 0.2) {
    return;
  }
  update_elapsed_sec_ = 0.0;

  std::optional<nav_msgs::msg::Odometry> state;
  std::chrono::steady_clock::time_point state_received_at;
  std::optional<DiagnosticSnapshot> adapter;
  std::optional<DiagnosticSnapshot> registration;
  std::optional<DiagnosticSnapshot> fusion;
  double output_rate_hz = 0.0;
  std::size_t output_rate_samples = 0U;
  const auto current = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = latest_state_;
    state_received_at = latest_state_received_at_;
    adapter = adapter_diagnostic_;
    registration = registration_diagnostic_;
    fusion = fusion_diagnostic_;
    output_rate_hz = published_rate_estimator_.rateHz();
    output_rate_samples = published_rate_estimator_.sampleCount();
  }

  double ros_now_sec = 0.0;
  if (const auto node_interface = rviz_ros_node_.lock()) {
    ros_now_sec = node_interface->get_raw_node()->now().seconds();
  }
  const double output_stale_sec = output_stale_property_->getFloat();
  const double diagnostic_stale_sec = diagnostic_stale_property_->getFloat();
  bool output_fresh = false;
  if (state) {
    const double state_stamp_sec = stampSeconds(state->header.stamp);
    output_fresh = (std::isfinite(ros_now_sec) && ros_now_sec > 0.0) ?
      sourceTimeFresh(ros_now_sec, state_stamp_sec, output_stale_sec) :
      std::chrono::duration<double>(current - state_received_at).count() <= output_stale_sec;
  }

  const bool adapter_fresh =
    diagnosticIsFresh(adapter, current, ros_now_sec, diagnostic_stale_sec);
  auto interface_state = classifyInterface(
    adapter ? &adapter->status : nullptr,
    adapter_fresh);
  interface_state = combineInterfaceWithOutput(
    interface_state, state.has_value(), output_fresh);
  const auto registration_state = classifyRegistration(
    registration ? &registration->status : nullptr,
    diagnosticIsFresh(registration, current, ros_now_sec, diagnostic_stale_sec));
  const auto gnss_state = classifyGnss(
    fusion ? &fusion->status : nullptr,
    diagnosticIsFresh(fusion, current, ros_now_sec, diagnostic_stale_sec));

  interface_property_->setString(interfaceLabel(interface_state));
  setStatus(
    interfaceLevel(interface_state), "Interface",
    QString("Autoware Localization Interface: ") + interfaceLabel(interface_state));

  if (state) {
    const auto & position = state->pose.pose.position;
    const auto & orientation = state->pose.pose.orientation;
    const auto & velocity = state->twist.twist.linear;
    const double yaw_deg = yawFromQuaternion(
      orientation.x, orientation.y, orientation.z, orientation.w) * kRadiansToDegrees;
    position_x_property_->setString(number(position.x, 2) + " m");
    position_y_property_->setString(number(position.y, 2) + " m");
    yaw_property_->setString(number(yaw_deg, 1) + " deg");
    speed_property_->setString(number(std::hypot(velocity.x, velocity.y), 2) + " m/s");
    output_rate_property_->setString(
      (!output_fresh || !adapter_fresh) ? "STALE" :
      (output_rate_samples >= 2U ? number(output_rate_hz, 1) + " Hz" : "CALCULATING"));
    setStatus(
      output_fresh ? rviz_common::properties::StatusProperty::Ok :
      rviz_common::properties::StatusProperty::Error,
      "Kinematic State", output_fresh ? "ACTIVE" : "STALE");
  } else {
    position_x_property_->setString("WAITING");
    position_y_property_->setString("WAITING");
    yaw_property_->setString("WAITING");
    speed_property_->setString("WAITING");
    output_rate_property_->setString("WAITING");
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "Kinematic State", "WAITING");
  }

  const std::string map_frame = map_frame_property_->getFrameStd();
  const std::string base_frame = base_frame_property_->getFrameStd();
  transform_property_->setName(
    QString::fromStdString(map_frame + " -> " + base_frame));
  bool transform_available = false;
  bool transform_fresh = false;
  try {
    const auto transformer = context_->getFrameManager()->getTransformer();
    if (transformer) {
      const auto transform =
        transformer->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
      transform_available = true;
      const double transform_stamp_sec = stampSeconds(transform.header.stamp);
      transform_fresh = (std::isfinite(ros_now_sec) && ros_now_sec > 0.0) ?
        sourceTimeFresh(ros_now_sec, transform_stamp_sec, output_stale_sec) : output_fresh;
    }
  } catch (const tf2::TransformException &) {
    transform_available = false;
  }
  const QString transform_state = transform_available && transform_fresh && output_fresh ?
    "ACTIVE" :
    (transform_available ? "STALE" : "UNAVAILABLE");
  transform_property_->setString(transform_state);
  setStatus(
    transform_available && transform_fresh && output_fresh ?
    rviz_common::properties::StatusProperty::Ok :
    rviz_common::properties::StatusProperty::Error,
    "Localization TF", QString::fromStdString(map_frame + " -> " + base_frame) +
    ": " + transform_state);

  QString registration_detail;
  if (registration) {
    if (registration_state == RegistrationVisualState::ACCEPTED) {
      if (const auto source = diagnosticValue(
          registration->status, "lidar_registration_source"))
      {
        registration_detail = " (" + QString::fromStdString(*source) + ")";
      }
    } else if (const auto reason = diagnosticValue(
        registration->status, "lidar_rejection_reason"))
    {
      registration_detail = " (" + QString::fromStdString(*reason) + ")";
    }
  }
  const QString registration_text =
    QString(registrationLabel(registration_state)) + registration_detail;
  registration_property_->setString(registration_text);
  setStatus(
    registrationLevel(registration_state), "Registration", registration_text);

  QString gnss_detail;
  if (fusion) {
    if (const auto mode = diagnosticValue(fusion->status, "recovery.mode")) {
      gnss_detail = " (" + QString::fromStdString(*mode) + ")";
    }
  }
  const QString gnss_text = QString(gnssLabel(gnss_state)) + gnss_detail;
  gnss_property_->setString(gnss_text);
  gnss_color_property_->setColor(toQColor(gnssColor(gnss_state)));
  setStatus(gnssLevel(gnss_state), "GNSS", gnss_text);
}

void LocalizationStatusDisplay::showWaitingState()
{
  interface_property_->setString("WAITING");
  position_x_property_->setString("WAITING");
  position_y_property_->setString("WAITING");
  yaw_property_->setString("WAITING");
  speed_property_->setString("WAITING");
  transform_property_->setString("WAITING");
  output_rate_property_->setString("WAITING");
  registration_property_->setString("WAITING");
  gnss_property_->setString("WAITING");
  gnss_color_property_->setColor(toQColor(gnssColor(GnssVisualState::WAITING)));
  setStatus(
    rviz_common::properties::StatusProperty::Warn, "Interface", "WAITING");
  setStatus(
    rviz_common::properties::StatusProperty::Warn, "Kinematic State", "WAITING");
  setStatus(
    rviz_common::properties::StatusProperty::Warn, "Localization TF", "WAITING");
  setStatus(
    rviz_common::properties::StatusProperty::Warn, "Registration", "WAITING");
  setStatus(rviz_common::properties::StatusProperty::Warn, "GNSS", "WAITING");
}

void LocalizationStatusDisplay::reset()
{
  rviz_common::Display::reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_state_.reset();
    latest_state_received_at_ = {};
    published_rate_estimator_ = CumulativeRateEstimator(3.0);
    adapter_diagnostic_.reset();
    registration_diagnostic_.reset();
    fusion_diagnostic_.reset();
  }
  update_elapsed_sec_ = 1.0;
  showWaitingState();
}

}  // namespace pure_odometry_bringup

PLUGINLIB_EXPORT_CLASS(
  pure_odometry_bringup::LocalizationStatusDisplay,
  rviz_common::Display)
