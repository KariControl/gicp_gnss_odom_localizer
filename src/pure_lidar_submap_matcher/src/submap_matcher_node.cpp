#include "pure_lidar_submap_matcher/submap_matcher_node.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace pure_lidar_submap_matcher
{
namespace
{
using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool finiteTransform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite();
}
}  // namespace

SubmapMatcherNode::SubmapMatcherNode(const rclcpp::NodeOptions & options)
: Node("submap_matcher", options)
{
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
  precision_frame_ = declare_parameter<std::string>(
    "precision_frame", precision_frame_);
  scan_topic_ = declare_parameter<std::string>("scan_topic", scan_topic_);
  correction_topic_ = declare_parameter<std::string>(
    "correction_topic", correction_topic_);
  diagnostics_topic_ = declare_parameter<std::string>(
    "diagnostics_topic", diagnostics_topic_);
  num_threads_ = declare_parameter<int>("num_threads", num_threads_);
  match_every_ = declare_parameter<int>("match_every", match_every_);
  min_keyframes_ = declare_parameter<int>("min_keyframes", min_keyframes_);
  max_keyframes_ = declare_parameter<int>("max_keyframes", max_keyframes_);
  min_points_ = declare_parameter<int>("min_points", min_points_);
  rejection_rebuild_threshold_ = declare_parameter<int>(
    "consecutive_rejections_before_rebuild", rejection_rebuild_threshold_);
  keyframe_policy_.min_interval_sec = declare_parameter<double>(
    "keyframe.min_interval_sec", keyframe_policy_.min_interval_sec);
  keyframe_policy_.max_interval_sec = declare_parameter<double>(
    "keyframe.max_interval_sec", keyframe_policy_.max_interval_sec);
  keyframe_policy_.min_translation_m = declare_parameter<double>(
    "keyframe.min_translation_m", keyframe_policy_.min_translation_m);
  keyframe_policy_.min_yaw_rad = declare_parameter<double>(
    "keyframe.min_yaw_rad", keyframe_policy_.min_yaw_rad);
  map_voxel_leaf_m_ = declare_parameter<double>("map.voxel_leaf_m", map_voxel_leaf_m_);
  map_max_points_ = declare_parameter<int>("map.max_points", map_max_points_);
  registration_type_ = declare_parameter<std::string>(
    "registration.type", registration_type_);
  max_corr_dist_m_ = declare_parameter<double>(
    "registration.max_corr_dist_m", max_corr_dist_m_);
  max_iterations_ = declare_parameter<int>("registration.max_iterations", max_iterations_);
  transformation_epsilon_ = declare_parameter<double>(
    "registration.transformation_epsilon", transformation_epsilon_);
  rotation_epsilon_ = declare_parameter<double>(
    "registration.rotation_epsilon", rotation_epsilon_);
  correspondence_randomness_ = declare_parameter<int>(
    "registration.correspondence_randomness", correspondence_randomness_);
  voxel_resolution_ = declare_parameter<double>(
    "registration.voxel_resolution", voxel_resolution_);
  match_limits_.max_fitness = declare_parameter<double>(
    "gate.max_fitness", match_limits_.max_fitness);
  match_limits_.min_inlier_ratio = declare_parameter<double>(
    "gate.min_inlier_ratio", match_limits_.min_inlier_ratio);
  match_limits_.max_translation_m = declare_parameter<double>(
    "gate.max_correction_translation_m", match_limits_.max_translation_m);
  match_limits_.max_yaw_rad = declare_parameter<double>(
    "gate.max_correction_yaw_rad", match_limits_.max_yaw_rad);
  match_limits_.max_z_m = declare_parameter<double>(
    "gate.max_correction_z_m", match_limits_.max_z_m);
  match_limits_.max_roll_pitch_rad = declare_parameter<double>(
    "gate.max_correction_roll_pitch_rad", match_limits_.max_roll_pitch_rad);
  robust_config_.window_size = static_cast<std::size_t>(declare_parameter<int>(
    "robust.window_size", static_cast<int>(robust_config_.window_size)));
  robust_config_.min_consistent = static_cast<std::size_t>(declare_parameter<int>(
    "robust.min_consistent", static_cast<int>(robust_config_.min_consistent)));
  robust_config_.consistency_translation_m = declare_parameter<double>(
    "robust.consistency_translation_m", robust_config_.consistency_translation_m);
  robust_config_.consistency_yaw_rad = declare_parameter<double>(
    "robust.consistency_yaw_rad", robust_config_.consistency_yaw_rad);
  robust_config_.huber_translation_m = declare_parameter<double>(
    "robust.huber_translation_m", robust_config_.huber_translation_m);
  robust_config_.huber_yaw_rad = declare_parameter<double>(
    "robust.huber_yaw_rad", robust_config_.huber_yaw_rad);
  robust_config_.max_commit_pivot_step_m = declare_parameter<double>(
    "robust.max_commit_pivot_step_m", robust_config_.max_commit_pivot_step_m);
  robust_config_.max_commit_yaw_step_rad = declare_parameter<double>(
    "robust.max_commit_yaw_step_rad", robust_config_.max_commit_yaw_step_rad);
  position_stddev_m_ = declare_parameter<double>(
    "output.position_stddev_m", position_stddev_m_);
  yaw_stddev_rad_ = declare_parameter<double>("output.yaw_stddev_rad", yaw_stddev_rad_);
  tf_lookup_timeout_sec_ = declare_parameter<double>(
    "tf_lookup_timeout_sec", tf_lookup_timeout_sec_);
  diagnostics_period_sec_ = declare_parameter<double>(
    "diagnostics_period_sec", diagnostics_period_sec_);

  const bool registration_valid = registration_type_ == "GICP" ||
    registration_type_ == "VGICP";
  if (base_frame_.empty() || precision_frame_.empty() ||
    scan_topic_.empty() || correction_topic_.empty() ||
    diagnostics_topic_.empty() || num_threads_ < 1 || match_every_ < 1 ||
    min_keyframes_ < 1 || max_keyframes_ < min_keyframes_ || min_points_ < 20 ||
    rejection_rebuild_threshold_ < 1 || map_voxel_leaf_m_ <= 0.0 ||
    map_max_points_ < min_points_ || max_corr_dist_m_ <= 0.0 || max_iterations_ < 1 ||
    correspondence_randomness_ < 3 || voxel_resolution_ <= 0.0 ||
    position_stddev_m_ <= 0.0 || yaw_stddev_rad_ <= 0.0 ||
    tf_lookup_timeout_sec_ < 0.0 || diagnostics_period_sec_ <= 0.0 ||
    !registration_valid)
  {
    throw std::invalid_argument("invalid submap matcher parameter");
  }
  committer_ = RobustSe2Committer(robust_config_);
  matcher_session_id_ = makeSessionId();
  registration_.setNumThreads(num_threads_);
  registration_.setRegistrationType(registration_type_);
  registration_.setMaxCorrespondenceDistance(max_corr_dist_m_);
  registration_.setMaximumIterations(max_iterations_);
  registration_.setTransformationEpsilon(transformation_epsilon_);
  registration_.setRotationEpsilon(rotation_epsilon_);
  registration_.setCorrespondenceRandomness(correspondence_randomness_);
  registration_.setVoxelResolution(voxel_resolution_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  correction_publisher_ = create_publisher<Correction>(
    correction_topic_, rclcpp::QoS(10).reliable());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, 10);
  scan_subscription_ = create_subscription<Scan>(
    scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&SubmapMatcherNode::onScan, this, std::placeholders::_1));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(diagnostics_period_sec_)),
    std::bind(&SubmapMatcherNode::publishDiagnostics, this));
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.matcher_session = matcher_session_id_;
  }
  worker_ = std::thread(&SubmapMatcherNode::workerLoop, this);
  RCLCPP_INFO(
    get_logger(), "persistent full-SE2 submap matcher ready: session=%llu window=%zu N=%zu",
    static_cast<unsigned long long>(matcher_session_id_), robust_config_.window_size,
    robust_config_.min_consistent);
}

SubmapMatcherNode::~SubmapMatcherNode()
{
  queue_.close();
  if (worker_.joinable()) worker_.join();
}

void SubmapMatcherNode::onScan(Scan::ConstSharedPtr message)
{
  const bool replaced = queue_.push(std::move(message));
  std::lock_guard<std::mutex> lock(counters_mutex_);
  ++counters_.received;
  if (replaced) ++counters_.queue_drop;
}

void SubmapMatcherNode::workerLoop()
{
  while (rclcpp::ok()) {
    auto message = queue_.waitPop();
    if (!message) break;
    try {
      processScan(*message);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "submap worker exception: %s", exception.what());
      clearMap("worker_exception", true);
      stream_initialized_ = false;
    }
  }
}

void SubmapMatcherNode::processScan(const Scan::ConstSharedPtr & message)
{
  const auto start = Clock::now();
  auto finish = [this, &start, &message](const std::string & reason, bool malformed = false) {
      const double processing_ms = elapsedMs(start);
      double latency_ms = 0.0;
      try {
        const rclcpp::Time message_stamp(message->header.stamp, get_clock()->get_clock_type());
        latency_ms = std::max(0.0, (now() - message_stamp).seconds() * 1000.0);
      } catch (...) {}
      std::lock_guard<std::mutex> lock(counters_mutex_);
      ++counters_.processed;
      if (malformed) ++counters_.malformed;
      counters_.processing_last_ms = processing_ms;
      counters_.processing_sum_ms += processing_ms;
      counters_.processing_max_ms = std::max(
        counters_.processing_max_ms, counters_.processing_last_ms);
      counters_.latency_last_ms = latency_ms;
      counters_.latency_sum_ms += latency_ms;
      counters_.latency_max_ms = std::max(counters_.latency_max_ms, latency_ms);
      counters_.processing_window.push_back(processing_ms);
      counters_.latency_window.push_back(latency_ms);
      while (counters_.processing_window.size() > 256) counters_.processing_window.pop_front();
      while (counters_.latency_window.size() > 256) counters_.latency_window.pop_front();
      counters_.last_reason = reason;
    };
  std::string reason;
  if (!validateScan(*message, reason)) {
    finish(reason, true);
    return;
  }
  const bool frame_changed = (stream_initialized_ || last_key_.session != 0U) &&
    (message->header.frame_id != odom_frame_ || message->cloud.header.frame_id != cloud_frame_);
  if (frame_changed) extrinsic_cached_ = false;
  Cloud::Ptr cloud;
  if (!cloudToBase(*message, cloud, reason)) {
    finish(reason, true);
    return;
  }
  const Eigen::Isometry3d raw_pose = poseToIsometry(message->raw_pose.pose);
  const StreamKey key{message->odom_session_id, message->odom_generation,
    message->sequence, stampNs(message->header.stamp)};
  if (!stream_initialized_) {
    // The only non-startup path here is worker-exception recovery.  Keep the
    // previous exact-key fence alive while the map is uninitialized: otherwise
    // a delayed retired-session or pre-exception packet could resurrect/rewind
    // the stream simply by arriving first after the exception.
    bool preserve_correction = committer_.hasCommitted();
    std::string initialize_reason = preserve_correction ?
      "worker_exception_recovery" : "initialized";
    if (last_key_.session != 0U) {
      const StreamAction recovery_action = guardedStreamAction(
        last_key_, key, retired_odom_sessions_);
      if (recovery_action == StreamAction::stale) {
        {
          std::lock_guard<std::mutex> lock(counters_mutex_);
          ++counters_.stale;
        }
        finish("stale_or_duplicate_key_after_worker_exception");
        return;
      }
      if (key.session != last_key_.session) {
        retired_odom_sessions_.insert(last_key_.session);
        preserve_correction = false;
        initialize_reason = "odom_session_change_after_worker_exception";
      } else if (frame_changed) {
        preserve_correction = false;
        initialize_reason = "frame_change_after_worker_exception";
      } else if (key.generation != last_key_.generation) {
        initialize_reason = "odom_generation_change_after_worker_exception";
      }
    }
    initializeStream(
      *message, cloud, raw_pose, initialize_reason,
      preserve_correction);
    finish(initialize_reason);
    return;
  }
  // Retirement wins over frame-change handling: even a delayed old process
  // that used another cloud frame must not resurrect its map.
  const StreamAction guarded_action =
    guardedStreamAction(last_key_, key, retired_odom_sessions_);
  const StreamAction action = guarded_action == StreamAction::stale ?
    StreamAction::stale : (frame_changed ? StreamAction::reset : guarded_action);
  if (action == StreamAction::reset) {
    const bool same_session = key.session == last_key_.session && !frame_changed;
    if (key.session != last_key_.session) {
      retired_odom_sessions_.insert(last_key_.session);
    }
    const std::string reset_reason = frame_changed ? "frame_change" :
      (key.session != last_key_.session ? "odom_session_change" :
      (key.generation != last_key_.generation ? "odom_generation_change" :
      "nonmonotonic_key"));
    initializeStream(*message, cloud, raw_pose, reset_reason, same_session);
    finish(reset_reason);
    return;
  }
  if (action == StreamAction::stale) {
    {
      std::lock_guard<std::mutex> lock(counters_mutex_);
      ++counters_.stale;
    }
    finish("stale_or_duplicate_key");
    return;
  }
  if (key.sequence > last_key_.sequence + 1) {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.sequence_gap += key.sequence - last_key_.sequence - 1;
  }
  last_key_ = key;
  ++scans_since_rebuild_;

  const Eigen::Isometry3d committed = fromSe2(committer_.committed());
  const Eigen::Isometry3d precision_guess = committed * raw_pose;
  const Eigen::Isometry3d anchor_guess = precision_T_anchor_.inverse() * precision_guess;
  const bool map_ready = static_cast<int>(keyframes_.size()) >= min_keyframes_ &&
    target_ready_ && static_cast<int>(map_cloud_->size()) >= min_points_;
  MatchResult match_result;
  CommitResult commit_result;
  if (!map_ready) {
    match_result.reason = "warmup";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++counters_.warmup;
  } else if (scans_since_rebuild_ % static_cast<std::uint64_t>(match_every_) != 0) {
    match_result.reason = "interval_skip";
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++counters_.interval_skip;
  } else {
    match_result = match(cloud, anchor_guess, precision_guess);
    if (match_result.accepted) {
      consecutive_rejections_ = 0;
      const Eigen::Isometry3d measured_transform = match_result.precision_T_base * raw_pose.inverse();
      commit_result = committer_.add(toSe2(measured_transform), toSe2(raw_pose));
      if (commit_result.committed) {
        const Eigen::Isometry3d corrected = fromSe2(commit_result.transform) * raw_pose;
        publishCorrection(*message, corrected, match_result, commit_result);
        std::lock_guard<std::mutex> lock(counters_mutex_);
        ++counters_.committed;
      }
    } else {
      ++consecutive_rejections_;
      if (consecutive_rejections_ >= static_cast<std::uint32_t>(rejection_rebuild_threshold_)) {
        recoverMap(*message, cloud, raw_pose);
        finish("recovery_rebuild_preserved_correction");
        return;
      }
    }
  }

  // Warmup uses the committed raw prediction. Once matching is possible, a
  // rejected scan never contaminates the map; accepted registration may.
  if (!map_ready || match_result.accepted) {
    const Eigen::Isometry3d keyframe_pose = match_result.accepted ?
      match_result.anchor_T_base : anchor_guess;
    maybeAddKeyframe(*message, cloud, keyframe_pose);
  }
  {
    const Se2 value = committer_.committed();
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.last_session = key.session;
    counters_.last_generation = key.generation;
    counters_.last_sequence = key.sequence;
    counters_.matcher_session = matcher_session_id_;
    counters_.submap_generation = submap_generation_;
    counters_.consecutive_rejections = consecutive_rejections_;
    counters_.keyframes = keyframes_.size();
    counters_.map_points = map_cloud_->size();
    counters_.candidates = committer_.candidateCount();
    counters_.committed_x = value.x;
    counters_.committed_y = value.y;
    counters_.committed_yaw = value.yaw;
  }
  finish(commit_result.committed ? "committed" : match_result.reason);
}

bool SubmapMatcherNode::validateScan(const Scan & message, std::string & reason) const
{
  if (message.odom_session_id == 0 || message.odom_generation == 0 || message.sequence == 0) {
    reason = "zero_stream_key";
    return false;
  }
  if (message.header.frame_id.empty() || message.cloud.header.frame_id.empty() ||
    stampNs(message.header.stamp) <= 0 || stampNs(message.cloud.header.stamp) !=
    stampNs(message.header.stamp))
  {
    reason = "frame_or_stamp_mismatch";
    return false;
  }
  if (message.cloud.data.empty() || message.cloud.point_step == 0 ||
    message.cloud.width == 0 || message.cloud.height == 0)
  {
    reason = "empty_cloud";
    return false;
  }
  const auto & p = message.raw_pose.pose.position;
  const auto & q = message.raw_pose.pose.orientation;
  const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(qn) || qn < 1e-9)
  {
    reason = "invalid_raw_pose";
    return false;
  }
  return true;
}

bool SubmapMatcherNode::cloudToBase(
  const Scan & message, Cloud::Ptr & cloud, std::string & reason)
{
  auto input = Cloud::Ptr(new Cloud);
  try {pcl::fromROSMsg(message.cloud, *input);} catch (const std::exception & e) {
    reason = std::string("cloud_conversion:") + e.what();
    return false;
  }
  auto finite_cloud = Cloud::Ptr(new Cloud);
  finite_cloud->reserve(input->size());
  for (const auto & p : *input) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) finite_cloud->push_back(p);
  }
  finite_cloud->width = static_cast<std::uint32_t>(finite_cloud->size());
  finite_cloud->height = 1;
  if (static_cast<int>(finite_cloud->size()) < min_points_) {
    reason = "insufficient_cloud_points";
    return false;
  }
  const std::string & frame = message.cloud.header.frame_id;
  if (!extrinsic_cached_ || frame != cloud_frame_) {
    if (frame == base_frame_) {
      base_T_cloud_ = Eigen::Isometry3d::Identity();
    } else {
      try {
        base_T_cloud_ = transformToIsometry(tf_buffer_->lookupTransform(
          base_frame_, frame, tf2::TimePointZero,
          tf2::durationFromSec(tf_lookup_timeout_sec_)).transform);
      } catch (const tf2::TransformException & e) {
        reason = std::string("tf_lookup:") + e.what();
        return false;
      }
    }
    extrinsic_cached_ = true;
  }
  if (!finiteTransform(base_T_cloud_)) {
    reason = "nonfinite_extrinsic";
    return false;
  }
  if (base_T_cloud_.matrix().isIdentity(1e-12)) {
    cloud = finite_cloud;
  } else {
    cloud.reset(new Cloud);
    pcl::transformPointCloud(*finite_cloud, *cloud, base_T_cloud_.matrix().cast<float>());
  }
  return true;
}

SubmapMatcherNode::MatchResult SubmapMatcherNode::match(
  const Cloud::ConstPtr & cloud, const Eigen::Isometry3d & anchor_guess,
  const Eigen::Isometry3d & previous_precision_guess)
{
  MatchResult output;
  output.attempted = true;
  output.anchor_T_base = anchor_guess;
  const auto start = Clock::now();
  try {
    Cloud aligned;
    registration_.setInputSource(cloud);
    registration_.align(aligned, anchor_guess.matrix().cast<float>());
    const auto & result = registration_.getRegistrationResult();
    output.metrics.converged = registration_.hasConverged() && result.converged;
    output.metrics.fitness = registration_.getFitnessScore(max_corr_dist_m_);
    output.metrics.inlier_ratio = cloud->empty() ? 0.0 : std::clamp(
      static_cast<double>(result.num_inliers) / static_cast<double>(cloud->size()), 0.0, 1.0);
    output.anchor_T_base.matrix() = result.T_target_source.matrix();
    output.precision_T_base = precision_T_anchor_ * output.anchor_T_base;
    const Eigen::Vector3d difference =
      output.precision_T_base.translation() - previous_precision_guess.translation();
    output.innovation_x = difference.x();
    output.innovation_y = difference.y();
    const Eigen::Isometry3d local_correction =
      previous_precision_guess.inverse() * output.precision_T_base;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    rotationToRpy(local_correction.rotation(), roll, pitch, yaw);
    output.innovation_yaw = normalizeAngle(yaw);
    output.metrics.translation_m = std::hypot(output.innovation_x, output.innovation_y);
    output.metrics.yaw_rad = output.innovation_yaw;
    output.metrics.z_m = local_correction.translation().z();
    output.metrics.roll_rad = roll;
    output.metrics.pitch_rad = pitch;
    output.metrics.finite = finiteTransform(output.anchor_T_base) &&
      finiteTransform(output.precision_T_base);
    output.reason = matchRejection(match_limits_, output.metrics);
    output.accepted = output.reason.empty();
    if (output.accepted) output.reason = "accepted_candidate";
    registration_.clearSource();
  } catch (const std::exception & e) {
    registration_.clearSource();
    output.reason = std::string("registration_exception:") + e.what();
  }
  output.elapsed_ms = elapsedMs(start);
  {
    std::lock_guard<std::mutex> lock(counters_mutex_);
    ++counters_.attempted;
    if (output.accepted) ++counters_.accepted; else ++counters_.rejected;
    counters_.match_last_ms = output.elapsed_ms;
    counters_.match_sum_ms += output.elapsed_ms;
    counters_.match_max_ms = std::max(counters_.match_max_ms, output.elapsed_ms);
    counters_.match_window.push_back(output.elapsed_ms);
    while (counters_.match_window.size() > 256) counters_.match_window.pop_front();
  }
  return output;
}

void SubmapMatcherNode::publishCorrection(
  const Scan & scan, const Eigen::Isometry3d & corrected_pose,
  const MatchResult & match_result, const CommitResult & commit)
{
  Correction message;
  message.header = scan.header;
  message.odom_session_id = scan.odom_session_id;
  message.odom_generation = scan.odom_generation;
  message.sequence = scan.sequence;
  message.matcher_session_id = matcher_session_id_;
  message.submap_generation = submap_generation_;
  message.correction_id = ++correction_id_;
  message.precision_frame_id = precision_frame_;
  message.precision_from_raw.translation.x = commit.transform.x;
  message.precision_from_raw.translation.y = commit.transform.y;
  message.precision_from_raw.translation.z = 0.0;
  const Eigen::Quaterniond correction_quaternion(
    Eigen::AngleAxisd(commit.transform.yaw, Eigen::Vector3d::UnitZ()));
  message.precision_from_raw.rotation.x = correction_quaternion.x();
  message.precision_from_raw.rotation.y = correction_quaternion.y();
  message.precision_from_raw.rotation.z = correction_quaternion.z();
  message.precision_from_raw.rotation.w = correction_quaternion.w();
  message.corrected_pose.pose = scan.raw_pose.pose;
  message.corrected_pose.pose.position.x = corrected_pose.translation().x();
  message.corrected_pose.pose.position.y = corrected_pose.translation().y();
  message.corrected_pose.pose.position.z = 0.0;
  Eigen::Quaterniond quaternion(corrected_pose.rotation());
  quaternion.normalize();
  message.corrected_pose.pose.orientation.x = quaternion.x();
  message.corrected_pose.pose.orientation.y = quaternion.y();
  message.corrected_pose.pose.orientation.z = quaternion.z();
  message.corrected_pose.pose.orientation.w = quaternion.w();
  message.corrected_pose.covariance.fill(0.0);
  const double quality = std::clamp(
    (1.0 + match_result.metrics.fitness) /
    std::max(0.25, match_result.metrics.inlier_ratio), 1.0, 4.0);
  const double xy_var = std::clamp(position_stddev_m_ * position_stddev_m_ * quality, 1e-4, 4.0);
  const double yaw_var = std::clamp(yaw_stddev_rad_ * yaw_stddev_rad_ * quality, 1e-5, 0.25);
  message.corrected_pose.covariance[0] = xy_var;
  message.corrected_pose.covariance[7] = xy_var;
  message.corrected_pose.covariance[14] = 1e6;
  message.corrected_pose.covariance[21] = 1e6;
  message.corrected_pose.covariance[28] = 1e6;
  message.corrected_pose.covariance[35] = yaw_var;
  message.use_yaw = true;
  message.fitness = match_result.metrics.fitness;
  message.inlier_ratio = match_result.metrics.inlier_ratio;
  message.innovation_x_m = match_result.innovation_x;
  message.innovation_y_m = match_result.innovation_y;
  message.innovation_translation_m = std::hypot(
    match_result.innovation_x, match_result.innovation_y);
  message.innovation_yaw_rad = match_result.innovation_yaw;
  message.consistency_count = static_cast<std::uint32_t>(commit.consistent_count);
  correction_publisher_->publish(message);
  std::lock_guard<std::mutex> lock(counters_mutex_);
  ++counters_.correction_publish;
}

void SubmapMatcherNode::initializeStream(
  const Scan & scan, const Cloud::ConstPtr & cloud,
  const Eigen::Isometry3d & raw_pose, const std::string & reason,
  bool preserve_correction)
{
  clearMap(reason, preserve_correction);
  stream_initialized_ = true;
  last_key_ = {scan.odom_session_id, scan.odom_generation, scan.sequence,
    stampNs(scan.header.stamp)};
  odom_frame_ = scan.header.frame_id;
  cloud_frame_ = scan.cloud.header.frame_id;
  const Eigen::Isometry3d corrected = fromSe2(committer_.committed()) * raw_pose;
  precision_T_anchor_ = corrected;
  Keyframe keyframe;
  keyframe.stamp_ns = last_key_.stamp_ns;
  keyframe.cloud_base = cloud;
  keyframes_.push_back(keyframe);
  rebuildMap();
  std::lock_guard<std::mutex> lock(counters_mutex_);
  counters_.last_session = last_key_.session;
  counters_.last_generation = last_key_.generation;
  counters_.last_sequence = last_key_.sequence;
  counters_.last_reason = reason;
}

void SubmapMatcherNode::clearMap(const std::string &, bool preserve_correction)
{
  registration_.clearSource();
  registration_.clearTarget();
  target_ready_ = false;
  keyframes_.clear();
  map_cloud_.reset(new Cloud);
  scans_since_rebuild_ = 0;
  consecutive_rejections_ = 0;
  if (preserve_correction) committer_.clearCandidates(); else committer_.resetAll();
  if (++submap_generation_ == 0) submap_generation_ = 1;
  std::lock_guard<std::mutex> lock(counters_mutex_);
  ++counters_.stream_reset;
  counters_.matcher_session = matcher_session_id_;
  counters_.submap_generation = submap_generation_;
}

void SubmapMatcherNode::recoverMap(
  const Scan & scan, const Cloud::ConstPtr & cloud, const Eigen::Isometry3d & raw_pose)
{
  clearMap("consecutive_rejection_rebuild", true);
  precision_T_anchor_ = fromSe2(committer_.committed()) * raw_pose;
  Keyframe keyframe;
  keyframe.stamp_ns = stampNs(scan.header.stamp);
  keyframe.cloud_base = cloud;
  keyframes_.push_back(keyframe);
  scans_since_rebuild_ = 0;
  rebuildMap();
  std::lock_guard<std::mutex> lock(counters_mutex_);
  ++counters_.recovery_rebuild;
}

bool SubmapMatcherNode::maybeAddKeyframe(
  const Scan & scan, const Cloud::ConstPtr & cloud,
  const Eigen::Isometry3d & anchor_T_base)
{
  if (!cloud || keyframes_.empty()) return false;
  const auto & previous = keyframes_.back();
  const double dt = static_cast<double>(stampNs(scan.header.stamp) - previous.stamp_ns) * 1e-9;
  const Eigen::Isometry3d delta = previous.anchor_T_base.inverse() * anchor_T_base;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  rotationToRpy(delta.rotation(), roll, pitch, yaw);
  if (!keyframeDue(keyframe_policy_, dt,
    std::hypot(delta.translation().x(), delta.translation().y()), yaw)) return false;
  keyframes_.push_back(Keyframe{stampNs(scan.header.stamp), anchor_T_base, cloud});
  bool removed = false;
  while (static_cast<int>(keyframes_.size()) > max_keyframes_) {
    keyframes_.pop_front();
    removed = true;
  }
  if (removed) reanchorToOldest();
  rebuildMap();
  return true;
}

void SubmapMatcherNode::reanchorToOldest()
{
  if (keyframes_.empty()) return;
  const Eigen::Isometry3d old_anchor_T_new = keyframes_.front().anchor_T_base;
  const Eigen::Isometry3d new_T_old = old_anchor_T_new.inverse();
  for (auto & keyframe : keyframes_) {
    keyframe.anchor_T_base = new_T_old * keyframe.anchor_T_base;
  }
  precision_T_anchor_ = precision_T_anchor_ * old_anchor_T_new;
  // Pure coordinate change: committed precision<-raw remains untouched and
  // consumer-visible submap_generation intentionally remains stable.
}

void SubmapMatcherNode::rebuildMap()
{
  auto aggregate = Cloud::Ptr(new Cloud);
  for (const auto & keyframe : keyframes_) {
    if (!keyframe.cloud_base) continue;
    Cloud transformed;
    pcl::transformPointCloud(
      *keyframe.cloud_base, transformed, keyframe.anchor_T_base.matrix().cast<float>());
    *aggregate += transformed;
  }
  auto filtered = Cloud::Ptr(new Cloud);
  if (!aggregate->empty()) {
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(map_voxel_leaf_m_, map_voxel_leaf_m_, map_voxel_leaf_m_);
    voxel.setInputCloud(aggregate);
    voxel.filter(*filtered);
  }
  if (static_cast<int>(filtered->size()) > map_max_points_) {
    auto capped = Cloud::Ptr(new Cloud);
    capped->reserve(map_max_points_);
    const double stride = static_cast<double>(filtered->size()) / map_max_points_;
    for (int i = 0; i < map_max_points_; ++i) {
      capped->push_back((*filtered)[std::min(
        static_cast<std::size_t>(i * stride), filtered->size() - 1)]);
    }
    filtered = capped;
  }
  filtered->width = static_cast<std::uint32_t>(filtered->size());
  filtered->height = 1;
  map_cloud_ = filtered;
  registration_.clearTarget();
  target_ready_ = static_cast<int>(map_cloud_->size()) >= min_points_;
  if (target_ready_) registration_.setInputTarget(map_cloud_);
  std::lock_guard<std::mutex> lock(counters_mutex_);
  ++counters_.map_rebuild;
  counters_.keyframes = keyframes_.size();
  counters_.map_points = map_cloud_->size();
}

void SubmapMatcherNode::publishDiagnostics()
{
  Counters c;
  {std::lock_guard<std::mutex> lock(counters_mutex_); c = counters_;}
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "localization/submap_matcher";
  status.hardware_id = "none";
  status.level = c.received == 0 || c.queue_drop > 0 || c.consecutive_rejections > 0 ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = c.received == 0 ? "waiting for exact-key raw scans" : "persistent full-SE2 matching";
  auto add = [&status](const std::string & key, const auto & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
  auto add_s = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  auto percentile99 = [](const std::deque<double> & values) {
      if (values.empty()) return 0.0;
      std::vector<double> sorted(values.begin(), values.end());
      std::sort(sorted.begin(), sorted.end());
      const std::size_t index = std::min(
        sorted.size() - 1, static_cast<std::size_t>(std::ceil(0.99 * sorted.size()) - 1));
      return sorted[index];
    };
  add_s("correction_model", "persistent_full_se2_fixed_lag_robust");
  add_s("exact_key_contract", "odom_session+odom_generation+sequence+accepted_scan_stamp");
  add_s("matcher_restart_consumer_policy",
    "rebase_new_matcher_session_to_last_applied_precision_transform");
  add_s("late_duplicate_policy", "ignore_without_stream_or_map_reset");
  add_s("rejection_policy", "preserve_committed_transform_rebuild_map_only");
  add_s("rolling_reanchor_policy", "coordinate_change_preserves_transform_and_generation");
  add_s("last_reason", c.last_reason);
  add("received_count", c.received); add("queue_drop_count", c.queue_drop);
  add("processed_count", c.processed); add("malformed_count", c.malformed);
  add("stale_or_duplicate_count", c.stale);
  add("stream_reset_count", c.stream_reset); add("recovery_rebuild_count", c.recovery_rebuild);
  add("attempted_count", c.attempted); add("accepted_match_count", c.accepted);
  add("rejected_match_count", c.rejected); add("committed_count", c.committed);
  add("correction_publish_count", c.correction_publish);
  add("consecutive_rejection_count", c.consecutive_rejections);
  add("last_odom_session_id", c.last_session); add("last_odom_generation", c.last_generation);
  add("last_sequence", c.last_sequence); add("matcher_session_id", c.matcher_session);
  add("submap_generation", c.submap_generation); add("keyframe_count", c.keyframes);
  add("map_point_count", c.map_points); add("robust_candidate_count", c.candidates);
  add("committed_transform_x_m", c.committed_x); add("committed_transform_y_m", c.committed_y);
  add("committed_transform_yaw_rad", c.committed_yaw);
  add("processing_last_ms", c.processing_last_ms); add("processing_max_ms", c.processing_max_ms);
  add("processing_mean_ms", c.processed == 0 ? 0.0 :
    c.processing_sum_ms / static_cast<double>(c.processed));
  add("processing_p99_ms", percentile99(c.processing_window));
  add("match_last_ms", c.match_last_ms); add("match_max_ms", c.match_max_ms);
  add("match_mean_ms", c.attempted == 0 ? 0.0 :
    c.match_sum_ms / static_cast<double>(c.attempted));
  add("match_p99_ms", percentile99(c.match_window));
  add("latency_last_ms", c.latency_last_ms); add("latency_max_ms", c.latency_max_ms);
  add("latency_mean_ms", c.processed == 0 ? 0.0 :
    c.latency_sum_ms / static_cast<double>(c.processed));
  add("latency_p99_ms", percentile99(c.latency_window));
  add("accepted_attempted_ratio", c.attempted == 0 ? 0.0 :
    static_cast<double>(c.accepted) / static_cast<double>(c.attempted));
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

std::int64_t SubmapMatcherNode::stampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

std::uint64_t SubmapMatcherNode::makeSessionId()
{
  std::uint64_t value = static_cast<std::uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count());
  try {
    std::random_device random;
    value ^= static_cast<std::uint64_t>(random()) << 32U;
    value ^= random();
  } catch (...) {}
  return value == 0 ? 1 : value;
}

Eigen::Isometry3d SubmapMatcherNode::poseToIsometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  q.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}

Eigen::Isometry3d SubmapMatcherNode::transformToIsometry(
  const geometry_msgs::msg::Transform & transform)
{
  Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x,
    transform.rotation.y, transform.rotation.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  if (!q.coeffs().allFinite() || q.norm() < 1e-9) {
    result.matrix().setConstant(std::numeric_limits<double>::quiet_NaN());
    return result;
  }
  q.normalize();
  result.translation() = Eigen::Vector3d(
    transform.translation.x, transform.translation.y, transform.translation.z);
  result.linear() = q.toRotationMatrix();
  return result;
}

Se2 SubmapMatcherNode::toSe2(const Eigen::Isometry3d & transform)
{
  return {transform.translation().x(), transform.translation().y(),
    std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0))};
}

Eigen::Isometry3d SubmapMatcherNode::fromSe2(const Se2 & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = pose.x;
  result.translation().y() = pose.y;
  result.linear() = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

void SubmapMatcherNode::rotationToRpy(
  const Eigen::Matrix3d & rotation, double & roll, double & pitch, double & yaw)
{
  roll = std::atan2(rotation(2, 1), rotation(2, 2));
  pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  yaw = std::atan2(rotation(1, 0), rotation(0, 0));
}
}  // namespace pure_lidar_submap_matcher
