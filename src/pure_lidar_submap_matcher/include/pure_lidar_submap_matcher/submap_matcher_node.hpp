#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pure_lidar_msgs/msg/submap_correction.hpp>
#include <pure_lidar_msgs/msg/submap_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "pure_lidar_submap_matcher/latest_only_queue.hpp"
#include "pure_lidar_submap_matcher/submap_policy.hpp"

namespace pure_lidar_submap_matcher
{
class SubmapMatcherNode : public rclcpp::Node
{
public:
  explicit SubmapMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SubmapMatcherNode() override;

private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;
  using Scan = pure_lidar_msgs::msg::SubmapScan;
  using Correction = pure_lidar_msgs::msg::SubmapCorrection;

  struct Keyframe
  {
    std::int64_t stamp_ns{0};
    Eigen::Isometry3d anchor_T_base{Eigen::Isometry3d::Identity()};
    Cloud::ConstPtr cloud_base;
  };

  struct MatchResult
  {
    bool attempted{false};
    bool accepted{false};
    Eigen::Isometry3d anchor_T_base{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d precision_T_base{Eigen::Isometry3d::Identity()};
    MatchMetrics metrics;
    double innovation_x{0.0};
    double innovation_y{0.0};
    double innovation_yaw{0.0};
    double elapsed_ms{0.0};
    std::string reason{"not_attempted"};
  };

  struct Counters
  {
    std::uint64_t received{0}, queue_drop{0}, processed{0}, malformed{0}, stale{0};
    std::uint64_t stream_reset{0}, recovery_rebuild{0}, sequence_gap{0};
    std::uint64_t attempted{0}, accepted{0}, rejected{0}, committed{0};
    std::uint64_t warmup{0}, interval_skip{0}, map_rebuild{0};
    std::uint64_t correction_publish{0};
    std::uint32_t consecutive_rejections{0};
    std::uint64_t last_session{0}, last_generation{0}, last_sequence{0};
    std::uint64_t matcher_session{0}, submap_generation{0};
    std::size_t keyframes{0}, map_points{0}, candidates{0};
    double committed_x{0.0}, committed_y{0.0}, committed_yaw{0.0};
    double processing_last_ms{0.0}, processing_sum_ms{0.0}, processing_max_ms{0.0};
    double match_last_ms{0.0}, match_sum_ms{0.0}, match_max_ms{0.0};
    double latency_last_ms{0.0}, latency_sum_ms{0.0}, latency_max_ms{0.0};
    std::deque<double> processing_window, match_window, latency_window;
    std::string last_reason{"waiting_for_scan"};
  };

  void onScan(Scan::ConstSharedPtr message);
  void workerLoop();
  void processScan(const Scan::ConstSharedPtr & message);
  bool validateScan(const Scan & message, std::string & reason) const;
  bool cloudToBase(const Scan & message, Cloud::Ptr & cloud, std::string & reason);
  MatchResult match(
    const Cloud::ConstPtr & cloud, const Eigen::Isometry3d & anchor_guess,
    const Eigen::Isometry3d & previous_precision_guess);
  void publishCorrection(
    const Scan & scan, const Eigen::Isometry3d & corrected_pose,
    const MatchResult & match, const CommitResult & commit);
  void initializeStream(
    const Scan & scan, const Cloud::ConstPtr & cloud,
    const Eigen::Isometry3d & raw_pose, const std::string & reason,
    bool preserve_correction);
  void clearMap(const std::string & reason, bool preserve_correction);
  void recoverMap(
    const Scan & scan, const Cloud::ConstPtr & cloud, const Eigen::Isometry3d & raw_pose);
  bool maybeAddKeyframe(
    const Scan & scan, const Cloud::ConstPtr & cloud,
    const Eigen::Isometry3d & anchor_T_base);
  void reanchorToOldest();
  void rebuildMap();
  void publishDiagnostics();

  static std::int64_t stampNs(const builtin_interfaces::msg::Time & stamp);
  static std::uint64_t makeSessionId();
  static Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose & pose);
  static Eigen::Isometry3d transformToIsometry(const geometry_msgs::msg::Transform & transform);
  static Se2 toSe2(const Eigen::Isometry3d & transform);
  static Eigen::Isometry3d fromSe2(const Se2 & pose);
  static void rotationToRpy(
    const Eigen::Matrix3d & rotation, double & roll, double & pitch, double & yaw);

  std::string base_frame_{"base_link"};
  std::string precision_frame_{"odom_precision"};
  std::string scan_topic_{"/localization/submap_scan"};
  std::string correction_topic_{"/localization/submap_correction"};
  std::string diagnostics_topic_{"diagnostics"};
  int num_threads_{2}, match_every_{1}, min_keyframes_{3}, max_keyframes_{15};
  int min_points_{200}, rejection_rebuild_threshold_{8}, map_max_points_{80000};
  KeyframePolicy keyframe_policy_;
  double map_voxel_leaf_m_{0.35};
  std::string registration_type_{"VGICP"};
  double max_corr_dist_m_{1.0}, transformation_epsilon_{5e-4};
  double rotation_epsilon_{5e-4}, voxel_resolution_{1.0};
  int max_iterations_{30}, correspondence_randomness_{20};
  MatchLimits match_limits_;
  RobustConfig robust_config_;
  double position_stddev_m_{0.20}, yaw_stddev_rad_{0.04};
  double tf_lookup_timeout_sec_{0.25}, diagnostics_period_sec_{1.0};

  rclcpp::Subscription<Scan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<Correction>::SharedPtr correction_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  LatestOnlyQueue<Scan::ConstSharedPtr> queue_;
  std::thread worker_;

  bool stream_initialized_{false}, extrinsic_cached_{false}, target_ready_{false};
  StreamKey last_key_;
  std::set<std::uint64_t> retired_odom_sessions_;
  std::string odom_frame_, cloud_frame_;
  Eigen::Isometry3d base_T_cloud_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d precision_T_anchor_{Eigen::Isometry3d::Identity()};
  std::deque<Keyframe> keyframes_;
  Cloud::Ptr map_cloud_{new Cloud};
  std::uint64_t matcher_session_id_{0}, submap_generation_{0}, correction_id_{0};
  std::uint64_t scans_since_rebuild_{0};
  std::uint32_t consecutive_rejections_{0};
  RobustSe2Committer committer_;
  small_gicp::RegistrationPCL<Point, Point> registration_;
  mutable std::mutex counters_mutex_;
  Counters counters_;
};
}  // namespace pure_lidar_submap_matcher
