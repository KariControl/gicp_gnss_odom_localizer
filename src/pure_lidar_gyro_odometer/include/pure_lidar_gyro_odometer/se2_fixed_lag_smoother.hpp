// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

namespace pure_gyro_odometer
{
namespace se2
{

inline double normalizeYaw(double yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  while (yaw > kPi) yaw -= kTwoPi;
  while (yaw < -kPi) yaw += kTwoPi;
  return yaw;
}

struct Pose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RelativeFactor
{
  double dx{0.0};
  double dy{0.0};
  double dyaw_scan{0.0};
  double dyaw_imu{0.0};
  bool has_imu_yaw{true};
  double fitness{std::numeric_limits<double>::infinity()};
  bool converged{false};
  bool stationary{false};
  bool wheel_assisted{false};

  // Optional normalized SE(2) scan information matrix in row-major order for
  // residual [dx, dy, dyaw]. This preserves the weak direction reported by the
  // registration Hessian instead of treating every scan direction equally.
  bool has_scan_information{false};
  std::array<double, 9> scan_information{{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0}};

};

struct Config
{
  int window_size{20};
  int max_iterations{5};

  double scan_weight{1.0};
  double imu_weight{2.0};
  double smoothness_weight{0.5};
  double fitness_sigma{1.0};
  double min_scan_weight{0.1};
  double max_scan_weight{5.0};

  bool zupt_enable{false};
  double zupt_weight_translation{25.0};
  double zupt_weight_yaw{25.0};

  bool nhc_enable{false};
  double nhc_weight_lateral{2.0};
  double nhc_huber_delta_m{0.10};

  double prior_weight_position{1.0e6};
  double prior_weight_yaw{1.0e6};
  double diagonal_regularization{1.0e-8};
  double convergence_step_norm{1.0e-6};
  double max_iteration_translation_step_m{0.75};
  double max_iteration_yaw_step_rad{0.35};
  double max_solution_position_correction_m{2.0};
  double max_solution_yaw_correction_rad{0.35};
};

namespace detail
{

inline bool finitePose(const Pose & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

inline bool finiteFactor(const RelativeFactor & factor)
{
  const bool relative_ok =
    std::isfinite(factor.dx) && std::isfinite(factor.dy) &&
    std::isfinite(factor.dyaw_scan) && std::isfinite(factor.dyaw_imu);
  bool scan_information_ok = true;
  if (factor.has_scan_information) {
    const auto & h = factor.scan_information;
    scan_information_ok = std::all_of(
      h.begin(), h.end(), [](double value) {return std::isfinite(value);});

    const double scale = std::max({
      1.0, std::fabs(h[0]), std::fabs(h[4]), std::fabs(h[8]),
      std::fabs(h[1]), std::fabs(h[2]), std::fabs(h[5])});
    const double symmetry_tolerance = 1.0e-9 * scale;
    scan_information_ok = scan_information_ok &&
      std::fabs(h[1] - h[3]) <= symmetry_tolerance &&
      std::fabs(h[2] - h[6]) <= symmetry_tolerance &&
      std::fabs(h[5] - h[7]) <= symmetry_tolerance;

    // A symmetric 3x3 matrix is positive semidefinite iff all principal
    // minors are non-negative.  The tolerance permits a rank-deficient scan
    // Hessian but rejects an invalid indefinite information matrix.
    const double tolerance = 1.0e-10 * scale * scale * scale;
    const double minor_xy = h[0] * h[4] - h[1] * h[3];
    const double minor_xyaw = h[0] * h[8] - h[2] * h[6];
    const double minor_yyaw = h[4] * h[8] - h[5] * h[7];
    const double determinant =
      h[0] * (h[4] * h[8] - h[5] * h[7]) -
      h[1] * (h[3] * h[8] - h[5] * h[6]) +
      h[2] * (h[3] * h[7] - h[4] * h[6]);
    scan_information_ok = scan_information_ok &&
      h[0] >= -tolerance && h[4] >= -tolerance && h[8] >= -tolerance &&
      minor_xy >= -tolerance && minor_xyaw >= -tolerance &&
      minor_yyaw >= -tolerance && determinant >= -tolerance;
  }
  return relative_ok && scan_information_ok;
}

inline double huberScale(double residual, double delta)
{
  if (!(delta > 0.0) || !std::isfinite(residual)) return 1.0;
  const double magnitude = std::fabs(residual);
  return magnitude <= delta ? 1.0 : delta / magnitude;
}

inline double clampedScanWeight(const RelativeFactor & factor, const Config & config)
{
  const double sigma = std::max(1.0e-6, config.fitness_sigma);
  double quality = 1.0;
  if (std::isfinite(factor.fitness)) {
    quality = std::exp(-std::max(0.0, factor.fitness) / sigma);
  } else {
    quality = 0.0;
  }
  if (!factor.converged) quality *= 0.1;

  const double raw = std::max(0.0, config.scan_weight) * quality;
  return std::max(
    std::max(0.0, config.min_scan_weight),
    std::min(std::max(config.min_scan_weight, config.max_scan_weight), raw));
}

inline Pose propagate(const Pose & pose, const RelativeFactor & factor, const Config & config)
{
  const double scan_weight = clampedScanWeight(factor, config);
  const double imu_weight = factor.has_imu_yaw ? std::max(0.0, config.imu_weight) : 0.0;
  const double total_weight = std::max(1.0e-9, scan_weight + imu_weight);
  const double dyaw = normalizeYaw(
    (scan_weight * factor.dyaw_scan + imu_weight * factor.dyaw_imu) /
    total_weight);
  const double yaw_mid = pose.yaw + 0.5 * dyaw;
  const double c = std::cos(yaw_mid);
  const double s = std::sin(yaw_mid);
  return Pose{
    pose.x + c * factor.dx - s * factor.dy,
    pose.y + s * factor.dx + c * factor.dy,
    normalizeYaw(pose.yaw + dyaw)};
}

class DenseSystem
{
public:
  explicit DenseSystem(std::size_t dimension)
  : dimension_(dimension), h_(dimension * dimension, 0.0), g_(dimension, 0.0)
  {
  }

  double & h(std::size_t row, std::size_t col)
  {
    return h_[row * dimension_ + col];
  }

  double & g(std::size_t row)
  {
    return g_[row];
  }

  const std::vector<double> & matrix() const {return h_;}
  const std::vector<double> & gradient() const {return g_;}
  std::size_t dimension() const {return dimension_;}

private:
  std::size_t dimension_;
  std::vector<double> h_;
  std::vector<double> g_;
};

inline void addScalarResidual(
  DenseSystem & system, std::size_t block_a, const double jacobian_a[3],
  std::size_t block_b, const double jacobian_b[3], bool has_block_b,
  double residual, double weight)
{
  if (!(weight > 0.0) || !std::isfinite(residual)) return;
  for (std::size_t r = 0; r < 3; ++r) {
    const std::size_t row_a = block_a + r;
    system.g(row_a) += weight * jacobian_a[r] * residual;
    for (std::size_t c = 0; c < 3; ++c) {
      const std::size_t col_a = block_a + c;
      system.h(row_a, col_a) += weight * jacobian_a[r] * jacobian_a[c];
    }
  }

  if (!has_block_b) return;
  for (std::size_t r = 0; r < 3; ++r) {
    const std::size_t row_b = block_b + r;
    system.g(row_b) += weight * jacobian_b[r] * residual;
    for (std::size_t c = 0; c < 3; ++c) {
      const std::size_t col_b = block_b + c;
      system.h(row_b, col_b) += weight * jacobian_b[r] * jacobian_b[c];
      system.h(block_a + r, col_b) += weight * jacobian_a[r] * jacobian_b[c];
      system.h(row_b, block_a + c) += weight * jacobian_b[r] * jacobian_a[c];
    }
  }
}

inline void addVectorResidual(
  DenseSystem & system,
  std::size_t block_a,
  const double jacobian_a[3][3],
  std::size_t block_b,
  const double jacobian_b[3][3],
  const double residual[3],
  const std::array<double, 9> & information,
  double weight)
{
  if (!(weight > 0.0)) return;

  double weighted_residual[3]{0.0, 0.0, 0.0};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      weighted_residual[row] += information[3 * row + col] * residual[col];
    }
  }

  for (std::size_t state_row = 0; state_row < 3; ++state_row) {
    double gradient_a = 0.0;
    double gradient_b = 0.0;
    for (std::size_t residual_row = 0; residual_row < 3; ++residual_row) {
      gradient_a += jacobian_a[residual_row][state_row] * weighted_residual[residual_row];
      gradient_b += jacobian_b[residual_row][state_row] * weighted_residual[residual_row];
    }
    system.g(block_a + state_row) += weight * gradient_a;
    system.g(block_b + state_row) += weight * gradient_b;

    for (std::size_t state_col = 0; state_col < 3; ++state_col) {
      double h_aa = 0.0;
      double h_ab = 0.0;
      double h_ba = 0.0;
      double h_bb = 0.0;
      for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
          const double info = information[3 * r + c];
          h_aa += jacobian_a[r][state_row] * info * jacobian_a[c][state_col];
          h_ab += jacobian_a[r][state_row] * info * jacobian_b[c][state_col];
          h_ba += jacobian_b[r][state_row] * info * jacobian_a[c][state_col];
          h_bb += jacobian_b[r][state_row] * info * jacobian_b[c][state_col];
        }
      }
      system.h(block_a + state_row, block_a + state_col) += weight * h_aa;
      system.h(block_a + state_row, block_b + state_col) += weight * h_ab;
      system.h(block_b + state_row, block_a + state_col) += weight * h_ba;
      system.h(block_b + state_row, block_b + state_col) += weight * h_bb;
    }
  }
}

inline bool solveLinearSystem(
  const DenseSystem & system, std::vector<double> & solution, double regularization)
{
  const std::size_t n = system.dimension();
  if (n == 0 || system.matrix().size() != n * n || system.gradient().size() != n) {
    return false;
  }

  std::vector<double> a = system.matrix();
  std::vector<double> b(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    a[i * n + i] += std::max(0.0, regularization);
    b[i] = -system.gradient()[i];
  }

  // Dense Gaussian elimination with partial pivoting. The fixed-lag systems are
  // small (normally <= 80 variables), so this avoids another runtime dependency.
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    double pivot_abs = std::fabs(a[col * n + col]);
    for (std::size_t row = col + 1; row < n; ++row) {
      const double candidate = std::fabs(a[row * n + col]);
      if (candidate > pivot_abs) {
        pivot_abs = candidate;
        pivot = row;
      }
    }
    if (!(pivot_abs > 1.0e-14) || !std::isfinite(pivot_abs)) return false;

    if (pivot != col) {
      for (std::size_t c = col; c < n; ++c) {
        std::swap(a[col * n + c], a[pivot * n + c]);
      }
      std::swap(b[col], b[pivot]);
    }

    const double diagonal = a[col * n + col];
    for (std::size_t row = col + 1; row < n; ++row) {
      const double multiplier = a[row * n + col] / diagonal;
      if (!std::isfinite(multiplier)) return false;
      a[row * n + col] = 0.0;
      for (std::size_t c = col + 1; c < n; ++c) {
        a[row * n + c] -= multiplier * a[col * n + c];
      }
      b[row] -= multiplier * b[col];
    }
  }

  solution.assign(n, 0.0);
  for (std::size_t reverse = 0; reverse < n; ++reverse) {
    const std::size_t row = n - 1 - reverse;
    double rhs = b[row];
    for (std::size_t c = row + 1; c < n; ++c) {
      rhs -= a[row * n + c] * solution[c];
    }
    const double diagonal = a[row * n + row];
    if (!(std::fabs(diagonal) > 1.0e-14) || !std::isfinite(diagonal)) return false;
    solution[row] = rhs / diagonal;
    if (!std::isfinite(solution[row])) return false;
  }
  return true;
}

}  // namespace detail

class FixedLagSmoother
{
public:
  explicit FixedLagSmoother(Config config = Config{})
  : config_(sanitizeConfig(std::move(config)))
  {
  }

  void setConfig(Config config)
  {
    config_ = sanitizeConfig(std::move(config));
    reset(base_pose_);
  }

  const Config & config() const {return config_;}

  void reset(const Pose & pose)
  {
    base_pose_ = detail::finitePose(pose) ? pose : Pose{};
    base_pose_.yaw = normalizeYaw(base_pose_.yaw);
    factors_.clear();
    last_solution_.clear();
    initialized_ = true;
    last_pose_ = base_pose_;
  }

  bool initialized() const {return initialized_;}
  std::size_t factorCount() const {return factors_.size();}
  const Pose & pose() const {return last_pose_;}

  bool addFactor(const RelativeFactor & factor, Pose & output)
  {
    if (!initialized_ || !detail::finiteFactor(factor)) return false;

    const auto old_factors = factors_;
    const auto old_solution = last_solution_;
    const Pose old_base = base_pose_;
    const Pose old_pose = last_pose_;

    trimWindowForNewFactor();
    factors_.push_back(factor);

    std::vector<double> state;
    initializeState(state);
    const std::size_t predicted_index = 3 * factors_.size();
    const Pose predicted{
      state[predicted_index], state[predicted_index + 1],
      normalizeYaw(state[predicted_index + 2])};
    if (!optimize(state) || !stateFinite(state)) {
      factors_ = old_factors;
      last_solution_ = old_solution;
      base_pose_ = old_base;
      last_pose_ = old_pose;
      return false;
    }

    const std::size_t final_index = 3 * factors_.size();
    output = Pose{state[final_index], state[final_index + 1], normalizeYaw(state[final_index + 2])};
    if (!detail::finitePose(output)) {
      factors_ = old_factors;
      last_solution_ = old_solution;
      base_pose_ = old_base;
      last_pose_ = old_pose;
      return false;
    }

    const double position_correction = std::hypot(output.x - predicted.x, output.y - predicted.y);
    const double yaw_correction = std::fabs(normalizeYaw(output.yaw - predicted.yaw));
    if (position_correction > config_.max_solution_position_correction_m ||
      yaw_correction > config_.max_solution_yaw_correction_rad)
    {
      factors_ = old_factors;
      last_solution_ = old_solution;
      base_pose_ = old_base;
      last_pose_ = old_pose;
      return false;
    }

    last_solution_ = std::move(state);
    last_pose_ = output;
    return true;
  }

private:
  static Config sanitizeConfig(Config config)
  {
    config.window_size = std::max(1, config.window_size);
    config.max_iterations = std::max(1, config.max_iterations);
    config.scan_weight = std::max(0.0, config.scan_weight);
    config.imu_weight = std::max(0.0, config.imu_weight);
    config.smoothness_weight = std::max(0.0, config.smoothness_weight);
    config.fitness_sigma = std::max(1.0e-6, config.fitness_sigma);
    config.min_scan_weight = std::max(0.0, config.min_scan_weight);
    config.max_scan_weight = std::max(config.min_scan_weight, config.max_scan_weight);
    config.zupt_weight_translation = std::max(0.0, config.zupt_weight_translation);
    config.zupt_weight_yaw = std::max(0.0, config.zupt_weight_yaw);
    config.nhc_weight_lateral = std::max(0.0, config.nhc_weight_lateral);
    config.nhc_huber_delta_m = std::max(0.0, config.nhc_huber_delta_m);
    config.prior_weight_position = std::max(1.0, config.prior_weight_position);
    config.prior_weight_yaw = std::max(1.0, config.prior_weight_yaw);
    config.diagonal_regularization = std::max(0.0, config.diagonal_regularization);
    config.convergence_step_norm = std::max(1.0e-12, config.convergence_step_norm);
    config.max_iteration_translation_step_m =
      std::max(1.0e-6, config.max_iteration_translation_step_m);
    config.max_iteration_yaw_step_rad =
      std::max(1.0e-6, config.max_iteration_yaw_step_rad);
    config.max_solution_position_correction_m =
      std::max(1.0e-6, config.max_solution_position_correction_m);
    config.max_solution_yaw_correction_rad =
      std::max(1.0e-6, config.max_solution_yaw_correction_rad);
    return config;
  }

  void trimWindowForNewFactor()
  {
    while (static_cast<int>(factors_.size()) >= config_.window_size) {
      const std::size_t expected_dimension = 3 * (factors_.size() + 1);
      if (last_solution_.size() == expected_dimension && expected_dimension >= 6) {
        base_pose_ = Pose{
          last_solution_[3], last_solution_[4], normalizeYaw(last_solution_[5])};
        last_solution_.erase(last_solution_.begin(), last_solution_.begin() + 3);
      } else {
        base_pose_ = detail::propagate(base_pose_, factors_.front(), config_);
        last_solution_.clear();
      }
      factors_.pop_front();
    }
  }

  void initializeState(std::vector<double> & state) const
  {
    const std::size_t state_count = factors_.size() + 1;
    const std::size_t dimension = 3 * state_count;
    state.assign(dimension, 0.0);

    // Warm-start all previously optimized states when the fixed-lag window has not shifted.
    if (last_solution_.size() + 3 == dimension && !last_solution_.empty()) {
      std::copy(last_solution_.begin(), last_solution_.end(), state.begin());
      const std::size_t previous_index = dimension - 6;
      const Pose previous{
        state[previous_index], state[previous_index + 1], state[previous_index + 2]};
      const Pose current = detail::propagate(previous, factors_.back(), config_);
      state[dimension - 3] = current.x;
      state[dimension - 2] = current.y;
      state[dimension - 1] = current.yaw;
      return;
    }

    state[0] = base_pose_.x;
    state[1] = base_pose_.y;
    state[2] = base_pose_.yaw;
    Pose pose = base_pose_;
    for (std::size_t i = 0; i < factors_.size(); ++i) {
      pose = detail::propagate(pose, factors_[i], config_);
      const std::size_t index = 3 * (i + 1);
      state[index] = pose.x;
      state[index + 1] = pose.y;
      state[index + 2] = pose.yaw;
    }
  }

  static bool stateFinite(const std::vector<double> & state)
  {
    return std::all_of(state.begin(), state.end(), [](double value) {return std::isfinite(value);});
  }

  void buildSystem(const std::vector<double> & state, detail::DenseSystem & system) const
  {
    const std::size_t state_count = factors_.size() + 1;

    // Anchor the oldest state. This is the marginalization prior for the fixed-lag window.
    for (std::size_t component = 0; component < 2; ++component) {
      const double target = component == 0 ? base_pose_.x : base_pose_.y;
      const double residual = state[component] - target;
      system.h(component, component) += config_.prior_weight_position;
      system.g(component) += config_.prior_weight_position * residual;
    }
    {
      const double residual = normalizeYaw(state[2] - base_pose_.yaw);
      system.h(2, 2) += config_.prior_weight_yaw;
      system.g(2) += config_.prior_weight_yaw * residual;
    }

    for (std::size_t i = 1; i < state_count; ++i) {
      const std::size_t index = 3 * i;
      system.h(index, index) += 1.0e-9;
      system.h(index + 1, index + 1) += 1.0e-9;
      system.h(index + 2, index + 2) += 1.0e-9;
    }

    for (std::size_t i = 0; i < factors_.size(); ++i) {
      const RelativeFactor & factor = factors_[i];
      const std::size_t index_i = 3 * i;
      const std::size_t index_j = 3 * (i + 1);

      const double xi = state[index_i];
      const double yi = state[index_i + 1];
      const double yaw_i = state[index_i + 2];
      const double xj = state[index_j];
      const double yj = state[index_j + 1];
      const double yaw_j = state[index_j + 2];

      const double delta_x_world = xj - xi;
      const double delta_y_world = yj - yi;
      const double c = std::cos(yaw_i);
      const double s = std::sin(yaw_i);
      const double predicted_dx = c * delta_x_world + s * delta_y_world;
      const double predicted_dy = -s * delta_x_world + c * delta_y_world;

      const double scan_weight = detail::clampedScanWeight(factor, config_);
      const double imu_weight = factor.has_imu_yaw ? std::max(0.0, config_.imu_weight) : 0.0;

      const double jacobian_i_dx[3]{-c, -s, predicted_dy};
      const double jacobian_j_dx[3]{c, s, 0.0};
      const double jacobian_i_dy[3]{s, -c, -predicted_dx};
      const double jacobian_j_dy[3]{-s, c, 0.0};
      const double jacobian_i_yaw[3]{0.0, 0.0, -1.0};
      const double jacobian_j_yaw[3]{0.0, 0.0, 1.0};

      if (factor.has_scan_information) {
        const double jacobian_i_scan[3][3]{
          {jacobian_i_dx[0], jacobian_i_dx[1], jacobian_i_dx[2]},
          {jacobian_i_dy[0], jacobian_i_dy[1], jacobian_i_dy[2]},
          {jacobian_i_yaw[0], jacobian_i_yaw[1], jacobian_i_yaw[2]}};
        const double jacobian_j_scan[3][3]{
          {jacobian_j_dx[0], jacobian_j_dx[1], jacobian_j_dx[2]},
          {jacobian_j_dy[0], jacobian_j_dy[1], jacobian_j_dy[2]},
          {jacobian_j_yaw[0], jacobian_j_yaw[1], jacobian_j_yaw[2]}};
        const double residual_scan[3]{
          predicted_dx - factor.dx,
          predicted_dy - factor.dy,
          normalizeYaw((yaw_j - yaw_i) - factor.dyaw_scan)};
        detail::addVectorResidual(
          system, index_i, jacobian_i_scan, index_j, jacobian_j_scan,
          residual_scan, factor.scan_information, scan_weight);
      } else {
        detail::addScalarResidual(
          system, index_i, jacobian_i_dx, index_j, jacobian_j_dx, true,
          predicted_dx - factor.dx, scan_weight);
        detail::addScalarResidual(
          system, index_i, jacobian_i_dy, index_j, jacobian_j_dy, true,
          predicted_dy - factor.dy, scan_weight);
        detail::addScalarResidual(
          system, index_i, jacobian_i_yaw, index_j, jacobian_j_yaw, true,
          normalizeYaw((yaw_j - yaw_i) - factor.dyaw_scan), scan_weight);
      }
      if (imu_weight > 0.0) {
        detail::addScalarResidual(
          system, index_i, jacobian_i_yaw, index_j, jacobian_j_yaw, true,
          normalizeYaw((yaw_j - yaw_i) - factor.dyaw_imu), imu_weight);
      }

      if (config_.zupt_enable && factor.stationary) {
        detail::addScalarResidual(
          system, index_i, jacobian_i_dx, index_j, jacobian_j_dx, true,
          predicted_dx, config_.zupt_weight_translation);
        detail::addScalarResidual(
          system, index_i, jacobian_i_dy, index_j, jacobian_j_dy, true,
          predicted_dy, config_.zupt_weight_translation);
        detail::addScalarResidual(
          system, index_i, jacobian_i_yaw, index_j, jacobian_j_yaw, true,
          normalizeYaw(yaw_j - yaw_i), config_.zupt_weight_yaw);
      }

      if (config_.nhc_enable && config_.nhc_weight_lateral > 0.0) {
        const double delta_yaw = normalizeYaw(yaw_j - yaw_i);
        const double half_yaw = 0.5 * delta_yaw;
        const double ch = std::cos(half_yaw);
        const double sh = std::sin(half_yaw);
        const double dx_mid = ch * predicted_dx + sh * predicted_dy;
        const double dy_mid = -sh * predicted_dx + ch * predicted_dy;

        double jacobian_i_nhc[3];
        double jacobian_j_nhc[3];
        for (std::size_t component = 0; component < 3; ++component) {
          jacobian_i_nhc[component] =
            -sh * jacobian_i_dx[component] + ch * jacobian_i_dy[component];
          jacobian_j_nhc[component] =
            -sh * jacobian_j_dx[component] + ch * jacobian_j_dy[component];
        }
        jacobian_i_nhc[2] += 0.5 * dx_mid;
        jacobian_j_nhc[2] -= 0.5 * dx_mid;

        const double weight = config_.nhc_weight_lateral *
          detail::huberScale(dy_mid, config_.nhc_huber_delta_m);
        detail::addScalarResidual(
          system, index_i, jacobian_i_nhc, index_j, jacobian_j_nhc, true,
          dy_mid, weight);
      }

    }

    if (config_.smoothness_weight > 0.0 && state_count >= 3) {
      const double weight = config_.smoothness_weight;
      for (std::size_t i = 1; i + 1 < state_count; ++i) {
        for (std::size_t component = 0; component < 3; ++component) {
          const std::size_t index_m = 3 * (i - 1) + component;
          const std::size_t index_0 = 3 * i + component;
          const std::size_t index_p = 3 * (i + 1) + component;
          double residual = state[index_p] - 2.0 * state[index_0] + state[index_m];
          if (component == 2) residual = normalizeYaw(residual);

          system.h(index_m, index_m) += weight;
          system.h(index_0, index_0) += 4.0 * weight;
          system.h(index_p, index_p) += weight;
          system.h(index_m, index_0) -= 2.0 * weight;
          system.h(index_0, index_m) -= 2.0 * weight;
          system.h(index_m, index_p) += weight;
          system.h(index_p, index_m) += weight;
          system.h(index_0, index_p) -= 2.0 * weight;
          system.h(index_p, index_0) -= 2.0 * weight;
          system.g(index_m) += weight * residual;
          system.g(index_0) -= 2.0 * weight * residual;
          system.g(index_p) += weight * residual;
        }
      }
    }
  }

  double computeCost(const std::vector<double> & state) const
  {
    if (!stateFinite(state)) return std::numeric_limits<double>::infinity();
    const std::size_t state_count = factors_.size() + 1;
    double cost = 0.0;
    auto robust_cost = [](double residual, double delta) {
        const double magnitude = std::fabs(residual);
        if (!(delta > 0.0) || magnitude <= delta) return 0.5 * residual * residual;
        return delta * (magnitude - 0.5 * delta);
      };

    cost += 0.5 * config_.prior_weight_position *
      ((state[0] - base_pose_.x) * (state[0] - base_pose_.x) +
      (state[1] - base_pose_.y) * (state[1] - base_pose_.y));
    const double base_yaw_residual = normalizeYaw(state[2] - base_pose_.yaw);
    cost += 0.5 * config_.prior_weight_yaw * base_yaw_residual * base_yaw_residual;

    for (std::size_t i = 0; i < factors_.size(); ++i) {
      const RelativeFactor & factor = factors_[i];
      const std::size_t index_i = 3 * i;
      const std::size_t index_j = 3 * (i + 1);
      const double yaw_i = state[index_i + 2];
      const double delta_x_world = state[index_j] - state[index_i];
      const double delta_y_world = state[index_j + 1] - state[index_i + 1];
      const double c = std::cos(yaw_i);
      const double s = std::sin(yaw_i);
      const double predicted_dx = c * delta_x_world + s * delta_y_world;
      const double predicted_dy = -s * delta_x_world + c * delta_y_world;
      const double scan_weight = detail::clampedScanWeight(factor, config_);
      const double imu_weight = factor.has_imu_yaw ? std::max(0.0, config_.imu_weight) : 0.0;
      const double residual_dx = predicted_dx - factor.dx;
      const double residual_dy = predicted_dy - factor.dy;
      const double residual_scan_yaw = normalizeYaw(
        (state[index_j + 2] - yaw_i) - factor.dyaw_scan);
      const double residual_imu_yaw = normalizeYaw(
        (state[index_j + 2] - yaw_i) - factor.dyaw_imu);
      if (factor.has_scan_information) {
        const double residual[3]{residual_dx, residual_dy, residual_scan_yaw};
        double quadratic = 0.0;
        for (std::size_t row = 0; row < 3; ++row) {
          for (std::size_t col = 0; col < 3; ++col) {
            quadratic += residual[row] * factor.scan_information[3 * row + col] *
              residual[col];
          }
        }
        cost += 0.5 * scan_weight * std::max(0.0, quadratic);
      } else {
        cost += 0.5 * scan_weight *
          (residual_dx * residual_dx + residual_dy * residual_dy +
          residual_scan_yaw * residual_scan_yaw);
      }
      if (imu_weight > 0.0) {
        cost += 0.5 * imu_weight * residual_imu_yaw * residual_imu_yaw;
      }

      if (config_.zupt_enable && factor.stationary) {
        const double residual_yaw = normalizeYaw(state[index_j + 2] - yaw_i);
        cost += 0.5 * config_.zupt_weight_translation *
          (predicted_dx * predicted_dx + predicted_dy * predicted_dy);
        cost += 0.5 * config_.zupt_weight_yaw * residual_yaw * residual_yaw;
      }

      if (config_.nhc_enable && config_.nhc_weight_lateral > 0.0) {
        const double half_yaw = 0.5 * normalizeYaw(state[index_j + 2] - yaw_i);
        const double dy_mid = -std::sin(half_yaw) * predicted_dx +
          std::cos(half_yaw) * predicted_dy;
        cost += config_.nhc_weight_lateral *
          robust_cost(dy_mid, config_.nhc_huber_delta_m);
      }

    }

    if (config_.smoothness_weight > 0.0 && state_count >= 3) {
      for (std::size_t i = 1; i + 1 < state_count; ++i) {
        for (std::size_t component = 0; component < 3; ++component) {
          double residual =
            state[3 * (i + 1) + component] - 2.0 * state[3 * i + component] +
            state[3 * (i - 1) + component];
          if (component == 2) residual = normalizeYaw(residual);
          cost += 0.5 * config_.smoothness_weight * residual * residual;
        }
      }
    }
    return cost;
  }

  bool optimize(std::vector<double> & state) const
  {
    if (state.empty() || !stateFinite(state)) return false;
    const std::size_t dimension = state.size();

    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
      detail::DenseSystem system(dimension);
      buildSystem(state, system);

      std::vector<double> step;
      bool solved = detail::solveLinearSystem(
        system, step, config_.diagonal_regularization);
      if (!solved) {
        solved = detail::solveLinearSystem(
          system, step, std::max(1.0e-6, config_.diagonal_regularization * 1000.0));
      }
      if (!solved || step.size() != dimension) return false;

      double step_norm_sq = 0.0;
      for (std::size_t i = 0; i < dimension; i += 3) {
        const double translation_norm = std::hypot(step[i], step[i + 1]);
        if (translation_norm > config_.max_iteration_translation_step_m) {
          const double scale = config_.max_iteration_translation_step_m / translation_norm;
          step[i] *= scale;
          step[i + 1] *= scale;
        }
        step[i + 2] = std::max(
          -config_.max_iteration_yaw_step_rad,
          std::min(config_.max_iteration_yaw_step_rad, step[i + 2]));
        step_norm_sq += step[i] * step[i] + step[i + 1] * step[i + 1] +
          step[i + 2] * step[i + 2];
      }

      const double current_cost = computeCost(state);
      bool accepted = false;
      std::vector<double> candidate(dimension, 0.0);
      double line_scale = 1.0;
      for (int trial = 0; trial < 6; ++trial) {
        for (std::size_t i = 0; i < dimension; ++i) {
          candidate[i] = state[i] + line_scale * step[i];
        }
        for (std::size_t i = 2; i < dimension; i += 3) {
          candidate[i] = normalizeYaw(candidate[i]);
        }
        const double candidate_cost = computeCost(candidate);
        if (std::isfinite(candidate_cost) &&
          (!std::isfinite(current_cost) || candidate_cost <= current_cost + 1.0e-10))
        {
          state.swap(candidate);
          accepted = true;
          break;
        }
        line_scale *= 0.5;
      }
      if (!accepted) {
        // A tiny numerical step that cannot improve the cost is effectively converged.
        if (std::sqrt(step_norm_sq) < 10.0 * config_.convergence_step_norm) return true;
        return false;
      }
      if (line_scale * std::sqrt(step_norm_sq) < config_.convergence_step_norm) break;
    }
    return stateFinite(state);
  }

  Config config_;
  Pose base_pose_{};
  Pose last_pose_{};
  std::deque<RelativeFactor> factors_;
  std::vector<double> last_solution_;
  bool initialized_{false};
};

}  // namespace se2
}  // namespace pure_gyro_odometer
