#include "pure_lidar_submap_matcher/submap_policy.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>

namespace p = pure_lidar_submap_matcher;
namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {std::cerr << "FAIL: " << message << '\n'; std::exit(1);}
}

void testExactStreamPolicy()
{
  const p::StreamKey a{1, 2, 3, 100};
  require(p::streamAction(a, {1, 2, 4, 101}) == p::StreamAction::accept,
    "monotonic exact stream");
  require(p::streamAction(a, {1, 3, 4, 101}) == p::StreamAction::reset,
    "generation reset");
  require(p::streamAction({1, 3, 4, 101}, a) == p::StreamAction::stale,
    "late retired generation is ignored, not reset backwards");
  require(p::streamAction(a, {1, 2, 3, 102}) == p::StreamAction::stale,
    "duplicate sequence is ignored, not reset");
  require(p::streamAction(a, {1, 2, 4, 99}) == p::StreamAction::stale,
    "late timestamp is ignored, not reset");
  require(p::streamAction(a, {9, 1, 1, 1}) == p::StreamAction::reset,
    "new odometer session resets the stream");

  std::set<std::uint64_t> retired_sessions{1U};
  const p::StreamKey active{9, 1, 10, 200};
  require(
    p::guardedStreamAction(active, {1, 99, 999, 999}, retired_sessions) ==
    p::StreamAction::stale,
    "late retired odometer session cannot resurrect its submap");
}

void testRobustNConsistentCommit()
{
  p::RobustConfig config;
  config.min_consistent = 3;
  p::RobustSe2Committer committer(config);
  const p::Se2 raw{20.0, -5.0, 0.3};
  require(!committer.add({0.10, -0.05, 0.02}, raw).committed, "one is insufficient");
  require(!committer.add({0.11, -0.04, 0.021}, raw).committed, "two are insufficient");
  const auto result = committer.add({0.09, -0.06, 0.019}, raw);
  require(result.committed && result.consistent_count == 3, "three consistent matches commit");
  require(std::fabs(result.transform.yaw - 0.02) < 0.005, "robust full yaw commit");
}

void testOutlierRejectDoesNotEraseCommitted()
{
  p::RobustSe2Committer committer;
  const p::Se2 raw{5.0, 0.0, 0.0};
  (void)committer.add({0.2, 0.0, 0.03}, raw);
  (void)committer.add({0.21, 0.0, 0.031}, raw);
  require(committer.add({0.19, 0.0, 0.029}, raw).committed, "baseline commit");
  const auto committed = committer.committed();
  const auto outlier = committer.add({4.0, -3.0, 1.0}, raw);
  require(!outlier.committed, "outlier does not commit");
  require(std::fabs(committer.committed().x - committed.x) < 1e-12 &&
    std::fabs(committer.committed().yaw - committed.yaw) < 1e-12,
    "outlier preserves persistent transform");
  committer.clearCandidates();
  require(std::fabs(committer.committed().x - committed.x) < 1e-12,
    "map recovery clears candidates but not committed correction");
}

void testBasePivotStepGate()
{
  p::RobustConfig config;
  config.min_consistent = 2;
  config.max_commit_pivot_step_m = 0.25;
  p::RobustSe2Committer committer(config);
  const p::Se2 raw{100.0, 0.0, 0.0};
  (void)committer.add({0.0, 0.0, 0.001}, raw);
  require(committer.add({0.0, 0.0, 0.001}, raw).committed, "first commit");
  committer.clearCandidates();
  (void)committer.add({0.0, 0.0, 0.02}, raw);
  const auto result = committer.add({0.0, 0.0, 0.02}, raw);
  require(!result.committed && result.reason == "commit_step_gate",
    "rotation lever arm is gated at current vehicle pivot");
}
}  // namespace

int main()
{
  testExactStreamPolicy();
  testRobustNConsistentCommit();
  testOutlierRejectDoesNotEraseCommitted();
  testBasePivotStepGate();
  std::cout << "submap policy tests passed\n";
  return 0;
}
