// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair_config.h"

#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include "gtest/gtest.h"

namespace {
std::string serialized(const ecc::RepairConfig& config)
{
  std::ostringstream out;
  config.write(out);
  return out.str();
}
TEST(RepairConfig, PathSectionAndRemovedBackend) {
  ecc::RepairConfig config;
  std::istringstream input("[path]\nstrategy=feedback\nplacement=final\nverification=final\n");
  config.read(input);
  EXPECT_EQ(config.path.strategy, ecc::PathStrategy::Feedback);
  EXPECT_EQ(config.path.placement, ecc::PathPlacement::Final);
  EXPECT_EQ(config.path.verification, ecc::PathVerification::Final);
  EXPECT_EQ(serialized(config).find("gobi.backend"), std::string::npos);
  EXPECT_THROW(config.set("gobi.backend", "cuda"), std::invalid_argument);
  EXPECT_THROW(config.set("path.min_improvement_ns", "1e100"), std::invalid_argument);
  config.path.strategy = static_cast<ecc::PathStrategy>(99);
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(RepairConfig, PathUnlimitedBudgetsPreserveZeroAreaConstraint)
{
  ecc::RepairConfig config;
  config.set("path.max_trials", "0");
  config.set("path.max_area_growth", "-1");
  EXPECT_NE(serialized(config).find("path.max_trials=0\n"),
            std::string::npos);
  EXPECT_NE(serialized(config).find("path.max_area_growth=-1\n"),
            std::string::npos);
  EXPECT_THROW(config.set("path.max_area_growth", "-0.5"),
               std::invalid_argument);
  EXPECT_NO_THROW(config.set("path.max_area_growth", "0"));
  EXPECT_NO_THROW(config.set("path.strategy", "feedback"));
  config.set("path.min_relative_improvement", "0.001");
  EXPECT_NE(serialized(config).find("path.min_relative_improvement="),
            std::string::npos);
  for(const auto* value : {"-0.1", "1.1", "nan", "inf"})
    EXPECT_THROW(config.set("path.min_relative_improvement", value),
                 std::invalid_argument);
}

TEST(RepairConfig, PathPlacementModeRoundTripsAndRejectsUnknownModes)
{
  ecc::RepairConfig config;
  EXPECT_EQ(config.pathOptions().placement, ecc::PathPlacement::Trial);
  config.set("path.placement", "final");
  EXPECT_EQ(config.pathOptions().placement, ecc::PathPlacement::Final);
  EXPECT_NE(serialized(config).find("path.placement=final\n"), std::string::npos);
  EXPECT_THROW(config.set("path.placement", "off"), std::invalid_argument);
}

TEST(RepairConfig, PathGainScopesRoundTrip)
{
  for(const auto* scope : {"global", "current_global", "endpoints"}) {
    ecc::RepairConfig config;
    config.set("path.gain_scope", scope);
    EXPECT_NE(serialized(config).find(std::string("path.gain_scope=") + scope + "\n"),
              std::string::npos);
  }
  ecc::RepairConfig config;
  EXPECT_THROW(config.set("path.gain_scope", "unknown"), std::invalid_argument);
}

TEST(RepairConfig, PathRejectionStopIsOptionalAndRoundTrips)
{
  ecc::RepairConfig config;
  EXPECT_EQ(config.pathOptions().max_consecutive_rejections, 0u);
  config.set("path.max_consecutive_rejections", "32");
  EXPECT_EQ(config.pathOptions().max_consecutive_rejections, 32u);
  EXPECT_NE(serialized(config).find("path.max_consecutive_rejections=32\n"), std::string::npos);
  for(const auto* value : {"-1", "1.5", "nan", "2147483648"})
    EXPECT_THROW(config.set("path.max_consecutive_rejections", value), std::invalid_argument);
  config.set("path.max_consecutive_rejections", "0");
  EXPECT_EQ(config.pathOptions().max_consecutive_rejections, 0u);
}

TEST(RepairConfig, PathVerificationModeRoundTrips)
{
  ecc::RepairConfig config;
  EXPECT_EQ(config.pathOptions().verification, ecc::PathVerification::Trial);
  config.set("path.verification", "final");
  EXPECT_EQ(config.pathOptions().verification, ecc::PathVerification::Final);
  EXPECT_NE(serialized(config).find("path.verification=final\n"), std::string::npos);
  EXPECT_THROW(config.set("path.verification", "off"), std::invalid_argument);
}

TEST(RepairConfig, PathFeedbackControlsRoundTripWithoutChangingAcceptance)
{
  ecc::RepairConfig config;
  const auto before = config.pathOptions();
  for(const auto* key : {"path.failure_cache", "path.calibrate",
                         "path.diversify"}) {
    config.set(key, "false");
    EXPECT_NE(serialized(config).find(std::string(key) + "=false\n"), std::string::npos);
    EXPECT_THROW(config.set(key, "invalid"), std::invalid_argument);
  }
  const auto options = config.pathOptions();
  EXPECT_FALSE(options.failure_cache);
  EXPECT_FALSE(options.calibrate);
  EXPECT_FALSE(options.diversify);
  EXPECT_FALSE(options.adaptive_search);
  EXPECT_FALSE(options.endpoint_coverage);
  EXPECT_FALSE(options.group_schedule);
  EXPECT_TRUE(options.timing_precheck);
  for(const auto* key : {"path.adaptive_search", "path.timing_precheck",
                         "path.endpoint_coverage", "path.group_schedule"}) {
    config.set(key, "true");
    EXPECT_NE(serialized(config).find(std::string(key) + "=true\n"), std::string::npos);
    config.set(key, "false");
    EXPECT_NE(serialized(config).find(std::string(key) + "=false\n"), std::string::npos);
    EXPECT_THROW(config.set(key, "invalid"), std::invalid_argument);
  }
  EXPECT_EQ(options.min_improvement, before.min_improvement);
  EXPECT_EQ(options.min_relative_improvement, before.min_relative_improvement);
  config.set("path.incremental_analysis", "true");
  EXPECT_TRUE(config.pathOptions().incremental_analysis);
  EXPECT_NE(serialized(config).find("path.incremental_analysis=true\n"), std::string::npos);
  EXPECT_THROW(config.set("path.incremental_analysis", "invalid"), std::invalid_argument);
  config.set("path.reuse_rejections", "true");
  EXPECT_TRUE(config.pathOptions().reuse_rejections);
  EXPECT_NE(serialized(config).find("path.reuse_rejections=true\n"), std::string::npos);
  EXPECT_THROW(config.set("path.reuse_rejections", "invalid"), std::invalid_argument);
  config.set("path.incremental_parasitics", "true");
  EXPECT_TRUE(config.pathOptions().incremental_parasitics);
  EXPECT_NE(serialized(config).find("path.incremental_parasitics=true\n"), std::string::npos);
  EXPECT_THROW(config.set("path.incremental_parasitics", "invalid"), std::invalid_argument);
}

TEST(RepairConfig, SharesCommonConstraintsAndRuntimeHooks)
{
  ecc::RepairConfig config;
  config.set("common.max_fanout", "12");
  config.set("common.allow_gate_resize", "false");
  config.set("setup.max_trials", "3");
  config.set("setup.skip_duplicate_trials", "false");
  config.set("setup.defer_rollback_refresh", "false");
  config.set("path.max_paths", "5");
  config.set("path.max_candidates", "3");
  config.set("path.max_trials", "4");
  config.set("path.batch_size", "3");
  config.set("path.strategy", "sequential");
  config.set("path.fanout_repair", "false");
  config.set("path.recovery", "false");
  config.set("path.tree_candidates", "false");
  config.set("path.gain_scope", "global");
  EXPECT_THROW(config.set("path.gain_scope", "unknown"), std::invalid_argument);
  config.set("path.max_seconds", "1.5");
  config.set("path.max_refreshes", "16");
  config.set("path.max_inserted_buffers", "6");
  config.set("path.max_area_growth", "0.1");
  config.set("common.allow_load_downsize", "false");
  config.set("common.load_downsize_slack_margin_ns", "0.01");
  config.set("design.max_load_downsizes_per_pass", "3");
  config.set("common.vt_suffixes", "_R,_L,_SL");
  config.set("power.objective", "total");
  config.set("power.slack_margin_ns", "0.02");
  config.set("vg.max_buffers", "7");
  config.set("vg.backend", "cuda");
  config.set("vg.cuda_max_memory_mb", "256");
  config.set("vg.max_group_loads", "4");
  config.set("vg.min_gain", "0.25");
  config.set("vg.require_drv_reduction", "false");
  config.set("gobi.max_buffers", "9");
  config.set("gobi.max_nets", "3");
  auto power = config.powerOptions();
  EXPECT_EQ(power.objective, ecc::PowerObjective::Total);
  EXPECT_FLOAT_EQ(power.slack_margin, 2e-11f);
  EXPECT_EQ(power.common.vt_suffixes.size(), 3);
  config.design.max_passes = 2;
  unsigned prepares = 0;
  config.design.prepare = [&]() { ++prepares; };
  auto setup = config.setupOptions();
  const auto path = config.pathOptions();
  EXPECT_EQ(setup.common.max_fanout, 12);
  EXPECT_FALSE(path.fanout_repair);
  EXPECT_FALSE(path.recovery);
  EXPECT_FALSE(path.tree_candidates);
  EXPECT_EQ(path.gain_scope, ecc::PathGainScope::Global);
  EXPECT_FALSE(setup.common.allow_gate_resize);
  EXPECT_FALSE(setup.common.allow_load_downsize);
  EXPECT_FLOAT_EQ(setup.common.load_downsize_slack_margin, 1e-11f);
  EXPECT_EQ(config.design.max_load_downsizes_per_pass, 3);
  EXPECT_EQ(setup.max_trials, 3);
  EXPECT_FALSE(setup.skip_duplicate_trials);
  EXPECT_FALSE(setup.defer_rollback_refresh);
  EXPECT_EQ(path.max_paths, 5u);
  EXPECT_EQ(path.max_candidates, 3u);
  EXPECT_EQ(path.max_trials, 4u);
  EXPECT_EQ(path.batch_size, 3u);
  EXPECT_EQ(path.strategy, ecc::PathStrategy::Sequential);
  EXPECT_DOUBLE_EQ(path.max_seconds, 1.5);
  EXPECT_EQ(path.max_refreshes, 16u);
  EXPECT_EQ(path.max_inserted_buffers, 6u);
  EXPECT_DOUBLE_EQ(path.max_area_growth, 0.1);
  EXPECT_EQ(setup.max_passes, ecc::SetupRepairSettings{}.max_passes);
  EXPECT_EQ(config.vg.max_buffers, 7u);
  EXPECT_EQ(config.vg.backend, ecc::VgBackend::Cuda);
  EXPECT_EQ(config.vg.cuda_max_memory_mb, 256u);
  EXPECT_EQ(config.vg.max_group_loads, 4u);
  EXPECT_FLOAT_EQ(config.vg.min_gain, 0.25f);
  EXPECT_FALSE(config.vg.require_drv_reduction);
  EXPECT_EQ(config.gobi.max_buffers, 9u);
  EXPECT_EQ(config.gobi.max_nets, 3u);
  setup.common.prepare();
  EXPECT_EQ(prepares, 1);
}
TEST(RepairConfig, SectionsUnitsAndRoundTrip)
{
  ecc::RepairConfig config;
  std::istringstream input(
      "# comment\n[setup]\nmax_passes = 3\nmin_improvement_ns = 0.002\n"
      "[runtime]\nsta_threads = 2\n[common]\nmargin=0.15 # margin\n"
      "vt_suffixes=_R,_L,_SL\n[power]\nobjective=total\n");
  config.read(input);
  EXPECT_EQ(config.setup.max_passes, 3);
  EXPECT_FLOAT_EQ(config.setup.min_improvement, 2e-12f);
  EXPECT_EQ(config.sta_threads, 2);
  config.set("vg.max_buffers", "3");
  config.set("vg.max_group_loads", "2");
  EXPECT_EQ(config.power.objective, ecc::PowerObjective::Total);
  EXPECT_EQ(config.design.vt_suffixes.size(), 3);
  auto text = serialized(config);
  ecc::RepairConfig restored;
  std::istringstream saved(text);
  restored.read(saved);
  EXPECT_EQ(serialized(restored), text);
}
TEST(RepairConfig, RejectsMalformedValuesAtomically)
{
  ecc::RepairConfig config;
  const auto original = serialized(config);
  const std::pair<const char*, const char*> invalid[]
      = {{"common.vt_suffixes", ""},
         {"common.vt_suffixes", "_R"},
         {"common.vt_suffixes", "_R,_R"},
         {"common.vt_suffixes", "L,SL"},
         {"common.vt_suffixes", "_R,_L,"},
         {"power.objective", "area"},
         {"power.batch_size", "0"},
         {"power.slack_margin_ns", "nan"},
         {"power.min_relative_improvement", "1"},
         {"power.min_absolute_improvement_w", "-1"},
         {"setup.max_trials", "-1"},
         {"setup.max_passes", "1.5"},
         {"setup.max_trials", "2147483648"},
         {"setup.max_passes", "2junk"},
         {"setup.max_area_growth", "nan"},
         {"setup.max_area_growth", "inf"},
         {"common.margin", "1"},
         {"common.max_fanout", "1"},
         {"common.allow_gate_resize", "yes"},
         {"common.allow_load_downsize", "yes"},
         {"common.load_downsize_slack_margin_ns", "-1"},
         {"common.load_downsize_slack_margin_ns", "nan"},
         {"design.max_load_downsizes_per_pass", "-1"},
         {"setup.batch_backoff", "0"},
         {"setup.batch_backoff", "1"},
         {"vg.max_group_loads", "1"},
         {"vg.backend", "opencl"},
         {"vg.cuda_max_memory_mb", "0"},
         {"vg.min_gain", "-1"},
         {"vg.require_drv_reduction", "maybe"},
         {"setup.batch_size", "0"},
         {"common.recovered_slew_fraction", "0"},
         {"common.overload_group_fraction", "1"},
         {"setup.min_improvement_ns", "-1"},
         {"runtime.sta_threads", "0"},
         {"tolerances.tns_relative", "1"},
         {"path.strategy", "invalid"},
         {"path.max_seconds", "nan"},
         {"path.max_seconds", "-1"},
         {"setup.typo", "3"}};
  for (const auto& [key, value] : invalid) {
    EXPECT_THROW(config.set(key, value), std::exception) << key;
    EXPECT_EQ(serialized(config), original);
  }
  std::istringstream bad("setup.max_trials=1\nsetup.batch_size=0\n");
  try {
    config.read(bad, "bad.ini");
    FAIL();
  } catch (const std::exception& error) {
    EXPECT_NE(std::string(error.what()).find("bad.ini:2:"), std::string::npos);
  }
  EXPECT_EQ(serialized(config), original);
}
TEST(RepairConfig, ZeroBudgetsAreValidButApiNaNIsNot)
{
  ecc::RepairConfig config;
  for (const auto* key : {"design.max_passes",
                          "design.max_inserted_buffers",
                          "setup.max_passes",
                          "setup.max_trials",
                          "setup.max_inserted_buffers",
                          "setup.max_area_growth"})
    EXPECT_NO_THROW(config.set(key, "0"));
  auto options = config.setupOptions();
  options.common.tolerances.wns_absolute
      = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(options.validate(), std::invalid_argument);
}

}  // namespace

TEST(RepairConfig, VgSetupScopeRoundTripsAndRejectsUnknownScope) {
  ecc::RepairConfig c;
  EXPECT_EQ(c.vg.scope,ecc::VgScope::Drv);
  c.set("vg.scope","setup");
  std::ostringstream out;c.write(out);std::istringstream in(out.str());
  ecc::RepairConfig restored;restored.read(in);
  EXPECT_EQ(restored.vg.scope,ecc::VgScope::Setup);
  EXPECT_THROW(c.set("vg.scope","invalid"),std::invalid_argument);
}
