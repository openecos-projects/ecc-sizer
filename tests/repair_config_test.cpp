// SPDX-FileCopyrightText: 2026 ICT, CAS
// SPDX-License-Identifier: Apache-2.0
#include "repair/config.h"

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
  std::istringstream input("[path]\nplacement=final\nverification=final\n");
  config.read(input);
  EXPECT_THROW(config.set("path.strategy", "sequential"), std::invalid_argument);
  EXPECT_FALSE(config.path.max_trials);
  EXPECT_EQ(config.path.gain_scope, ecc::PathGainScope::Global);
  EXPECT_DOUBLE_EQ(config.path.min_relative_improvement, 0.001);
  EXPECT_EQ(config.path.placement, ecc::PathPlacement::Final);
  EXPECT_EQ(config.path.verification, ecc::PathVerification::Final);
  EXPECT_EQ(serialized(config).find("gobi.backend"), std::string::npos);
  EXPECT_THROW(config.set("gobi.backend", "cuda"), std::invalid_argument);
  EXPECT_THROW(config.set("path.min_improvement_ns", "1e100"), std::invalid_argument);
}

TEST(RepairConfig, PathUnlimitedBudgetsPreserveZeroAreaConstraint)
{
  ecc::RepairConfig config;
  config.set("path.max_trials", "unlimited");
  config.set("path.max_area_growth", "unlimited");
  EXPECT_NE(serialized(config).find("path.max_trials=unlimited\n"),
            std::string::npos);
  EXPECT_NE(serialized(config).find("path.max_area_growth=unlimited\n"),
            std::string::npos);
  EXPECT_THROW(config.set("path.max_area_growth", "-0.5"),
               std::invalid_argument);
  EXPECT_NO_THROW(config.set("path.max_area_growth", "0"));
  for (const auto* key : {"path.max_trials", "path.max_seconds", "path.max_refreshes",
                          "path.max_consecutive_rejections", "path.max_area_growth",
                          "setup.max_trials", "setup.max_area_growth"}) {
    config.set(key, "unlimited");
    EXPECT_NE(serialized(config).find(std::string(key) + "=unlimited\n"), std::string::npos);
    config.set(key, "0");
    EXPECT_NE(serialized(config).find(std::string(key) + "=0\n"), std::string::npos);
  }
  EXPECT_THROW(config.set("path.max_area_growth", "-1"), std::invalid_argument);
  EXPECT_THROW(config.set("path.strategy", "feedback"), std::invalid_argument);
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
  EXPECT_EQ(config.pathOptions().max_consecutive_rejections, 32u);
  config.set("path.max_consecutive_rejections", "unlimited");
  EXPECT_FALSE(config.pathOptions().max_consecutive_rejections);
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
  config.set("common.actions", "buffer,load_downsize");
  config.set("setup.max_trials", "3");
  config.set("setup.skip_duplicate_trials", "false");
  config.set("setup.defer_rollback_refresh", "false");
  config.set("path.max_paths", "5");
  config.set("path.max_candidates", "3");
  config.set("path.max_trials", "4");
  config.set("path.batch_size", "3");
  config.set("path.fanout_repair", "false");
  config.set("path.recovery", "false");
  config.set("path.tree_candidates", "false");
  config.set("path.gain_scope", "global");
  EXPECT_THROW(config.set("path.gain_scope", "unknown"), std::invalid_argument);
  config.set("path.max_seconds", "1.5");
  config.set("path.max_refreshes", "16");
  config.set("path.max_inserted_buffers", "6");
  config.set("path.max_area_growth", "0.1");
  config.set("common.actions", "buffer");
  config.set("common.load_downsize_slack_margin_ns", "0.01");
  config.set("drv.max_load_downsizes_per_pass", "3");
  config.set("common.vt_suffixes", "_R,_L,_SL");
  config.set("power.objective", "total");
  config.set("power.slack_margin_ns", "0.02");
  config.set("vg.max_buffers", "7");
  config.set("vg.backend", "cuda");
  config.set("vg.cuda_max_memory_mb", "256");
  config.set("vg.max_group_loads", "4");
  config.set("vg.min_gain", "0.25");
  config.set("vg.require_drv_non_regression", "false");
  config.set("gobi.max_buffers", "9");
  config.set("gobi.max_nets", "3");
  auto power = config.powerOptions();
  EXPECT_EQ(power.objective, ecc::PowerObjective::Total);
  EXPECT_FLOAT_EQ(power.slack_margin, 2e-11f);
  EXPECT_EQ(power.common.vt_suffixes.size(), 3);
  config.drv.max_passes = 2;
  auto setup = config.setupOptions();
  const auto path = config.pathOptions();
  EXPECT_EQ(setup.common.max_fanout, 12);
  EXPECT_FALSE(path.fanout_repair);
  EXPECT_FALSE(path.recovery);
  EXPECT_FALSE(path.tree_candidates);
  EXPECT_EQ(path.gain_scope, ecc::PathGainScope::Global);
  EXPECT_FALSE(setup.common.actions.resize);
  EXPECT_FALSE(setup.common.actions.load_downsize);
  EXPECT_FLOAT_EQ(setup.common.load_downsize_slack_margin, 1e-11f);
  EXPECT_EQ(config.drv.max_load_downsizes_per_pass, 3);
  EXPECT_EQ(setup.max_trials, 3);
  EXPECT_FALSE(setup.skip_duplicate_trials);
  EXPECT_FALSE(setup.defer_rollback_refresh);
  EXPECT_EQ(path.max_paths, 5u);
  EXPECT_EQ(path.max_candidates, 3u);
  EXPECT_EQ(path.max_trials, 4u);
  EXPECT_EQ(path.batch_size, 3u);
  ASSERT_TRUE(path.max_seconds);
  EXPECT_DOUBLE_EQ(*path.max_seconds, 1.5);
  EXPECT_EQ(path.max_refreshes, 16u);
  EXPECT_EQ(path.max_inserted_buffers, 6u);
  ASSERT_TRUE(path.max_area_growth);
  EXPECT_DOUBLE_EQ(*path.max_area_growth, 0.1);
  EXPECT_EQ(setup.max_passes, ecc::SetupRepairSettings{}.max_passes);
  EXPECT_EQ(config.vg.max_buffers, 7u);
  EXPECT_EQ(config.vg.backend, ecc::VgBackend::Cuda);
  EXPECT_EQ(config.vg.cuda_max_memory_mb, 256u);
  EXPECT_EQ(config.vg.max_group_loads, 4u);
  EXPECT_FLOAT_EQ(config.vg.min_gain, 0.25f);
  EXPECT_FALSE(config.vg.require_drv_non_regression);
  EXPECT_EQ(config.gobi.max_buffers, 9u);
  EXPECT_EQ(config.gobi.max_nets, 3u);
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
  EXPECT_EQ(config.common.vt_suffixes.size(), 3);
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
         {"common.actions.resize", "yes"},
         {"common.actions.load_downsize", "yes"},
         {"common.load_downsize_slack_margin_ns", "-1"},
         {"common.load_downsize_slack_margin_ns", "nan"},
         {"drv.max_load_downsizes_per_pass", "-1"},
         {"setup.batch_backoff", "0"},
         {"setup.batch_backoff", "1"},
         {"vg.max_group_loads", "1"},
         {"vg.backend", "opencl"},
         {"vg.cuda_max_memory_mb", "0"},
         {"vg.min_gain", "-1"},
         {"vg.require_drv_non_regression", "maybe"},
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
  for (const auto* key : {"drv.max_passes",
                          "drv.max_inserted_buffers",
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

TEST(BufferPlanBudget, NeverTruncatesATreeAndSkipsOversizedPlans)
{
    EXPECT_EQ(ecc::selectWholeBufferPlans(std::vector<std::size_t>{65, 40, 30, 24, 1}, 64),
              (std::vector<std::size_t>{1, 3}));
}
TEST(BufferPlanBudget, AbsentLimitIsUnlimitedAndEmptyPlansAreIgnored)
{
    EXPECT_EQ(ecc::selectWholeBufferPlans(std::vector<std::size_t>{0, 100, 5, 0, 10}, std::nullopt),
              (std::vector<std::size_t>{1, 2, 4}));
    EXPECT_TRUE(ecc::selectWholeBufferPlans(std::vector<std::size_t>{}, std::nullopt).empty());
    EXPECT_TRUE(ecc::selectWholeBufferPlans(std::vector<std::size_t>{0, 65}, 64).empty());
}
TEST(BufferPlanBudget, FitsExactlyAndPreservesNetOrder)
{
    EXPECT_EQ(ecc::selectWholeBufferPlans(std::vector<std::size_t>{32, 32, 1}, 64),
              (std::vector<std::size_t>{0, 1}));
}

TEST(RepairConfig, OptionalInsertionLimitsRoundTripAcrossEveryAlgorithm) {
  ecc::RepairConfig config;
  EXPECT_FALSE(config.drv.max_inserted_buffers);
  EXPECT_FALSE(config.setup.max_inserted_buffers);
  EXPECT_FALSE(config.path.max_inserted_buffers);
  EXPECT_FALSE(config.vg.max_buffers);
  EXPECT_FALSE(config.gobi.max_buffers);
  for (const auto* key : {"drv.max_inserted_buffers", "setup.max_inserted_buffers",
                          "path.max_inserted_buffers", "vg.max_buffers", "gobi.max_buffers"}) {
    for (const auto* value : {"0", "3", "unlimited"}) {
      config.set(key, value);
      std::ostringstream output;
      config.write(output);
      EXPECT_NE(output.str().find(std::string(key) + "=" + value + "\n"), std::string::npos);
      ecc::RepairConfig restored;
      std::istringstream input(output.str());
      restored.read(input);
      std::ostringstream again;
      restored.write(again);
      EXPECT_EQ(output.str(), again.str());
    }
    for (const auto* value : {"-1", "none", "1.5", "4294967296"})
      EXPECT_THROW(config.set(key, value), std::invalid_argument);
  }
}

TEST(RepairConfig, ActionsAreIndependentOfInsertionLimits) {
  ecc::RepairConfig config;
  config.set("common.actions", "resize,load_downsize");
  config.set("setup.max_inserted_buffers", "unlimited");
  EXPECT_FALSE(config.setupOptions().common.actions.buffer);
  EXPECT_TRUE(config.setupOptions().common.actions.resize);
  config.set("common.actions", "buffer");
  EXPECT_TRUE(config.pathOptions().common.actions.buffer);
  EXPECT_FALSE(config.pathOptions().common.actions.resize);
  EXPECT_FALSE(config.pathOptions().common.actions.load_downsize);
  config.set("setup.max_inserted_buffers", "0");
  EXPECT_TRUE(config.setupOptions().common.actions.buffer);
  EXPECT_EQ(config.setupOptions().max_inserted_buffers, 0u);
  config.set("common.actions", "none");
  EXPECT_EQ(config.common.actions, (ecc::RepairActions{false, false, false}));
  for (const auto* value : {"resize,resize", "buffer,", "", "bogus", "buffer,,resize"})
    EXPECT_THROW(config.set("common.actions", value), std::invalid_argument);
}

TEST(BufferPlanBudget, ZeroAllowsNoInsertionAndExhaustionCannotUnderflow) {
  EXPECT_EQ(ecc::remainingBufferBudget(std::nullopt, 100),
            std::numeric_limits<unsigned>::max());
  EXPECT_EQ(ecc::remainingBufferBudget(0, 0), 0u);
  EXPECT_EQ(ecc::remainingBufferBudget(3, 2), 1u);
  EXPECT_EQ(ecc::remainingBufferBudget(3, 4), 0u);
  const std::vector<std::size_t> counts{0, 1, 2};
  EXPECT_TRUE(ecc::selectWholeBufferPlans(counts, 0).empty());
}

TEST(RepairConfig, CommonPolicyPropagatesWithoutCouplingStageSettings) {
  ecc::RepairConfig config;
  config.set("common.max_fanout", "7");
  config.set("common.actions", "resize");
  config.set("drv.max_passes", "2");
  config.set("setup.max_passes", "5");
  const auto text = serialized(config);
  const auto drv = config.drvOptions();
  const auto setup = config.setupOptions();
  const auto path = config.pathOptions();
  const auto power = config.powerOptions();
  const auto vg = config.vgOptions();
  const auto gobi = config.gobiOptions();
  for (const auto *common : {&drv.common, &setup.common, &path.common, &power.common,
                             &vg.common, &gobi.common}) {
    EXPECT_EQ(common->max_fanout, 7u);
    EXPECT_EQ(common->actions, (ecc::RepairActions{true, false, false}));
  }
  EXPECT_EQ(drv.max_passes, 2u);
  EXPECT_EQ(setup.max_passes, 5u);
  auto trial = config.setupOptions();
  trial.common.actions.buffer = true;
  trial.max_passes = 1;
  EXPECT_EQ(serialized(config), text);
  config.set("common.max_fanout", "9");
  EXPECT_EQ(config.drvOptions().common.max_fanout, 9u);
  EXPECT_EQ(config.setupOptions().common.max_fanout, 9u);
  EXPECT_EQ(config.setupOptions().max_passes, 5u);
}
