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
TEST(RepairConfig, PathUnlimitedBudgetsPreserveZeroAreaConstraint)
{
  ecc::RepairConfig config;
  config.set("setup.path_buffer_max_trials", "0");
  config.set("setup.path_buffer_max_area_growth", "-1");
  EXPECT_NE(serialized(config).find("setup.path_buffer_max_trials=0\n"),
            std::string::npos);
  EXPECT_NE(serialized(config).find("setup.path_buffer_max_area_growth=-1\n"),
            std::string::npos);
  EXPECT_THROW(config.set("setup.path_buffer_max_area_growth", "-0.5"),
               std::invalid_argument);
  EXPECT_NO_THROW(config.set("setup.path_buffer_max_area_growth", "0"));
  EXPECT_NO_THROW(config.set("setup.path_buffer_strategy", "feedback"));
  config.set("setup.path_buffer_min_relative_improvement", "0.001");
  EXPECT_NE(serialized(config).find("setup.path_buffer_min_relative_improvement="),
            std::string::npos);
  for(const auto* value : {"-0.1", "1.1", "nan", "inf"})
    EXPECT_THROW(config.set("setup.path_buffer_min_relative_improvement", value),
                 std::invalid_argument);
}

TEST(RepairConfig, PathPlacementModeRoundTripsAndRejectsUnknownModes)
{
  ecc::RepairConfig config;
  EXPECT_EQ(config.setupOptions().path_buffer_placement, "trial");
  config.set("setup.path_buffer_placement", "final");
  EXPECT_EQ(config.setupOptions().path_buffer_placement, "final");
  EXPECT_NE(serialized(config).find("setup.path_buffer_placement=final\n"), std::string::npos);
  EXPECT_THROW(config.set("setup.path_buffer_placement", "off"), std::invalid_argument);
}

TEST(RepairConfig, PathGainScopesRoundTrip)
{
  for(const auto* scope : {"global", "current_global", "endpoints"}) {
    ecc::RepairConfig config;
    config.set("setup.path_buffer_gain_scope", scope);
    EXPECT_EQ(config.setupOptions().path_buffer_gain_scope, scope);
    EXPECT_NE(serialized(config).find(std::string("setup.path_buffer_gain_scope=") + scope + "\n"),
              std::string::npos);
  }
  ecc::RepairConfig config;
  EXPECT_THROW(config.set("setup.path_buffer_gain_scope", "unknown"), std::invalid_argument);
}

TEST(RepairConfig, PathRejectionStopIsOptionalAndRoundTrips)
{
  ecc::RepairConfig config;
  EXPECT_EQ(config.setupOptions().path_buffer_max_consecutive_rejections, 0u);
  config.set("setup.path_buffer_max_consecutive_rejections", "32");
  EXPECT_EQ(config.setupOptions().path_buffer_max_consecutive_rejections, 32u);
  EXPECT_NE(serialized(config).find("setup.path_buffer_max_consecutive_rejections=32\n"), std::string::npos);
  for(const auto* value : {"-1", "1.5", "nan", "2147483648"})
    EXPECT_THROW(config.set("setup.path_buffer_max_consecutive_rejections", value), std::invalid_argument);
  config.set("setup.path_buffer_max_consecutive_rejections", "0");
  EXPECT_EQ(config.setupOptions().path_buffer_max_consecutive_rejections, 0u);
}

TEST(RepairConfig, PathVerificationModeRoundTrips)
{
  ecc::RepairConfig config;
  EXPECT_EQ(config.setupOptions().path_buffer_verification, "trial");
  config.set("setup.path_buffer_verification", "final");
  EXPECT_EQ(config.setupOptions().path_buffer_verification, "final");
  EXPECT_NE(serialized(config).find("setup.path_buffer_verification=final\n"), std::string::npos);
  EXPECT_THROW(config.set("setup.path_buffer_verification", "off"), std::invalid_argument);
}

TEST(RepairConfig, PathFeedbackControlsRoundTripWithoutChangingAcceptance)
{
  ecc::RepairConfig config;
  const auto before = config.setupOptions();
  for(const auto* key : {"setup.path_buffer_failure_cache", "setup.path_buffer_calibrate",
                         "setup.path_buffer_diversify"}) {
    config.set(key, "false");
    EXPECT_NE(serialized(config).find(std::string(key) + "=false\n"), std::string::npos);
    EXPECT_THROW(config.set(key, "invalid"), std::invalid_argument);
  }
  const auto options = config.setupOptions();
  EXPECT_FALSE(options.path_buffer_failure_cache);
  EXPECT_FALSE(options.path_buffer_calibrate);
  EXPECT_FALSE(options.path_buffer_diversify);
  EXPECT_FALSE(options.path_buffer_adaptive_search);
  EXPECT_FALSE(options.path_buffer_endpoint_coverage);
  EXPECT_FALSE(options.path_buffer_group_schedule);
  EXPECT_TRUE(options.path_buffer_timing_precheck);
  for(const auto* key : {"setup.path_buffer_adaptive_search", "setup.path_buffer_timing_precheck",
                         "setup.path_buffer_endpoint_coverage", "setup.path_buffer_group_schedule"}) {
    config.set(key, "true");
    EXPECT_NE(serialized(config).find(std::string(key) + "=true\n"), std::string::npos);
    config.set(key, "false");
    EXPECT_NE(serialized(config).find(std::string(key) + "=false\n"), std::string::npos);
    EXPECT_THROW(config.set(key, "invalid"), std::invalid_argument);
  }
  EXPECT_EQ(options.min_improvement, before.min_improvement);
  EXPECT_EQ(options.path_buffer_min_relative_improvement, before.path_buffer_min_relative_improvement);
  config.set("setup.path_buffer_incremental_analysis", "true");
  EXPECT_TRUE(config.setupOptions().path_buffer_incremental_analysis);
  EXPECT_NE(serialized(config).find("setup.path_buffer_incremental_analysis=true\n"), std::string::npos);
  EXPECT_THROW(config.set("setup.path_buffer_incremental_analysis", "invalid"), std::invalid_argument);
  config.set("setup.path_buffer_reuse_rejections", "true");
  EXPECT_TRUE(config.setupOptions().path_buffer_reuse_rejections);
  EXPECT_NE(serialized(config).find("setup.path_buffer_reuse_rejections=true\n"), std::string::npos);
  EXPECT_THROW(config.set("setup.path_buffer_reuse_rejections", "invalid"), std::invalid_argument);
  config.set("setup.path_buffer_incremental_parasitics", "true");
  EXPECT_TRUE(config.setupOptions().path_buffer_incremental_parasitics);
  EXPECT_NE(serialized(config).find("setup.path_buffer_incremental_parasitics=true\n"), std::string::npos);
  EXPECT_THROW(config.set("setup.path_buffer_incremental_parasitics", "invalid"), std::invalid_argument);
}

TEST(RepairConfig, SharesCommonConstraintsAndRuntimeHooks)
{
  ecc::RepairConfig config;
  config.set("common.max_fanout", "12");
  config.set("common.allow_gate_resize", "false");
  config.set("setup.max_trials", "3");
  config.set("setup.skip_duplicate_trials", "false");
  config.set("setup.defer_rollback_refresh", "false");
  config.set("setup.path_buffer_enabled", "true");
  config.set("setup.path_buffer_only", "true");
  config.set("setup.path_buffer_max_paths", "5");
  config.set("setup.path_buffer_max_candidates", "3");
  config.set("setup.path_buffer_max_trials", "4");
  config.set("setup.path_buffer_batch_size", "3");
  config.set("setup.path_buffer_strategy", "sequential");
  config.set("setup.path_buffer_fanout_repair", "false");
  config.set("setup.path_buffer_recovery", "false");
  config.set("setup.path_buffer_tree_candidates", "false");
  config.set("setup.path_buffer_gain_scope", "global");
  EXPECT_THROW(config.set("setup.path_buffer_gain_scope", "unknown"), std::invalid_argument);
  config.set("setup.path_buffer_max_seconds", "1.5");
  config.set("setup.path_buffer_max_refreshes", "16");
  config.set("setup.path_buffer_max_inserted_buffers", "6");
  config.set("setup.path_buffer_max_area_growth", "0.1");
  config.set("common.allow_load_downsize", "false");
  config.set("common.load_downsize_slack_margin_ns", "0.01");
  config.set("design.max_load_downsizes_per_pass", "3");
  config.set("common.vt_suffixes", "_R,_L,_SL");
  config.set("power.enabled", "true");
  config.set("power.objective", "total");
  config.set("power.slack_margin_ns", "0.02");
  config.set("vg.max_buffers", "7");
  config.set("vg.backend", "cuda");
  config.set("vg.cuda_max_memory_mb", "256");
  config.set("vg.max_group_loads", "4");
  config.set("vg.min_gain", "0.25");
  config.set("vg.require_drv_reduction", "false");
  config.set("gbi.max_buffers", "9");
  config.set("gbi.max_nets", "3");
  auto power = config.powerOptions();
  EXPECT_TRUE(power.enabled);
  EXPECT_EQ(power.objective, "total");
  EXPECT_FLOAT_EQ(power.slack_margin, 2e-11f);
  EXPECT_EQ(power.common.vt_suffixes.size(), 3);
  config.design.max_passes = 2;
  unsigned prepares = 0;
  config.design.prepare = [&]() { ++prepares; };
  auto setup = config.setupOptions();
  EXPECT_EQ(setup.common.max_fanout, 12);
  EXPECT_FALSE(setup.path_buffer_fanout_repair);
  EXPECT_FALSE(setup.path_buffer_recovery);
  EXPECT_FALSE(setup.path_buffer_tree_candidates);
  EXPECT_EQ(setup.path_buffer_gain_scope, "global");
  EXPECT_FALSE(setup.common.allow_gate_resize);
  EXPECT_FALSE(setup.common.allow_load_downsize);
  EXPECT_FLOAT_EQ(setup.common.load_downsize_slack_margin, 1e-11f);
  EXPECT_EQ(config.design.max_load_downsizes_per_pass, 3);
  EXPECT_EQ(setup.max_trials, 3);
  EXPECT_FALSE(setup.skip_duplicate_trials);
  EXPECT_FALSE(setup.defer_rollback_refresh);
  EXPECT_TRUE(setup.path_buffer_enabled);
  EXPECT_TRUE(setup.path_buffer_only);
  EXPECT_EQ(setup.path_buffer_max_paths, 5u);
  EXPECT_EQ(setup.path_buffer_max_candidates, 3u);
  EXPECT_EQ(setup.path_buffer_max_trials, 4u);
  EXPECT_EQ(setup.path_buffer_batch_size, 3u);
  EXPECT_EQ(setup.path_buffer_strategy, "sequential");
  EXPECT_DOUBLE_EQ(setup.path_buffer_max_seconds, 1.5);
  EXPECT_EQ(setup.path_buffer_max_refreshes, 16u);
  EXPECT_EQ(setup.path_buffer_max_inserted_buffers, 6u);
  EXPECT_DOUBLE_EQ(setup.path_buffer_max_area_growth, 0.1);
  EXPECT_EQ(setup.max_passes, ecc::SetupRepairSettings{}.max_passes);
  EXPECT_EQ(config.vg.max_buffers, 7u);
  EXPECT_EQ(config.vg.backend, "cuda");
  EXPECT_EQ(config.vg.cuda_max_memory_mb, 256u);
  EXPECT_EQ(config.vg.max_group_loads, 4u);
  EXPECT_FLOAT_EQ(config.vg.min_gain, 0.25f);
  EXPECT_FALSE(config.vg.require_drv_reduction);
  EXPECT_EQ(config.gbi.max_buffers, 9u);
  EXPECT_EQ(config.gbi.max_nets, 3u);
  setup.common.prepare();
  EXPECT_EQ(prepares, 1);
}
TEST(RepairConfig, SectionsUnitsAndRoundTrip)
{
  ecc::RepairConfig config;
  std::istringstream input(
      "# comment\n[setup]\nmax_passes = 3\nmin_improvement_ns = 0.002\n"
      "[runtime]\nsta_threads = 2\n[common]\nmargin=0.15 # margin\n"
      "vt_suffixes=_R,_L,_SL\n[power]\nenabled=true\nobjective=total\n");
  config.read(input);
  EXPECT_EQ(config.setup.max_passes, 3);
  EXPECT_FLOAT_EQ(config.setup.min_improvement, 2e-12f);
  EXPECT_EQ(config.sta_threads, 2);
  EXPECT_TRUE(config.power.enabled);
  config.set("vg.max_buffers", "3");
  config.set("vg.max_group_loads", "2");
  EXPECT_EQ(config.power.objective, "total");
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
         {"setup.path_buffer_strategy", "invalid"},
         {"setup.path_buffer_max_seconds", "nan"},
         {"setup.path_buffer_max_seconds", "-1"},
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
TEST(RepairConfig, ExplicitOverridesWinOverFilesAndAliasesRemainSupported)
{
  const auto dir = std::filesystem::temp_directory_path()
                   / ("ecc-repair-config-" + std::to_string(getpid()));
  std::filesystem::create_directory(dir);
  struct Cleanup
  {
    std::filesystem::path dir;
    ~Cleanup() { std::filesystem::remove_all(dir); }
  } cleanup{dir};
  auto ini = dir / "settings with spaces.ini";
  auto cmd = dir / "run.cmd";
  {
    std::ofstream out(ini);
    out << "setup.max_trials=9\nsetup.max_passes=2\n";
  }
  {
    std::ofstream out(cmd);
    out << "-sizer_setup_trials 3\n-sizer_repair_config \"" << ini.string()
        << "\" # load\n-sizer_repair_set setup.batch_size=1\n";
  }
  ecc::RepairConfig config;
  ecc::loadRepairConfigFiles(config, cmd.string());
  std::ifstream commands(cmd);
  std::string line;
  while (std::getline(commands, line))
    ecc::applyRepairConfigOverride(config, line);
  EXPECT_EQ(config.setup.max_trials, 3);
  EXPECT_EQ(config.setup.max_passes, 2);
  EXPECT_EQ(config.setup.batch_size, 1);
  EXPECT_TRUE(
      ecc::applyRepairConfigOverride(config, "-sizer_repair_no_gate_resize"));
  EXPECT_FALSE(config.design.allow_gate_resize);
  EXPECT_THROW(
      ecc::applyRepairConfigOverride(config, "-sizer_setup_trials 2 extra"),
      std::exception);
  EXPECT_THROW(
      ecc::applyRepairConfigOverride(config, "-sizer_repair_set setup.typo=3"),
      std::exception);
  EXPECT_THROW(ecc::applyRepairConfigOverride(config, "-sizer_repair_set"),
               std::exception);
}
}  // namespace

TEST(PbbiConfig, RoundtripBudgetsAndIndependentSwitches)
{
  ecc::RepairConfig c;
  c.set("pbbi.enabled","true");c.set("pbbi.max_frontier_states","0");
  c.set("pbbi.scheduler","global");c.set("pbbi.scheduler_window","3");
  c.set("pbbi.forecast_cache","false");c.set("pbbi.path_cache","false");
  for(const auto* k:{"pbbi.pin_diagnostics","pbbi.candidate_electrical_check","pbbi.operating_point_pruning","pbbi.failure_feedback"})c.set(k,"false");
  c.set("pbbi.buffer_aware_timing","false");c.set("pbbi.slew_propagation","false");
  c.set("pbbi.slew_cache_entries","7");
  c.set("pbbi.slew_samples","24");c.set("pbbi.timing_max_frontier_states","32");
  c.set("pbbi.timing_max_expansions","12345");
  c.set("pbbi.max_expansions","0");c.set("pbbi.max_seconds","1.12345678912345");
  c.set("pbbi.gate_sizing","false");c.set("pbbi.sink_sizing","false");
  c.set("pbbi.min_improvement_ns","0.000123456789");
  std::ostringstream out;c.write(out);std::istringstream in(out.str());
  ecc::RepairConfig restored;restored.read(in);
  EXPECT_EQ(restored.pbbi.scheduler,"global");EXPECT_EQ(restored.pbbi.scheduler_window,3u);
  EXPECT_FALSE(restored.pbbi.forecast_cache);EXPECT_FALSE(restored.pbbi.path_cache);
  EXPECT_THROW(c.set("pbbi.scheduler","unknown"),std::invalid_argument);
  c.set("pbbi.scheduler","adaptive");
  EXPECT_EQ(c.pbbi.scheduler,"adaptive");
  EXPECT_THROW(c.set("pbbi.scheduler_window","0"),std::invalid_argument);
  EXPECT_FALSE(restored.pbbi.pin_diagnostics);EXPECT_FALSE(restored.pbbi.candidate_electrical_check);
  EXPECT_FALSE(restored.pbbi.operating_point_pruning);EXPECT_FALSE(restored.pbbi.failure_feedback);
  EXPECT_FALSE(restored.pbbi.buffer_aware_timing);EXPECT_FALSE(restored.pbbi.slew_propagation);
  EXPECT_EQ(restored.pbbi.slew_cache_entries,7u);
  EXPECT_EQ(restored.pbbi.slew_samples,24u);EXPECT_EQ(restored.pbbi.timing_max_frontier_states,32u);
  EXPECT_EQ(restored.pbbi.timing_max_expansions,12345u);
  EXPECT_THROW(c.set("pbbi.slew_samples","1"),std::invalid_argument);
  EXPECT_EQ(restored.pbbi.max_seconds,c.pbbi.max_seconds);
  EXPECT_EQ(restored.pbbi.min_improvement,c.pbbi.min_improvement);
  EXPECT_TRUE(restored.pbbi.enabled);EXPECT_FALSE(restored.pbbiOptions().gate_sizing);
  EXPECT_THROW(c.set("pbbi.max_paths","0"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.max_seconds","-1"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.max_area_growth","nan"),std::invalid_argument);
}

TEST(RepairConfig, VgSetupScopeRoundTripsAndRejectsUnknownScope) {
  ecc::RepairConfig c;
  EXPECT_EQ(c.vg.scope,"drv");
  c.set("vg.scope","setup");
  std::ostringstream out;c.write(out);std::istringstream in(out.str());
  ecc::RepairConfig restored;restored.read(in);
  EXPECT_EQ(restored.vg.scope,"setup");
  EXPECT_THROW(c.set("vg.scope","invalid"),std::invalid_argument);
}

TEST(RepairConfig, PbbiPlacementModeRoundTrip) {
  ecc::RepairConfig c;
  EXPECT_FALSE(c.pbbi.incremental_placement);
  c.set("pbbi.incremental_placement","true");
  std::ostringstream out;c.write(out);std::istringstream in(out.str());
  ecc::RepairConfig restored;restored.read(in);
  EXPECT_TRUE(restored.pbbi.incremental_placement);
}

TEST(RepairConfig, PbbiPlacementFeedbackRoundTrip) {
  ecc::RepairConfig c;c.set("pbbi.placement_feedback","true");
  std::ostringstream out;c.write(out);std::istringstream in(out.str());
  ecc::RepairConfig restored;restored.read(in);
  EXPECT_TRUE(restored.pbbi.placement_feedback);
}

TEST(RepairConfig, PbbiNetScopeRoundTrip) {
  ecc::RepairConfig c;
  EXPECT_EQ(c.pbbi.scope,"path");
  c.set("pbbi.scope","net");
  std::ostringstream out;c.write(out);std::istringstream stream(out.str());
  ecc::RepairConfig restored;restored.read(stream);
  EXPECT_EQ(restored.pbbiOptions().scope,"net");
  EXPECT_THROW(c.set("pbbi.scope","vg"),std::invalid_argument);
  EXPECT_EQ(c.pbbi.scope,"net");
}

TEST(RepairConfig, PbbiAreaTargetsRoundTripAndValidate) {
  ecc::RepairConfig c;
  EXPECT_EQ(c.pbbi.objective,"timing");EXPECT_FALSE(c.pbbi.target_tns);
  c.set("pbbi.objective","area");c.set("pbbi.target_wns_ns","-0.123456789123");
  c.set("pbbi.target_tns_ns","-654.5023055447093");
  std::ostringstream out;c.write(out);std::istringstream in(out.str());
  ecc::RepairConfig restored;restored.read(in);
  EXPECT_EQ(restored.pbbi.objective,"area");
  EXPECT_DOUBLE_EQ(restored.pbbi.target_wns,c.pbbi.target_wns);
  EXPECT_EQ(restored.pbbi.target_tns,c.pbbi.target_tns);
  EXPECT_THROW(c.set("pbbi.objective","power"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.target_wns_ns","nan"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.target_tns_ns","inf"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.target_wns_ns","0.1"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.target_tns_ns","0.1"),std::invalid_argument);
  EXPECT_THROW(c.set("pbbi.candidate_electrical_check","false"),std::invalid_argument);
  c.set("pbbi.target_tns_ns","none");EXPECT_FALSE(c.pbbiOptions().target_tns);
}
