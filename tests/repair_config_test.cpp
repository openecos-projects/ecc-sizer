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
TEST(RepairConfig, SharesCommonConstraintsAndRuntimeHooks)
{
  ecc::RepairConfig config;
  config.set("common.max_fanout", "12");
  config.set("common.allow_gate_resize", "false");
  config.set("setup.max_trials", "3");
  config.set("setup.skip_duplicate_trials", "false");
  config.set("setup.defer_rollback_refresh", "false");
  config.set("common.allow_load_downsize", "false");
  config.set("common.load_downsize_slack_margin_ns", "0.01");
  config.set("design.max_load_downsizes_per_pass", "3");
  config.set("common.vt_suffixes", "_R,_L,_SL");
  config.set("power.enabled", "true");
  config.set("power.objective", "total");
  config.set("power.slack_margin_ns", "0.02");
  config.set("gbi.max_buffers", "7");
  config.set("gbi.backend", "cuda");
  config.set("gbi.cuda_max_memory_mb", "256");
  config.set("gbi.max_group_loads", "4");
  config.set("gbi.min_gain", "0.25");
  config.set("gbi.require_drv_reduction", "false");
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
  EXPECT_FALSE(setup.common.allow_gate_resize);
  EXPECT_FALSE(setup.common.allow_load_downsize);
  EXPECT_FLOAT_EQ(setup.common.load_downsize_slack_margin, 1e-11f);
  EXPECT_EQ(config.design.max_load_downsizes_per_pass, 3);
  EXPECT_EQ(setup.max_trials, 3);
  EXPECT_FALSE(setup.skip_duplicate_trials);
  EXPECT_FALSE(setup.defer_rollback_refresh);
  EXPECT_EQ(setup.max_passes, ecc::SetupRepairSettings{}.max_passes);
  EXPECT_EQ(config.gbi.max_buffers, 7u);
  EXPECT_EQ(config.gbi.backend, "cuda");
  EXPECT_EQ(config.gbi.cuda_max_memory_mb, 256u);
  EXPECT_EQ(config.gbi.max_group_loads, 4u);
  EXPECT_FLOAT_EQ(config.gbi.min_gain, 0.25f);
  EXPECT_FALSE(config.gbi.require_drv_reduction);
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
  config.set("gbi.max_buffers", "3");
  config.set("gbi.max_group_loads", "2");
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
         {"gbi.max_group_loads", "1"},
         {"gbi.backend", "opencl"},
         {"gbi.cuda_max_memory_mb", "0"},
         {"gbi.min_gain", "-1"},
         {"gbi.require_drv_reduction", "maybe"},
         {"setup.batch_size", "0"},
         {"common.recovered_slew_fraction", "0"},
         {"common.overload_group_fraction", "1"},
         {"setup.min_improvement_ns", "-1"},
         {"runtime.sta_threads", "0"},
         {"tolerances.tns_relative", "1"},
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
