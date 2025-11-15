#include "ProGuiSim.h"
#include <gtest/gtest.h>

TEST(GapLayoutTest, GapCallbackInvocationClamp) {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  mjSpec* spec = nullptr;
  mjvScene scene{};
  mjrContext context{};
  mjUI ui{};
  mjuiState uiState{};
  mjvCamera camera{};
  mjvOption option{};

  ProGuiSim sim(model, data, spec, &scene, &context, &ui, &uiState, &camera, &option);

  // Track callback invocations.
  double lastRatio = -1.0;
  int callCount = 0;
  sim.setApplyGapLayoutCallback([&](double r){
    lastRatio = r;
    ++callCount;
  });

  // Increase several times, expect clamp at 1.0.
  for (int i = 0; i < 30; ++i) sim.increaseGap();
  EXPECT_NEAR(sim.gapRatio(), 1.0, 1e-9);
  EXPECT_GE(callCount, 1);
  EXPECT_NEAR(lastRatio, 1.0, 1e-9);

  // Decrease several times, expect clamp at 0.0.
  for (int i = 0; i < 40; ++i) sim.decreaseGap();
  EXPECT_NEAR(sim.gapRatio(), 0.0, 1e-9);
  EXPECT_GE(callCount, 2);
  EXPECT_NEAR(lastRatio, 0.0, 1e-9);
}