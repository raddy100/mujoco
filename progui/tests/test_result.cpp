#include "Result.h"
#include "ProGuiSim.h"
#include "ProGuiChain.h"
#include "ProGuiUI.h"
#include <gtest/gtest.h>

// Minimal scaffolding: instantiate objects with null MuJoCo pointers (no validate)
// so we can test pure logic (gap ratio adjustments, Result factories, direction history)
// without requiring a loaded model.

TEST(ResultTest, FactoryHelpers) {
  Result ok = Result::success();
  EXPECT_TRUE(ok.ok());
  EXPECT_EQ(ok.status, Status::Ok);
  EXPECT_TRUE(ok.message.empty());

  Result nr = Result::notReady("deps missing");
  EXPECT_FALSE(nr.ok());
  EXPECT_EQ(nr.status, Status::NotReady);
  EXPECT_EQ(nr.message, "deps missing");

  Result io = Result::ioError("file");
  EXPECT_EQ(io.status, Status::IOError);
  EXPECT_EQ(std::string(toString(io.status)), "IOError");

  Result ia = Result::invalidArg("bad");
  EXPECT_EQ(ia.status, Status::InvalidArgument);
  EXPECT_EQ(ia.message, "bad");

  Result rc = Result::recompileFailed("fail");
  EXPECT_EQ(rc.status, Status::RecompileFailed);

  Result ie = Result::internalError("oops");
  EXPECT_EQ(ie.status, Status::InternalError);
}

TEST(SimTest, GapClampIncreaseDecrease) {
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
  double initial = sim.gapRatio();
  EXPECT_NEAR(initial, 0.05, 1e-9);

  // Increase until clamp
  for (int i = 0; i < 100; ++i) sim.increaseGap();
  EXPECT_LE(sim.gapRatio(), 1.0);
  EXPECT_NEAR(sim.gapRatio(), 1.0, 1e-9);

  // Decrease until clamp
  for (int i = 0; i < 200; ++i) sim.decreaseGap();
  EXPECT_GE(sim.gapRatio(), 0.0);
  EXPECT_NEAR(sim.gapRatio(), 0.0, 1e-9);
}

TEST(DirectionTest, RecordTurnForwardHistory) {
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
  ProGuiChain chain(spec, model, data, &sim);

  // Setting spawn face should not record until a change occurs.
  EXPECT_TRUE(chain.directionHistory().empty());
  chain.setSpawnFace(SpawnFace::PosY);
  EXPECT_FALSE(chain.directionHistory().empty()); // turn recorded (PosX default -> PosY)
  size_t afterFirstTurn = chain.directionHistory().size();
  chain.setSpawnFace(SpawnFace::PosZ); // up
  EXPECT_GT(chain.directionHistory().size(), afterFirstTurn);

  // Simulate forward spawn attempts (will fail NotReady but still can add forward only after lastBody exists).
  // Since model/spec are null, spawnCube will return NotReady; forward should not be recorded.
  auto res = chain.spawnCube();
  EXPECT_EQ(res.status, Status::NotReady);

  // Ensure history still only has turn tokens, not "forward".
  for (auto const& tok : chain.directionHistory()) {
    EXPECT_NE(tok, "forward");
  }
}

// main provided by GTest