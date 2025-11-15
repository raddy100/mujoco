#include "ProGuiChain.h"
#include "ProGuiSim.h"
#include <gtest/gtest.h>

TEST(DirectionHistoryTokens, TurnClassification) {
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

  // Initial face PosX -> change to PosY should record "left"
  ASSERT_TRUE(chain.directionHistory().empty());
  chain.setSpawnFace(SpawnFace::PosY);
  ASSERT_FALSE(chain.directionHistory().empty());
  EXPECT_EQ(chain.directionHistory().back(), "left");

  // PosY -> PosX should record "right"
  chain.setSpawnFace(SpawnFace::PosX);
  EXPECT_EQ(chain.directionHistory().back(), "right");

  // PosX -> PosZ should record "up"
  chain.setSpawnFace(SpawnFace::PosZ);
  EXPECT_EQ(chain.directionHistory().back(), "up");

  // PosZ -> PosX should record "Turnfront"
  chain.setSpawnFace(SpawnFace::PosX);
  EXPECT_EQ(chain.directionHistory().back(), "Turnfront");

  // PosX -> NegY (right turn sequence) yields "right"
  chain.setSpawnFace(SpawnFace::NegY);
  EXPECT_EQ(chain.directionHistory().back(), "right");
}

TEST(DirectionHistorySuppression, SuppressDuringReplay) {
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

  chain.setHistorySuppressed(true);
  chain.setSpawnFace(SpawnFace::PosY); // would normally record
  EXPECT_TRUE(chain.directionHistory().empty());

  chain.setHistorySuppressed(false);
  chain.setSpawnFace(SpawnFace::PosZ);
  EXPECT_FALSE(chain.directionHistory().empty());
  EXPECT_EQ(chain.directionHistory().back(), "up");
}