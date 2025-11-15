#pragma once
/**
 * @file ProGuiChain.h
 * @brief Encapsulates cube chain construction, modification, loop creation, persistence,
 *        and direction history tracking for the refactored ProGUI.
 *
 * This class migrates the responsibilities of legacy progui_chain.cpp and associated
 * globals into a cohesive C++17 object. It operates on non-owning pointers to MuJoCo
 * mjSpec / mjModel / mjData (owned elsewhere, typically managed by application setup)
 * and collaborates with ProGuiSim for physics/render related operations.
 *
 * Invariants:
 *  - All non-owning MuJoCo pointers (spec_, model_, data_, sim_) must be valid for
 *    mutating operations returning Result::Ok.
 *  - chain_[0] (if present) represents the root cube body.
 *  - nextCubeId_ tracks the maximum parsed cube ID encountered to avoid duplicate names.
 *
 * Thread-safety: NOT thread-safe. External synchronization required for concurrent usage.
 *
 * Error handling: Methods that can fail return Result. Query methods return simple
 * types or std::optional<>. Exceptions reserved only for construction-time validation.
 */

#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>

#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <optional>
#include <filesystem>
#include <cassert>

#include "Result.h"
#include "ProGuiSim.h" // SpawnFace + constants

/**
 * @class ProGuiChain
 * @brief Manages creation and manipulation of a linear (with optional loop equality)
 *        chain of cube bodies, including direction history logging and layout updates
 *        in response to gap changes.
 */
class ProGuiChain {
public:
  /**
   * @brief Represents a spawned cube's metadata.
   *
   * Fields:
   *  - specBody: pointer to associated mjsBody (non-owning; owned by spec_)
   *  - name: unique cube name "cube_<id>"
   *  - bodyId: runtime MuJoCo body id (updated via refreshBodyIdIndexMap)
   *  - axis/sign: relative placement axis and direction (+/-) toward parent
   *  - distanceFactor: 1 normal spacing, 2 for loop spacing (double distance)
   */
  struct ChainEntry {
    mjsBody*     specBody{nullptr};
    std::string  name{};
    int          bodyId{-1};
    int          axis{-1};
    int          sign{+1};
    int          distanceFactor{1};
  };

  /**
   * @brief Saved pre-physics local transform (positions & quaternion).
   */
  struct SavedBodyPos {
    std::string name;
    double pos[3]{0,0,0};
    double quat[4]{1,0,0,0};
  };

  /**
   * @brief Construct chain manager (no validation).
   * @param spec  MuJoCo specification pointer
   * @param model MuJoCo model pointer
   * @param data  MuJoCo data pointer
   * @param sim   Reference simulation manager
   */
  ProGuiChain(mjSpec* spec,
              mjModel* model,
              mjData* data,
              ProGuiSim* sim) noexcept;

  /**
   * @brief Construct with optional validation; throws std::invalid_argument on null pointers.
   * @param validate When true, verifies all mandatory pointers are non-null.
   */
  ProGuiChain(mjSpec* spec,
              mjModel* model,
              mjData* data,
              ProGuiSim* sim,
              bool validate);

  ProGuiChain(const ProGuiChain&) = delete;
  ProGuiChain& operator=(const ProGuiChain&) = delete;
  ProGuiChain(ProGuiChain&&) = default;
  ProGuiChain& operator=(ProGuiChain&&) = default;
  ~ProGuiChain() = default;

  // ---------------------------------------------------------------------------
  // Chain Operations
  // ---------------------------------------------------------------------------

  /**
   * @brief Spawn a new cube using the current spawn face (equivalent to legacy "forward").
   *
   * Behavior:
   *  - If this is the first cube (root), a free joint is created.
   *  - Otherwise a ball joint is added unless obstructed loop contact triggers equality constraints.
   *  - Direction history "forward" token is recorded only for non-root spawns when recording is enabled.
   *
   * Failure Modes:
   *  - Status::NotReady if any of spec_, model_, or data_ are null.
   *  - Status::RecompileFailed if mj_recompile fails after modification.
   *  - Status::InternalError if MuJoCo spec element creation fails.
   *
   * Postconditions (on success):
   *  - lastBody_, lastMarker_, probeRect_ updated.
   *  - chain_ extended, bodyId refresh scheduled.
   *  - Rendering context rebuilt via ProGuiSim.
   *
   * @return Result status indicating success or failure.
   */
  Result spawnCube();

  /**
   * @brief Spawn cube driven by a direction token adjusting spawn face as needed.
   *
   * Supported tokens:
   *   "forward", "left", "right", "up", "down", "Turnfront", "TurnBack".
   * Unknown tokens are treated as "forward".
   *
   * Face Change:
   *  - When token implies a face rotation, recordTurn() is invoked before spawning.
   *
   * @param dirToken Direction token (case-sensitive).
   * @return Result propagated from underlying spawnCube logic.
   */
  Result spawnCubeFromInput(std::string_view dirToken);

  /**
   * @brief Delete the last (most recently spawned) cube; root cube cannot be removed.
   *
   * Direction History:
   *  - Prunes trailing "forward", loop, and turn tokens associated with the deleted cube.
   *
   * Failure Modes:
   *  - Status::NotReady if spec_/model_/data_ null.
   *  - Status::InvalidArgument if attempting to delete when chain size <= 1.
   *  - Status::RecompileFailed if mj_recompile fails.
   *
   * @return Structured Result of the delete operation.
   */
  Result deleteLastCube();

  /**
   * @brief Translate all cube positions along +X or -X with a progressive offset scaling by index.
   *
   * Algorithm:
   *  - Each body's displacement fraction f = (i+1)/N producing a smooth spread.
   *
   * @param dir Sign of movement (+ moves +X, - moves -X).
   * @return Result with NotReady/RecompileFailed/internal errors as appropriate.
   */
  Result moveLastCubeX(int dir);

  /**
   * @brief Translate all cube positions along +Y or -Y with progressive offset scaling.
   *
   * @param dir Sign of movement (+ moves +Y, - moves -Y).
   */
  Result moveLastCubeY(int dir);

  /**
   * @brief Translate all cube positions along +Z or -Z with progressive offset scaling.
   *
   * @param dir Sign of movement (+ moves +Z, - moves -Z).
   */
  Result moveLastCubeZ(int dir);

  /**
   * @brief Enable physics (gravity restoration) via ProGuiSim for a non-empty chain.
   *
   * @return Result::InvalidArgument if chain empty; other failures forwarded from ProGuiSim.
   */
  Result enablePhysics();

  /**
   * @brief Snapshot current local position & quaternion for each body prior to physics run.
   *
   * Postconditions:
   *  - hasSavedPrePhysicsState_ true if at least one body captured.
   *
   * @return Always Status::Ok (no failure modes currently).
   */
  Result savePrePhysicsState();

  /**
   * @brief Restore chain to previously saved pre-physics state (positions and quaternions).
   *
   * Behavior:
   *  - Adjusts chain size to match snapshot (spawns or deletes as necessary) with recording suppressed.
   *  - Disables gravity during reset, then reapplies build-mode options.
   *
   * Failure Modes:
   *  - Status::NotReady if spec_/model_/data_ null.
   *  - Status::InvalidArgument if no saved state exists.
   *  - Status::RecompileFailed if MuJoCo recompile fails.
   *
   * @return Result describing success or failure.
   */
  Result resetToSavedState();

  // ---------------------------------------------------------------------------
  // Persistence
  // ---------------------------------------------------------------------------

  /**
   * @brief Save direction history tokens to file.
   * @param file Path (defaults to directions.txt if empty).
   */
  Result saveDirections(const std::filesystem::path& file);

  /**
   * @brief Load chain from direction tokens file (rebuilding chain relative placements).
   * @param file Path to token file
   */
  Result loadDirections(const std::filesystem::path& file);

  // ---------------------------------------------------------------------------
  // Direction History
  // ---------------------------------------------------------------------------

  /**
   * @brief Record forward movement token (ignored if suppression active).
   */
  void recordForward();

  /**
   * @brief Record a turn between faces (left/right/up/down/Turnfront/TurnBack).
   * @param oldFace Prior spawn face
   * @param newFace New spawn face
   */
  void recordTurn(SpawnFace oldFace, SpawnFace newFace);

  /**
   * @brief Clear entire direction history.
   */
  void clearDirectionHistory() noexcept;

  /**
   * @return Immutable reference to direction tokens vector.
   */
  [[nodiscard]] const std::vector<std::string>& directionHistory() const noexcept { return directionHistory_; }

  /**
   * @brief Suppress recording (used during replay).
   * @param suppress True to hide new entries.
   */
  void setHistorySuppressed(bool suppress) noexcept { suppressDirRecord_ = suppress; }

  /**
   * @return Current spawn face.
   */
  [[nodiscard]] SpawnFace spawnFace() const noexcept { return spawnFace_; }

  /**
   * @brief Set new spawn face (records turn & updates marker).
   * @param newFace Target face
   */
  Result setSpawnFace(SpawnFace newFace);

  // ---------------------------------------------------------------------------
  // Layout / Gap
  // ---------------------------------------------------------------------------

  /**
   * @brief Apply layout update for given gap ratio (invoked via ProGuiSim callback).
   * @param gapRatio Ratio in [0,1]
   */
  void applyGapLayout(double gapRatio);

  // ---------------------------------------------------------------------------
  // Queries
  // ---------------------------------------------------------------------------

  [[nodiscard]] size_t chainSize() const noexcept { return chain_.size(); }
  [[nodiscard]] bool canSpawnCube() const noexcept;
  [[nodiscard]] bool canDeleteTarget() const noexcept;
  [[nodiscard]] const ChainEntry* lastEntry() const noexcept { return chain_.empty() ? nullptr : &chain_.back(); }
  [[nodiscard]] mjsBody* lastBody() const noexcept { return lastBody_; }
  [[nodiscard]] mjsGeom* lastMarker() const noexcept { return lastMarker_; }
  [[nodiscard]] mjsGeom* probeGeom()  const noexcept { return probeRect_; }
  [[nodiscard]] std::string_view probeName() const noexcept { return probeName_; }

  /**
   * @brief Refresh internal bodyId index map (calls ProGuiSim::updateBodyIdIndexMap).
   */
  Result refreshBodyIdIndexMap();

  /**
   * @brief Lookup chain index for MuJoCo body id.
   * @param bodyId MuJoCo body identifier
   * @return Optional index
   */
  std::optional<size_t> chainIndexForBodyId(int bodyId) const;

  // ---------------------------------------------------------------------------
  // Debug / Probe
  // ---------------------------------------------------------------------------

  /**
   * @brief Query body id currently hit by probe geometry.
   */
  std::optional<int> probeHitBodyId() const;

  // ---------------------------------------------------------------------------
  // UI Marker
  // ---------------------------------------------------------------------------

  /**
   * @brief Update spawn marker geometry on last body after spawn/delete/face change.
   */
  Result updateMarkerOnLastBody();

private:
  // Non-owning MuJoCo pointers.
  mjSpec*    spec_{};
  mjModel*   model_{};
  mjData*    data_{};
  ProGuiSim* sim_{};

  // Chain state.
  std::vector<ChainEntry> chain_;
  mjsBody*  lastBody_{nullptr};
  mjsGeom*  lastMarker_{nullptr};
  mjsGeom*  probeRect_{nullptr};
  std::string probeName_;

  // Direction tracking.
  std::vector<std::string> directionHistory_;
  bool suppressDirRecord_{false};

  // Spawn face.
  SpawnFace spawnFace_{SpawnFace::PosX};

  // Pre-physics saved positions.
  bool hasSavedPrePhysicsState_{false};
  std::vector<SavedBodyPos> savedPrePhysicsChain_;

  // Body id -> chain index map.
  std::unordered_map<int,size_t> bodyIdToIndex_;

  // Next cube numeric id counter.
  int nextCubeId_{0};

  // ---------------- Internal Helpers ----------------

  int parseCubeId(const char* name) const noexcept;
  void refreshNextCubeIdFromSpec();
  double currentGap(double gapRatio) const noexcept;

  bool loopContactCheck(std::string& outTargetBodyName);
  Result loopCreate(int spawnAxis, int spawnSign,
                    const std::string& newBodyName,
                    const std::string& targetBodyName);

  void deleteEqualitiesReferencing(const std::string& bodyName);
  void pruneHistoryOnDelete();
  void reconstructChainFromSpec();
  Result updateProbeForFace();

  void faceToAxisSign(SpawnFace face, int& axisIdx, int& sign) const noexcept {
    ProGuiSim::faceToAxisSign(face, axisIdx, sign);
  }

  static bool isTurnToken(const std::string& s);
  static bool isLoopToken(const std::string& s);
  Result doSpawn(bool fromToken);
  Result recompileAndForward(); // (unused placeholder for potential factoring)
  void updateJointLimitsForGap(double gapRatio);
  void zeroNewVelAct(int old_nv, int old_na);
  void forwardAndRefresh();
};

