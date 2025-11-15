#pragma once
/**
 * @file ProGuiSim.h
 * @brief Simulation and visualization management for refactored ProGUI.
 *
 * Encapsulates MuJoCo model/data/spec pointers (non-owning by default) and
 * provides high-level operations previously implemented as free functions:
 *  - Rebuild of render context
 *  - Physics option application
 *  - Gap ratio adjustment and propagation
 *  - Probe contact queries / debug printing
 *  - Axis/sign derivation from spawn face
 *  - Body id index map maintenance
 *
 * Thread-safety: NOT thread-safe. All methods expect single-threaded invocation
 * from the main simulation/UI loop. External synchronization required if used
 * concurrently.
 *
 * Lifetime & Ownership:
 *  - By default, ProGuiSim does NOT own mjModel/mjData/mjSpec; caller retains
 *    responsibility for creation/destruction. Ownership transfer may be added later.
 *
 * Error Handling:
 *  - Recoverable failures return Result (see Result.h).
 *  - Construction validates pointers and may throw std::invalid_argument if
 *    mandatory ones are null (when validate flag is true).
 */ 

#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <optional>
#include <cassert>

#include "Result.h"
#include <GLFW/glfw3.h>

#define M_PI 3.14159265358979323846 // pi
/**
 * @brief Strongly typed spawn face enumeration replacing legacy int constants.
 */
enum class SpawnFace : int {
  PosX = 0,
  NegX = 1,
  PosY = 2,
  NegY = 3,
  PosZ = 4,
  NegZ = 5
};

/**
 * @class ProGuiSim
 * @brief Simulation / rendering manager. Provides operations affecting global physics
 *        and visualization state that were formerly spread across multiple C-style files.
 */
class ProGuiSim {
public:
  // Physics tuning constants (constexpr instead of mutable globals).
  static constexpr double kJointDamping = 0.15;
  static constexpr double kGeomMargin   = 0.001;
  static constexpr double kSolref[2]    = {0.15, 0.7};
  static constexpr double kSolimp[5]    = {0.9, 0.95, 0.001, 0.5, 2.0};
  static constexpr double kBoxHalf      = 0.05;

  /**
   * @brief Construct with non-owning references to MuJoCo core structures (no validation).
   */
  ProGuiSim(mjModel* model,
            mjData* data,
            mjSpec* spec,
            mjvScene* scene,
            mjrContext* context,
            mjUI* ui,
            mjuiState* uiState,
            mjvCamera* camera,
            mjvOption* option) noexcept;

  /**
   * @brief Construct with optional validation. Throws std::invalid_argument if any mandatory pointer is null.
   * @param validate Set true to enforce non-null pointer checks.
   */
  ProGuiSim(mjModel* model,
            mjData* data,
            mjSpec* spec,
            mjvScene* scene,
            mjrContext* context,
            mjUI* ui,
            mjuiState* uiState,
            mjvCamera* camera,
            mjvOption* option,
            bool validate);

  ProGuiSim(const ProGuiSim&) = delete;
  ProGuiSim& operator=(const ProGuiSim&) = delete;
  ProGuiSim(ProGuiSim&&) = default;
  ProGuiSim& operator=(ProGuiSim&&) = default;
  ~ProGuiSim() = default; // Non-owning; no deletion of MuJoCo objects.

  /**
   * @brief Rebuild render context (scene + font atlas) and resize UI.
   * @return Result::Ok on success, NotReady if required pointers are null.
   */
  Result rebuildRenderContext() noexcept;

  /**
   * @brief Apply build-mode physics options: zero gravity & disable gravity flag if physics disabled.
   * @return Result signaling success or NotReady.
   */
  Result applyBuildModeOptions() noexcept;

  /**
   * @brief Increase gap ratio by fixed increment (0.05) clamped to 1.0 and invoke layout callback.
   */
  Result increaseGap();

  /**
   * @brief Decrease gap ratio by fixed increment (0.05) clamped to 0.0 and invoke layout callback.
   */
  Result decreaseGap();

  /// Current gap ratio.
  [[nodiscard]] double gapRatio() const noexcept { return gapRatio_; }
  /// Physics enabled flag.
  [[nodiscard]] bool physicsEnabled() const noexcept { return physicsEnabled_; }
  /// Box collision mode flag.
  [[nodiscard]] bool boxesCollideEnabled() const noexcept { return boxesCollide_; }

  /**
   * @brief Enable physics: restore saved gravity and clear gravity disable flag.
   */
  Result enablePhysics();

  /**
   * @brief Set collision mode for subsequently created geoms.
   */
  void setBoxesCollide(bool enabled) noexcept { boxesCollide_ = enabled; }

  /**
   * @brief Capture current gravity from model before disabling physics.
   */
  void captureCurrentGravity() noexcept;

  /**
   * @brief Derive axis index and sign from spawn face.
   * @param face spawn face enumeration
   * @param axisIdx output axis index (0/1/2)
   * @param sign output sign (+1/-1)
   */
  static inline void faceToAxisSign(SpawnFace face, int& axisIdx, int& sign) noexcept {
    switch (face) {
      case SpawnFace::PosX: axisIdx = 0; sign = +1; break;
      case SpawnFace::NegX: axisIdx = 0; sign = -1; break;
      case SpawnFace::PosY: axisIdx = 1; sign = +1; break;
      case SpawnFace::NegY: axisIdx = 1; sign = -1; break;
      case SpawnFace::PosZ: axisIdx = 2; sign = +1; break;
      case SpawnFace::NegZ: axisIdx = 2; sign = -1; break;
      default:              axisIdx = 0; sign = +1; break;
    }
  }

  /**
   * @brief Print probe contacts for debugging, listing each contact involving the probe.
   * @param probeGeomName Name of the probe geometry to search contacts for (must exist in model).
   * @param bodyIdToIndex Mapping from MuJoCo body id to chain index for contextual logging.
   * @return Result::success on completion (even if no contacts), Result::InvalidArgument if probe name
   *         unresolved, Result::NotReady if required model/data pointers are null.
   */
  Result debugPrintProbeContacts(std::string_view probeGeomName,
                                 const std::unordered_map<int, size_t>& bodyIdToIndex) const;

  /**
   * @brief Query body id first hit by the named probe geometry (excluding newest body).
   * @param probeGeomName Name of probe geometry.
   * @param newestBodyId Body id of most recently spawned cube to ignore (avoid self-hit).
   * @return std::optional<int> containing body id of first qualifying contact or std::nullopt if no hit
   *         or prerequisites (model/data or name) missing.
   */
  std::optional<int> probeHitBodyId(std::string_view probeGeomName,
                                    int newestBodyId) const;

  /**
   * @brief Populate map from body id to chain vector index.
   * @param bodyNames vector of body names
   * @param outMap output map
   */
  Result updateBodyIdIndexMap(const std::vector<std::string>& bodyNames,
                              std::unordered_map<int, size_t>& outMap);

  /**
   * @brief Register callback invoked when gap ratio changes.
   */
  void setApplyGapLayoutCallback(void(*fn)(double ratio)) noexcept {
    applyGapLayoutFn_ = fn;
  }

  /**
   * @brief Mark physics enabled internal flag (used during reset).
   */
  void markPhysicsEnabled(bool enabled) noexcept { physicsEnabled_ = enabled; }
  
  /**
   * @brief Mark OpenGL initialization readiness for rendering operations.
   */
  void markGlReady(bool ready) noexcept { glReady_ = ready; }
  
  /**
   * @brief Query whether OpenGL has been initialized (GL loader + context current).
   */
  [[nodiscard]] bool glReady() const noexcept { return glReady_; }

  // -------- Camera control (mouse-driven) --------
  /**
   * @brief Begin camera drag at window coords (x,y).
   */
  void cameraBeginDrag(double x, double y) noexcept;
  /**
   * @brief Update camera drag with new window coords (x,y).
   */
  void cameraDrag(double x, double y) noexcept;
  /**
   * @brief End current camera drag.
   */
  void cameraEndDrag() noexcept;
  /**
   * @brief Zoom camera in/out by scroll offset (positive zooms in).
   */
  void cameraZoom(double yoffset) noexcept;
  /**
   * @brief Apply internal yaw/pitch/distance to mjvCamera for this frame.
   */
  void updateCamera() noexcept;
  /**
   * @brief Whether camera is currently being dragged.
   */
  [[nodiscard]] bool cameraDragging() const noexcept { return camDragging_; }
  
  private:
  // Non-owning MuJoCo pointers (asserted where needed).
  mjModel*    model_{};
  mjData*     data_{};
  mjSpec*     spec_{};
  mjvScene*   scene_{};
  mjrContext* context_{};
  mjUI*       ui_{};
  mjuiState*  uiState_{};
  mjvCamera*  camera_{};
  mjvOption*  option_{};

  double gapRatio_{0.05};
  bool boxesCollide_{false};
  bool physicsEnabled_{false};
  mjtNum savedGravity_[3]{0,0,0};
  bool glReady_{false};
  bool firstInit{true};

  // Camera interaction state
  bool   camDragging_{false};
  double camLastX_{0.0};
  double camLastY_{0.0};
  double camYawDeg_{90.0};     // initialized from mjvCamera in ctor if available
  double camPitchDeg_{-25.0};  // initialized from mjvCamera in ctor if available
  double camDistance_{3.0};    // initialized from mjvCamera in ctor if available
  double camRotSensitivity_{0.25};   // deg per pixel
  double camZoomSensitivity_{0.12};  // exponential zoom factor
  double camMinDistance_{0.2};
  double camMaxDistance_{100.0};

  void (*applyGapLayoutFn_)(double) = nullptr; // Chain callback

  // Internal helpers.
  Result recompileIfNeeded(); // placeholder if future logic required

  // Clamp & propagate gap change.
  void applyGapChange() noexcept;
};