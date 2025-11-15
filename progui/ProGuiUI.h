#pragma once
/**
 * @file ProGuiUI.h
 * @brief GLFW + MuJoCo UI event handling refactored from legacy progui_ui.cpp.
 *
 * Responsibilities:
 *  - Translate keyboard / mouse / scroll input into calls on ProGuiChain and ProGuiSim.
 *  - Manage transient file I/O capture state (enter filename then save/load).
 *  - Maintain an editable filename buffer (std::string) instead of legacy char array.
 *
 * Thread-safety: NOT thread-safe. All handlers assume invocation on main thread.
 * Error handling: Handlers return Result where failure is possible; pure event
 * callbacks with no recoverable failure return void.
 */

#include <GLFW/glfw3.h>
#include <string>
#include <string_view>
#include <cctype>
#include <vector>
#include <functional>
#include <mujoco/mjui.h>

// Forward declare MuJoCo renderer types to avoid heavy includes in header.


#include "Result.h"
#include "ProGuiSim.h"
#include "ProGuiChain.h"

/**
 * @brief File I/O capture state.
 */
enum class FileIOMode {
  None = 0,
  Save = 1,
  Load = 2
};

/**
 * @class ProGuiUI
 * @brief UI / input adapter translating raw events into high-level chain/sim operations.
 *
 * Design Notes:
 *  - Non-owning references to ProGuiSim and ProGuiChain; lifetime managed by caller.
 *  - Filename capture replaces legacy global char buffer with std::string.
 *  - Avoids direct rendering / mjUI panel manipulation (focus of refactor is structural).
 *
 * Error Handling:
 *  - Operational failures (e.g., spawnCube failure due to NotReady) are logged to std::cout.
 *  - Methods that cannot fail (e.g., onMouseMove) return void.
 *
 * Extension Points:
 *  - recordSpawnFaceChange placeholder kept for future UI synchronization.
 *  - Camera manipulation hooks retained as comments.
 */
class ProGuiUI {
public:
  // Button label constants (constexpr string_view replaces global const char*).
  static constexpr std::string_view kBtnSpawnCube       = "Spawn Cube (C)";
  static constexpr std::string_view kBtnStartPhysics    = "Start Physics (P)";
  static constexpr std::string_view kBtnResetChain      = "Reset Chain (R)";
  static constexpr std::string_view kBtnIncreaseGap     = "Increase Gap (+)";
  static constexpr std::string_view kBtnDecreaseGap     = "Decrease Gap (-)";
  static constexpr std::string_view kBtnSaveChain       = "Save Chain (Ctrl+S)";
  static constexpr std::string_view kBtnLoadChain       = "Load Chain (Ctrl+L)";
  static constexpr std::string_view kBtnSaveToFile      = "Save To File";
  static constexpr std::string_view kBtnLoadFromFile    = "Load From File";
  static constexpr std::string_view kBtnPrintDirections = "Print Directions";

  /**
   * @brief Construct UI adapter (non-owning).
   * @param window GLFWwindow pointer (may be null in headless/demo usage).
   * @param sim Reference to simulation manager.
   * @param chain Reference to chain manager.
   */
  ProGuiUI(GLFWwindow* window,
           ProGuiSim& sim,
           ProGuiChain& chain) noexcept
    : window_(window), sim_(sim), chain_(chain) {}

  ProGuiUI(const ProGuiUI&) = delete;
  ProGuiUI& operator=(const ProGuiUI&) = delete;
  ProGuiUI(ProGuiUI&&) = default;
  ProGuiUI& operator=(ProGuiUI&&) = default;
  ~ProGuiUI() = default;

  /**
   * @brief Handle keyboard input (refactored legacy keyboard()).
   *
   * Responsibilities:
   *  - Initiates file capture modes on Ctrl+S / Ctrl+L.
   *  - Delegates direction / face changes (arrow keys) to ProGuiChain::setSpawnFace().
   *  - Spawning (C), physics enable (P), reset (R), gap +/- (=/-, keypad add/subtract).
   *  - Snapshot (F5) saves pre-physics chain state.
   *
   * Recording:
   *  - Direction history mutated indirectly via chain methods.
   *
   * Preconditions:
   *  - sim_ and chain_ references valid.
   *
   * @param key GLFW key.
   * @param scancode Platform scancode (unused).
   * @param action GLFW_PRESS / GLFW_REPEAT / GLFW_RELEASE.
   * @param mods Bitmask of modifier keys.
   */
  void onKeyboard(int key, int scancode, int action, int mods);

  /**
   * @brief Handle mouse button press/release events.
   *
   * Updates internal button state flags (left/middle/right). Camera manipulation
   * integration is deferred (placeholder in onMouseMove).
   *
   * @param button GLFW mouse button identifier.
   * @param action GLFW_PRESS or GLFW_RELEASE.
   * @param mods   Modifier bitmask (currently ignored).
   */
  void onMouseButton(int button, int action, int mods);

  /**
   * @brief Handle cursor motion events.
   *
   * Tracks deltas (dx, dy) for potential future camera operations. Presently
   * discards the delta unless left button is pressed; hook retained for extensibility.
   *
   * @param xpos New X position in window coordinates.
   * @param ypos New Y position in window coordinates.
   */
  void onMouseMove(double xpos, double ypos);

  /**
   * @brief Handle scroll wheel input.
   *
   * Placeholder: legacy implementation adjusted camera zoom. Retained for
   * future expansion without side effects.
   *
   * @param xoffset Horizontal scroll offset.
   * @param yoffset Vertical scroll offset.
   */
  void onScroll(double xoffset, double yoffset);

  // -------- GUI Buttons --------
  /**
   * @brief Initialize a default set of GUI buttons.
   */
  void initDefaultButtons();

  /**
   * @brief Render GUI overlay (text-only for now) using MuJoCo renderer.
   * @param ctx      MuJoCo render context.
   * @param viewport Current framebuffer viewport.
   */
  void renderGUI(const mjrContext& ctx, const mjrRect& viewport);

  /**
   * @brief Current filename buffer view used during file capture.
   * @return std::string_view over internal filename.
   */
  [[nodiscard]] std::string_view filename() const noexcept { return ioFilename_; }

  /// File IO mode.
  [[nodiscard]] FileIOMode fileIOMode() const noexcept { return fileMode_; }

  /// Set external UI width hint.
  void setUIWidth(int w) noexcept { uiWidth_ = w; }

private:
  GLFWwindow*  window_{};
  ProGuiSim&   sim_;
  ProGuiChain& chain_;

  // Simple GUI button
  struct UIButton {
    double x{0}, y{0}, w{0}, h{0};   // window coords in pixels (origin: top-left)
    std::string label;
    std::function<void()> onClick;
    bool isInside(double px, double py) const noexcept {
      return (px >= x && px <= x + w && py >= y && py <= y + h);
    }
  };

  std::vector<UIButton> buttons_;

  std::string ioFilename_{"chain_save.txt"};
  FileIOMode  fileMode_{FileIOMode::None};
  int         uiWidth_{220};

  // Mouse state (locally tracked instead of global).
  bool buttonLeft_{false};
  bool buttonMiddle_{false};
  bool buttonRight_{false};
  double lastX_{0.0};
  double lastY_{0.0};

  // Append character to filename (bounded length).
  void appendFilenameChar(char c) {
    if (ioFilename_.size() < 255) ioFilename_.push_back(c);
  }

  // Handle file capture key input. Returns true if key consumed.
  bool handleFileCapture(int key, int action, int mods);

  // Interpret direction key into spawn face change.
  void applyFaceKey(int key);

  // Perform save/load based on current mode.
  void finalizeFileCapture();

  // Dispatch click to buttons; returns true if a button handled the click.
  bool dispatchButtonClick(double x, double y);

  // Map token "forward" spawn.
  void spawnForward();

  // Helpers for face mapping to direction tokens (delegates to chain).
  void recordSpawnFaceChange(ProGuiChain::ChainEntry const*); // placeholder

  // (UI button dispatch via mjUI omitted in refactor scope.)
};