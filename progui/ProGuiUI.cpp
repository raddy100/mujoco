#include "ProGuiUI.h"
#include <iostream>
#include <filesystem>
#include <cctype>
#include <sstream>
#include <mujoco/mujoco.h>
#include <mujoco/mjvisualize.h>

// ---------------- Internal Helpers ----------------

bool ProGuiUI::handleFileCapture(int key, int action, int mods) {
  if (fileMode_ == FileIOMode::None) {
    // Initiate capture (Ctrl+S / Ctrl+L) if not already in capture mode.
    if ((mods & GLFW_MOD_CONTROL) && action == GLFW_PRESS) {
      if (key == GLFW_KEY_S) {
        fileMode_ = FileIOMode::Save;
        ioFilename_.clear();
        std::cout << "[capture/save] Enter filename then press Enter\n";
        return true;
      } else if (key == GLFW_KEY_L) {
        fileMode_ = FileIOMode::Load;
        ioFilename_.clear();
        std::cout << "[capture/load] Enter filename then press Enter\n";
        return true;
      }
    }
    return false;
  }

  // Active capture mode.
  if (action != GLFW_PRESS && action != GLFW_REPEAT) return true;

  if (key == GLFW_KEY_ESCAPE) {
    fileMode_ = FileIOMode::None;
    ioFilename_.clear();
    std::cout << "[capture] canceled\n";
    return true;
  }

  if (key == GLFW_KEY_ENTER || key == GLFW_KEY_KP_ENTER) {
    finalizeFileCapture();
    return true;
  }

  if (key == GLFW_KEY_BACKSPACE) {
    if (!ioFilename_.empty()) ioFilename_.pop_back();
    return true;
  }

  // Simple accepted characters; rely on keyboard layout for others.
  if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z) {
    char c = static_cast<char>('a' + (key - GLFW_KEY_A));
    appendFilenameChar(c);
    return true;
  }
  if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
    char c = static_cast<char>('0' + (key - GLFW_KEY_0));
    appendFilenameChar(c);
    return true;
  }

  // Symbols.
  switch (key) {
    case GLFW_KEY_PERIOD: appendFilenameChar('.'); return true;
    case GLFW_KEY_MINUS:  appendFilenameChar('-'); return true;
    case GLFW_KEY_SLASH:  appendFilenameChar('/'); return true;
    default: break;
  }

  return true;
}

void ProGuiUI::finalizeFileCapture() {
  if (fileMode_ == FileIOMode::None) return;
  std::filesystem::path p(ioFilename_);
  if (p.empty()) {
    std::cout << "[capture] empty filename ignored\n";
    fileMode_ = FileIOMode::None;
    return;
  }
  if (fileMode_ == FileIOMode::Save) {
    auto r = chain_.saveDirections(p);
    std::cout << "[save] " << (r.ok() ? "ok: " : "fail: ") << r.message << "\n";
  } else if (fileMode_ == FileIOMode::Load) {
    auto r = chain_.loadDirections(p);
    std::cout << "[load] " << (r.ok() ? "ok: " : "fail: ") << r.message << "\n";
  }
  fileMode_ = FileIOMode::None;
}

void ProGuiUI::spawnForward() {
  if (!chain_.canSpawnCube()) {
    std::cout << "[spawn] prerequisites missing\n";
    return;
  }
  auto r = chain_.spawnCube();
  if (!r.ok()) {
    std::cout << "[spawn] failed: " << r.message << "\n";
  }
}

void ProGuiUI::applyFaceKey(int key) {
  if (!chain_.lastMarker()) {
    std::cout << "[cursor] target marker unavailable\n";
    return;
  }
  SpawnFace cur = chain_.spawnFace();
  SpawnFace next = cur;
  if (key == GLFW_KEY_UP) {
    next = SpawnFace::PosY;
  } else if (key == GLFW_KEY_DOWN) {
    next = SpawnFace::NegY;
  } else if (key == GLFW_KEY_LEFT) {  
    next = SpawnFace::NegX;    
  } else if (key == GLFW_KEY_RIGHT) {   
    next = SpawnFace::PosX;
  }
  else if (key == GLFW_KEY_W){
      next = SpawnFace::PosZ;
  }
  else if (key == GLFW_KEY_X)
  {
      next = SpawnFace::NegZ;
  }
  if (next != cur) {
    auto r = chain_.setSpawnFace(next);
    if (!r.ok()) {
      std::cout << "[face] change failed: " << r.message << "\n";
    } else {
      std::cout << "[face] changed\n";
    }
  }
}

void ProGuiUI::recordSpawnFaceChange(ProGuiChain::ChainEntry const*) {
  // Placeholder for potential extended logic (e.g. UI sync).
}

// ---------------- Public Event Handlers ----------------

void ProGuiUI::onKeyboard(int key, int scancode, int action, int mods) {
  (void)scancode;
  if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

  // File capture first; if active consume all keys.
  if (fileMode_ != FileIOMode::None) {
    handleFileCapture(key, action, mods);
    return;
  }

  // Initiate capture if Ctrl+S or Ctrl+L.
  if (handleFileCapture(key, action, mods)) return;

  switch (key) {
    case GLFW_KEY_C:
      if (!(mods & GLFW_MOD_CONTROL)) spawnForward();
      break;
    case GLFW_KEY_P:
      if (!(mods & GLFW_MOD_CONTROL)) {
         auto r = chain_.savePrePhysicsState();
         if (!r.ok())
              std::cout << "[snapshot] failed: " << r.message << "\n";
         r = chain_.enablePhysics();
        if (!r.ok()) std::cout << "[physics] enable failed: " << r.message << "\n";
      }
      break;
    case GLFW_KEY_R:
      if (!(mods & GLFW_MOD_CONTROL)) {
        auto r = chain_.resetToSavedState();
        if (!r.ok()) std::cout << "[reset] failed: " << r.message << "\n";
      }
      break;
    case GLFW_KEY_S:
      if ((mods & GLFW_MOD_CONTROL) && action == GLFW_PRESS) {
        handleFileCapture(key, action, mods);
      }
      break;
    case GLFW_KEY_L:
      if ((mods & GLFW_MOD_CONTROL) && action == GLFW_PRESS) {
        handleFileCapture(key, action, mods);
      }
      break;
    case GLFW_KEY_EQUAL: // '+' with shift
    case GLFW_KEY_KP_ADD:
      if (!(mods & GLFW_MOD_CONTROL)) {
        auto r = sim_.increaseGap();
        if (!r.ok()) std::cout << "[gap+] failed: " << r.message << "\n";
      }
      break;
    case GLFW_KEY_MINUS:
    case GLFW_KEY_KP_SUBTRACT:
      if (!(mods & GLFW_MOD_CONTROL)) {
        auto r = sim_.decreaseGap();
        if (!r.ok()) std::cout << "[gap-] failed: " << r.message << "\n";
      }
      break;
    case GLFW_KEY_UP:
    case GLFW_KEY_DOWN:
    case GLFW_KEY_LEFT:
    case GLFW_KEY_RIGHT:
    case GLFW_KEY_W:
    case GLFW_KEY_X:
      applyFaceKey(key);
      break;
    case GLFW_KEY_F5:
      // Save pre-physics snapshot.
      {
        auto r = chain_.savePrePhysicsState();
        if (!r.ok()) std::cout << "[snapshot] failed: " << r.message << "\n";
      }
      break;
    case GLFW_KEY_DELETE:
      if (chain_.canDeleteTarget()) {
        auto r = chain_.deleteLastCube();
        if (!r.ok()) std::cout << "[delete] failed: " << r.message << "\n";
      } else {
        std::cout << "[delete] no deletable cube\n";
      }
      break;
    default:
      break;
  }
}

void ProGuiUI::onMouseButton(int button, int action, int mods) {
  (void)mods;
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      buttonLeft_ = true;
      // Sync cursor to current position at click time
      double cx = lastX_, cy = lastY_;
      if (window_) { glfwGetCursorPos(window_, &cx, &cy); }
      lastX_ = cx; lastY_ = cy;

      // First, attempt to dispatch as a GUI button click.
      const bool handled = dispatchButtonClick(cx, cy);
      // If not handled by a GUI button, begin camera drag.
      if (!handled) {
        sim_.cameraBeginDrag(cx, cy);
      }
    } else if (action == GLFW_RELEASE) {
      buttonLeft_ = false;
      sim_.cameraEndDrag();
    }
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    buttonMiddle_ = (action == GLFW_PRESS);
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    buttonRight_ = (action == GLFW_PRESS);
  }
}

void ProGuiUI::onMouseMove(double xpos, double ypos) {
  // Track deltas for potential consumers.
  double dx = xpos - lastX_;
  double dy = ypos - lastY_;
  lastX_ = xpos;
  lastY_ = ypos;
  if (buttonLeft_) {
    // Route drag to camera.
    sim_.cameraDrag(xpos, ypos);
    (void)dx; (void)dy;
  }
}

void ProGuiUI::onScroll(double xoffset, double yoffset) {
  (void)xoffset;
  // Zoom camera with vertical scroll.
  sim_.cameraZoom(yoffset);
}

// -------- GUI Buttons --------

bool ProGuiUI::dispatchButtonClick(double x, double y) {
  // Iterate top-most first (last added on top).
  for (auto it = buttons_.rbegin(); it != buttons_.rend(); ++it) {
    if (it->isInside(x, y)) {
      if (it->onClick) it->onClick();
      return true;
    }
  }
  return false;
}

void ProGuiUI::initDefaultButtons() {
  buttons_.clear();
  // Basic vertical stack at left margin.
  double x = 12.0;
  double y = 12.0;
  const double w = 180.0;
  const double h = 28.0;
  const double pad = 8.0;

  auto addBtn = [&](std::string label, std::function<void()> fn) {
    buttons_.push_back(UIButton{x, y, w, h, std::move(label), std::move(fn)});
    y += h + pad;
  };

  addBtn(std::string(kBtnSpawnCube), [this]() { this->spawnForward(); });
  addBtn(std::string(kBtnStartPhysics), [this]() {
    auto r = chain_.enablePhysics();
    if (!r.ok()) std::cout << "[physics] enable failed: " << r.message << "\n";
  });
  addBtn(std::string(kBtnResetChain), [this]() {
    auto r = chain_.resetToSavedState();
    if (!r.ok()) std::cout << "[reset] failed: " << r.message << "\n";
  });
  addBtn(std::string(kBtnIncreaseGap), [this]() {
    auto r = sim_.increaseGap();
    if (!r.ok()) std::cout << "[gap+] failed: " << r.message << "\n";
  });
  addBtn(std::string(kBtnDecreaseGap), [this]() {
    auto r = sim_.decreaseGap();
    if (!r.ok()) std::cout << "[gap-] failed: " << r.message << "\n";
  });
  addBtn(std::string(kBtnSaveToFile), [this]() {
    fileMode_ = FileIOMode::Save;
    ioFilename_.clear();
    std::cout << "[capture/save] Enter filename then press Enter\n";
  });
  addBtn(std::string(kBtnLoadFromFile), [this]() {
    fileMode_ = FileIOMode::Load;
    ioFilename_.clear();
    std::cout << "[capture/load] Enter filename then press Enter\n";
  });
  // (Print Directions button removed: method not defined in ProGuiChain)
}

void ProGuiUI::renderGUI(const mjrContext& ctx, const mjrRect& viewport) {
  // Text-only GUI overlay listing buttons; clickable hitboxes are active separately.
  std::ostringstream os;
  os << "Buttons (click in window regions):\n";
  for (const auto& b : buttons_) {
    os << " - " << b.label << "\n";
  }
  os << "\nMouse: L-drag = orbit, wheel = zoom";

  // Top-left overlay
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, os.str().c_str(), nullptr, &ctx);
}