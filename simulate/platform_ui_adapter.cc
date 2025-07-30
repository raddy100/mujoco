// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "platform_ui_adapter.h"

#include <chrono>

namespace mujoco {

// Constructor: Initializes the MuJoCo rendering context to default values.
PlatformUIAdapter::PlatformUIAdapter() {
  mjr_defaultContext(&con_);
}

// Frees the MuJoCo rendering context resources.
// Call this when the context is no longer needed (e.g., on shutdown).
void PlatformUIAdapter::FreeMjrContext() {
  mjr_freeContext(&con_);
}

// Refreshes (recreates) the MuJoCo rendering context if the model or font scale has changed.
// Returns true if the context was recreated, false otherwise.
bool PlatformUIAdapter::RefreshMjrContext(const mjModel* m, int fontscale) {
  if (m != last_model_ || fontscale != last_fontscale_) {
    mjr_makeContext(m, &con_, fontscale);
    last_model_ = m;
    last_fontscale_ = fontscale;
    return true;
  }
  return false;
}

// Ensures the rendering context matches the required size.
// This is a stub in this implementation and always returns false.
bool PlatformUIAdapter::EnsureContextSize() {
  return false;
}

// Handles file drop events (e.g., when files are dragged and dropped onto the window).
// Updates the UI state and triggers the event callback if set.
void PlatformUIAdapter::OnFilesDrop(int count, const char** paths) {
  state_.type = mjEVENT_FILESDROP;
  state_.dropcount = count;
  state_.droppaths = paths;

  // Application-specific processing via callback
  if (event_callback_) {
    event_callback_(&state_);
  }

  // Clean up: we don't own the paths pointer, so reset it
  state_.dropcount = 0;
  state_.droppaths = nullptr;
}

// Handles keyboard events.
// Translates the platform-specific key code, updates the UI state, and triggers the event callback.
void PlatformUIAdapter::OnKey(int key, int scancode, int act) {
  // Translate API-specific key code to MuJoCo key code
  int mj_key = TranslateKeyCode(key);

  // Only handle key down events
  if (!IsKeyDownEvent(act)) {
    return;
  }

  // Update UI state with current modifiers and mouse state
  UpdateMjuiState();

  // Set key event info
  state_.type = mjEVENT_KEY;
  state_.key = mj_key;
  state_.keytime = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // Application-specific processing via callback
  if (event_callback_) {
    event_callback_(&state_);
  }

  last_key_ = mj_key;
}

// Handles mouse button events (press/release).
// Translates the button, updates the UI state, handles double-clicks and dragging, and triggers the event callback.
void PlatformUIAdapter::OnMouseButton(int button, int act)  {
  // Translate API-specific mouse button code to MuJoCo button
  mjtButton mj_button = TranslateMouseButton(button);

  // Update UI state with current modifiers and mouse state
  UpdateMjuiState();

  // Swap left and right buttons if Alt is pressed
  if (state_.alt) {
    if (mj_button == mjBUTTON_LEFT) {
      mj_button = mjBUTTON_RIGHT;
    } else if (mj_button == mjBUTTON_RIGHT) {
      mj_button = mjBUTTON_LEFT;
    }
  }

  // Handle button press
  if (IsMouseButtonDownEvent(act)) {
    double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // Detect double-click (within 250 ms)
    if (mj_button == state_.button && now - state_.buttontime < 0.25) {
      state_.doubleclick = 1;
    } else {
      state_.doubleclick = 0;
    }

    // Set press event info
    state_.type = mjEVENT_PRESS;
    state_.button = mj_button;
    state_.buttontime = now;

    // Start dragging if mouse is over a rectangle
    if (state_.mouserect) {
      state_.dragbutton = state_.button;
      state_.dragrect = state_.mouserect;
    }
  }
  // Handle button release
  else {
    state_.type = mjEVENT_RELEASE;
  }

  // Application-specific processing via callback
  if (event_callback_) {
    event_callback_(&state_);
  }

  // Stop dragging after release
  if (state_.type == mjEVENT_RELEASE) {
    state_.dragrect = 0;
    state_.dragbutton = 0;
  }
}

// Handles mouse movement events while a button is pressed.
// Updates the UI state and triggers the event callback.
void PlatformUIAdapter::OnMouseMove(double x, double y) {
  // Ignore if no mouse buttons are pressed
  if (!state_.left && !state_.right && !state_.middle) {
    return;
  }

  // update state
  UpdateMjuiState();

  // set move info
  state_.type = mjEVENT_MOVE;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void PlatformUIAdapter::OnScroll(double xoffset, double yoffset) {
  // update state
  UpdateMjuiState();

  // set scroll info, scale by buffer-to-window ratio
  const double buffer_window_ratio =
      static_cast<double>(GetFramebufferSize().first) / GetWindowSize().first;
  state_.type = mjEVENT_SCROLL;
  state_.sx = xoffset * buffer_window_ratio;
  state_.sy = yoffset * buffer_window_ratio;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void PlatformUIAdapter::OnWindowRefresh() {
  state_.type = mjEVENT_REDRAW;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void PlatformUIAdapter::OnWindowResize(int width, int height) {
  auto [buf_width, buf_height] = GetFramebufferSize();
  state_.rect[0].width = buf_width;
  state_.rect[0].height = buf_height;
  if (state_.nrect < 1) state_.nrect = 1;

  // update window layout
  if (layout_callback_) {
    layout_callback_(&state_);
  }

  // update state
  UpdateMjuiState();

  // set resize info
  state_.type = mjEVENT_RESIZE;

  // stop dragging
  state_.dragbutton = 0;
  state_.dragrect = 0;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void PlatformUIAdapter::UpdateMjuiState() {
  // mouse buttons
  state_.left = IsLeftMouseButtonPressed();
  state_.right = IsRightMouseButtonPressed();
  state_.middle = IsMiddleMouseButtonPressed();

  // keyboard modifiers
  state_.control = IsCtrlKeyPressed();
  state_.shift = IsShiftKeyPressed();
  state_.alt = IsAltKeyPressed();

  // swap left and right if Alt
  if (state_.alt) {
    int tmp = state_.left;
    state_.left = state_.right;
    state_.right = tmp;
  }

  // get mouse position, scale by buffer-to-window ratio
  auto [x, y] = GetCursorPosition();
  const double buffer_window_ratio =
      static_cast<double>(GetFramebufferSize().first) / GetWindowSize().first;
  x *= buffer_window_ratio;
  y *= buffer_window_ratio;

  // invert y to match OpenGL convention
  y = state_.rect[0].height - y;

  // save
  state_.dx = x - state_.x;
  state_.dy = y - state_.y;
  state_.x = x;
  state_.y = y;

  // find mouse rectangle
  state_.mouserect = mjr_findRect(mju_round(x), mju_round(y), state_.nrect-1, state_.rect+1) + 1;
}
}  // namespace mujoco
