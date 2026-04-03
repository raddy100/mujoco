#pragma once

#include "chainmaker.h"

// Build the mjUI panel and initialize mjuiState.
void BuildUI(AppState& app);

// Per-frame UI rect setup (must be called before mjui_update).
void SetupUIRect(AppState& app, mjrRect viewport);

// Handle a UI button click event.
void HandleUIClick(AppState& app, mjuiItem* item, int glfw_action);

// GLFW callbacks — registered in main.cc
void KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods);
void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods);
void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

// File I/O key handler (called while io_mode != 0)
void HandleFileIOKey(AppState& app, int key, int mods);
