#pragma once

#include "chainmaker.h"

// Apply the current world.sim_preset to the live mjModel* (no-op if not in sim).
// Patches opt.solver/timestep/iterations and toggles contype/conaffinity on
// box vs sphere geoms.  Safe to call mid-simulation: takes effect next mj_step.
void ApplySimPreset(AppState& app);

// Enter the simulation stage: compile world → mjModel, start physics.
void EnterSimulation(AppState& app);

// Exit simulation, return to build stage.
void ExitSimulation(AppState& app);

// Advance simulation by one display frame (~1/60 s of physics).
void StepSimulation(AppState& app);

// Render the profiler overlay (timers, contacts, DOF).
void RenderProfilerOverlay(const AppState& app, mjrRect viewport);

// Start recording video to the given file path via ffmpeg.
// Returns true on success. Recording works in both BUILD and SIMULATE modes.
bool StartRecording(AppState& app, const char* path);

// Stop an active recording and close the ffmpeg pipe.
void StopRecording(AppState& app);

// Toggle recording on/off (F9 key). Generates timestamped filename if needed.
void ToggleRecording(AppState& app);

// Capture the current framebuffer and write it to the ffmpeg pipe.
// Call this after all rendering but before glfwSwapBuffers.
void CaptureRecordingFrame(AppState& app, mjrRect viewport);
