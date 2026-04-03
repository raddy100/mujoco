#pragma once

// ---------------------------------------------------------------------------
// ChainMaker IPC Test Server
//
// When ChainMaker is launched with the --test flag it opens a TCP server on
// port 47832 (localhost only).  A Python test script connects and sends
// newline-delimited JSON commands; the server responds with newline-delimited
// JSON replies.  All command execution happens on the main GL thread to avoid
// any OpenGL or MuJoCo thread-safety issues.
// ---------------------------------------------------------------------------

#include <mujoco/mujoco.h>   // mjrRect needs full definition

struct AppState;

// Opaque server object (defined in chainmaker_ipc.cpp)
struct IpcServer;

// Start the background accept/recv thread.  Call once after the GLFW window
// has been created (so the GL context is current on the main thread).
void IpcServerStart(AppState& app);

// Shut down the server and free resources.  Safe to call even if the server
// was never started.
void IpcServerStop(AppState& app);

// Process at most one pending command per call.  Must be called from the main
// GL thread, after mjr_render() so that screenshot commands capture the
// current frame.  viewport is the full window rectangle.
void IpcProcessCommands(AppState& app, mjrRect viewport);
