#include "chainmaker.h"
#include "chainmaker_render.h"
#include "chainmaker_ui.h"
#include "chainmaker_sim.h"
#include "chainmaker_ipc.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

// ---------------------------------------------------------------------------
// Global application state (all other files access via extern)
// ---------------------------------------------------------------------------

AppState g_app;

// ---------------------------------------------------------------------------
// UpdateBuildCamera — compute mjvGLCamera from mjvCamera for the NULL-model
// build stage (mjv_updateCamera requires a valid mjModel).
// ---------------------------------------------------------------------------

static void UpdateBuildCamera(AppState& app) {
    const double kPi = 3.14159265358979323846;
    double az  = app.cam.azimuth   * kPi / 180.0;
    double el  = app.cam.elevation * kPi / 180.0;
    double ca  = cos(az), sa = sin(az);
    double ce  = cos(el), se = sin(el);
    double dist = std::max(0.05, app.cam.distance);

    // Forward vector (lookat → eye direction is -forward)
    double fwdX = ce*ca, fwdY = ce*sa, fwdZ = se;
    // Up vector
    double upX  = -se*ca, upY = -se*sa, upZ = ce;
    // Eye position
    double eyeX = app.cam.lookat[0] - dist*fwdX;
    double eyeY = app.cam.lookat[1] - dist*fwdY;
    double eyeZ = app.cam.lookat[2] - dist*fwdZ;

    float near_f    = (float)(0.005 * dist);
    float far_f     = (float)(100.0 * dist);
    float half_fov  = (float)(kPi * 45.0 / 180.0 / 2.0);
    float top       = near_f * (float)tan(half_fov);

    for (int eye = 0; eye < 2; eye++) {
        mjvGLCamera& gc = app.scn.camera[eye];
        gc.pos[0]     = (float)eyeX;
        gc.pos[1]     = (float)eyeY;
        gc.pos[2]     = (float)eyeZ;
        gc.forward[0] = (float)fwdX;
        gc.forward[1] = (float)fwdY;
        gc.forward[2] = (float)fwdZ;
        gc.up[0]      = (float)upX;
        gc.up[1]      = (float)upY;
        gc.up[2]      = (float)upZ;
        gc.frustum_near   = near_f;
        gc.frustum_far    = far_f;
        gc.frustum_center = 0.0f;
        gc.frustum_bottom = -top;
        gc.frustum_top    = top;
        gc.orthographic   = 0;
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, const char** argv) {

    // --- Parse command-line flags ---
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--test") == 0) {
            g_app.test_mode = true;
            std::cout << "[ChainMaker] Test mode enabled — IPC server will start on port 47832\n";
        }
    }

    // --- GLFW init ---
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW\n";
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(1400, 900,
                                          "ChainMaker v0.1 — Bead-on-String Robot Designer",
                                          nullptr, nullptr);
    if (!window) {
        std::cerr << "Could not create GLFW window\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    g_app.window = window;

    // --- MuJoCo visualization (NULL model — build stage) ---
    mjv_defaultCamera(&g_app.cam);
    mjv_defaultOption(&g_app.opt);
    mjv_defaultScene(&g_app.scn);
    mjr_defaultContext(&g_app.con);

    mjv_makeScene(nullptr, &g_app.scn, kMaxSceneGeoms);

    // Create minimal model so mjr_makeContext initializes built-in geometry VBOs.
    // Without this, mjr_makeContext(nullptr,...) returns early and skips makeBuiltin(),
    // leaving no VBOs for box/sphere/arrow rendering in the build stage.
    {
        mjSpec* spec = mj_makeSpec();
        char err[200] = {};
        g_app.build_model = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        if (!g_app.build_model) {
            std::cerr << "Failed to create minimal build model: " << err << "\n";
            glfwDestroyWindow(window);
            glfwTerminate();
            return 1;
        }
    }
    mjr_makeContext(g_app.build_model, &g_app.con, mjFONTSCALE_150);

    // Camera initial position
    g_app.cam.type      = mjCAMERA_FREE;
    g_app.cam.lookat[0] = 0.3;
    g_app.cam.lookat[1] = 0.0;
    g_app.cam.lookat[2] = 0.15;
    g_app.cam.azimuth   = 135.0;
    g_app.cam.elevation = -25.0;
    g_app.cam.distance  = 1.5;

    g_app.opt.frame = mjFRAME_NONE;

    // --- Build UI panel ---
    BuildUI(g_app);

    // Compute UI item positions now that we have valid framebuffer dimensions.
    // mjui_resize inside BuildUI runs before any rect is configured, so item
    // bounding boxes would be zero and mjui_event could never hit-test them.
    {
        int fbw, fbh;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        SetupUIRect(g_app, {0, 0, fbw, fbh});
        mjui_resize(&g_app.ui, &g_app.con);
    }

    // --- Create first chain ---
    CreateFirstChain(g_app.world);

    // --- Start IPC test server (only if --test flag was given) ---
    if (g_app.test_mode) {
        IpcServerStart(g_app);
    }

    // --- Register GLFW callbacks ---
    glfwSetKeyCallback(window,         KeyboardCallback);
    glfwSetCursorPosCallback(window,   MouseMoveCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetScrollCallback(window,      ScrollCallback);

    // ---------------------------------------------------------------------------
    // Main render loop
    // ---------------------------------------------------------------------------
    while (!glfwWindowShouldClose(window)) {

        // Physics step (simulation stage only)
        if (g_app.mode == AppMode::SIMULATE) {
            StepSimulation(g_app);
        }

        // Get framebuffer dimensions
        int fb_width = 0, fb_height = 0;
        glfwGetFramebufferSize(window, &fb_width, &fb_height);
        mjrRect viewport = {0, 0, fb_width, fb_height};

        // Update scene
        if (g_app.mode == AppMode::SIMULATE && g_app.sim_model) {
            mjv_updateScene(g_app.sim_model, g_app.sim_data, &g_app.opt,
                            nullptr, &g_app.cam, mjCAT_ALL, &g_app.scn);
        } else {
            // Build stage: manually compute GL camera (no valid mjModel)
            UpdateBuildCamera(g_app);
            PopulateBuildScene(g_app);
        }

        // Update UI layout for this frame; resize recomputes item bounding boxes
        // so mjui_event hit-testing works correctly after window resize.
        SetupUIRect(g_app, viewport);
        mjui_resize(&g_app.ui, &g_app.con);
        mjui_update(-1, -1, &g_app.ui, &g_app.uistate, &g_app.con);

        // Render 3D scene
        mjr_render(viewport, &g_app.scn, &g_app.con);

        // Render overlays (status bar, mode indicator, file I/O prompt)
        RenderOverlays(g_app, viewport);

        // Profiler overlay (always shown in sim mode; P key toggles full breakdown)
        if (g_app.mode == AppMode::SIMULATE) {
            RenderProfilerOverlay(g_app, viewport);
        }

        // Render UI panel (on top)
        mjui_render(&g_app.ui, &g_app.uistate, &g_app.con);

        // IPC: process one pending test command per frame.
        // Called after all rendering so screenshots capture the current frame.
        if (g_app.test_mode) {
            IpcProcessCommands(g_app, viewport);
        }

        // Capture video frame (after all rendering, before swap)
        if (g_app.is_recording) {
            CaptureRecordingFrame(g_app, viewport);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ---------------------------------------------------------------------------
    // Cleanup
    // ---------------------------------------------------------------------------
    if (g_app.is_recording) StopRecording(g_app);
    if (g_app.sim_data)    mj_deleteData(g_app.sim_data);
    if (g_app.sim_model)   mj_deleteModel(g_app.sim_model);
    if (g_app.sim_spec)    mj_deleteSpec(g_app.sim_spec);
    if (g_app.build_model) mj_deleteModel(g_app.build_model);

    IpcServerStop(g_app);

    mjv_freeScene(&g_app.scn);
    mjr_freeContext(&g_app.con);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
