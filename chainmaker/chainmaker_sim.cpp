#include "chainmaker_sim.h"
#include "chainmaker_compile.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#ifdef _WIN32
#  define popen  _popen
#  define pclose _pclose
#endif

// ---------------------------------------------------------------------------
// Timer callback for MuJoCo profiling — returns elapsed time in milliseconds.
// MuJoCo stores cumulative timer values in d->timer[i].duration using the
// same units this callback returns.
// ---------------------------------------------------------------------------

static mjtNum ChainmakerTimer() {
    static auto t0 = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0);
    return static_cast<mjtNum>(elapsed.count());
}

// Real-time ratio tracking (updated by StepSimulation)
static double g_rt_ratio = 0.0;   // physics_time / wall_time  (1.0 = real-time)

// ---------------------------------------------------------------------------
// EnterSimulation
// ---------------------------------------------------------------------------

void EnterSimulation(AppState& app) {
    if (app.world.chains.empty()) {
        std::cerr << "EnterSimulation: no chains to simulate\n";
        return;
    }
    if (app.mode == AppMode::SIMULATE) {
        ExitSimulation(app);
        return;
    }

    CompileResult res = CompileWorld(app.world);
    if (!res.model) {
        std::cerr << "EnterSimulation: compile failed: " << res.error << "\n";
        return;
    }

    app.sim_spec  = res.spec;
    app.sim_model = res.model;
    app.sim_data  = res.data;
    app.mode      = AppMode::SIMULATE;
    g_rt_ratio    = 0.0;

    // Install profiling timer (must be set BEFORE any mj_step calls)
    mjcb_time = ChainmakerTimer;

    // Install contact filter
    mjcb_contactfilter = ChainmakerContactFilter;

    // Rebuild render context with the compiled model
    mjv_freeScene(&app.scn);
    mjr_freeContext(&app.con);

    mjv_defaultScene(&app.scn);
    mjr_defaultContext(&app.con);
    mjv_makeScene(app.sim_model, &app.scn, kMaxSceneGeoms);
    mjr_makeContext(app.sim_model, &app.con, mjFONTSCALE_150);

    mjui_resize(&app.ui, &app.con);
    mjr_addAux(app.ui.auxid, app.ui.width, app.ui.maxheight,
               app.ui.spacing.samples, &app.con);

    // Run forward kinematics to initialise state
    mj_forward(app.sim_model, app.sim_data);

    std::cout << "Simulation started: nbody=" << app.sim_model->nbody
              << " nv=" << app.sim_model->nv
              << " timestep=" << app.sim_model->opt.timestep
              << " iterations=" << app.sim_model->opt.iterations << "\n";
}

// ---------------------------------------------------------------------------
// ExitSimulation
// ---------------------------------------------------------------------------

void ExitSimulation(AppState& app) {
    if (app.sim_data)  { mj_deleteData(app.sim_data);   app.sim_data  = nullptr; }
    if (app.sim_model) { mj_deleteModel(app.sim_model); app.sim_model = nullptr; }
    if (app.sim_spec)  { mj_deleteSpec(app.sim_spec);   app.sim_spec  = nullptr; }

    // Remove profiling/filter callbacks
    mjcb_time          = nullptr;
    mjcb_contactfilter = nullptr;

    // Rebuild render context for build stage (must use build_model so that
    // makeBuiltin() creates geometry VBOs for box/sphere/arrow rendering)
    mjv_freeScene(&app.scn);
    mjr_freeContext(&app.con);

    mjv_defaultScene(&app.scn);
    mjr_defaultContext(&app.con);
    mjv_makeScene(nullptr, &app.scn, kMaxSceneGeoms);
    mjr_makeContext(app.build_model, &app.con, mjFONTSCALE_150);

    mjui_resize(&app.ui, &app.con);
    mjr_addAux(app.ui.auxid, app.ui.width, app.ui.maxheight,
               app.ui.spacing.samples, &app.con);

    app.mode = AppMode::BUILD;
    std::cout << "Returned to build mode\n";
}

// ---------------------------------------------------------------------------
// StepSimulation
//
// Advances physics by at most one display frame (1/60 s) of simulation time.
// Hard cap: never run more than kMaxStepsPerFrame mj_step() calls per display
// frame — this prevents the render loop from stalling when physics is slow.
// The real-time ratio (g_rt_ratio) is updated so the profiler can report it.
// ---------------------------------------------------------------------------

static constexpr int kMaxStepsPerFrame = 16;  // cap: never run > 16 steps/frame

void StepSimulation(AppState& app) {
    if (!app.sim_model || !app.sim_data) return;

    auto wall_start = std::chrono::steady_clock::now();

    mjtNum simstart = app.sim_data->time;
    double target_advance = 1.0 / 60.0;
    int steps = 0;

    while ((app.sim_data->time - simstart) < target_advance &&
           steps < kMaxStepsPerFrame) {
        mj_step(app.sim_model, app.sim_data);
        ++steps;
    }

    double wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();

    // Real-time ratio: how much simulated time we advanced vs wall time spent.
    // 1.0 = real-time; < 1.0 = slower than real-time.
    if (wall_ms > 0.01) {
        double phys_ms = (app.sim_data->time - simstart) * 1000.0;
        g_rt_ratio = 0.9 * g_rt_ratio + 0.1 * (phys_ms / wall_ms);  // EMA
    }
}

// ---------------------------------------------------------------------------
// RenderProfilerOverlay
//
// Minimal one-liner always shown in sim mode (bottom-right corner).
// Full breakdown shown when show_profiler = true (P key).
// ---------------------------------------------------------------------------

void RenderProfilerOverlay(const AppState& app, mjrRect viewport) {
    if (!app.sim_model || !app.sim_data) return;

    const mjData*  d = app.sim_data;
    const mjModel* m = app.sim_model;

    // Helper: ms per step for a given timer slot
    auto timer_ms = [&](int slot) -> double {
        if (d->timer[slot].number <= 0) return 0.0;
        return d->timer[slot].duration / d->timer[slot].number;
    };

    double step_ms  = timer_ms(mjTIMER_STEP);
    double col_ms   = timer_ms(mjTIMER_POS_COLLISION);
    double sol_ms   = timer_ms(mjTIMER_CONSTRAINT);
    double kin_ms   = timer_ms(mjTIMER_POS_KINEMATICS);
    double inert_ms = timer_ms(mjTIMER_POS_INERTIA);

    // Warn when step > 16ms (can't keep up with 60 Hz display)
    const char* perf_warn = (step_ms > 16.0) ? " !! SLOW" : "";
    const char* rec_tag   = app.is_recording ? " [REC]" : "";

    if (app.show_profiler) {
        // Full breakdown
        char buf[768];
        std::snprintf(buf, sizeof(buf),
            "--- Profiler -------------------\n"
            "Step total : %6.2f ms%s\n"
            "  Collision: %6.2f ms  (%3.0f%%)\n"
            "  Solver   : %6.2f ms  (%3.0f%%)\n"
            "  Kinematics:%6.2f ms  (%3.0f%%)\n"
            "  Inertia  : %6.2f ms  (%3.0f%%)\n"
            "Contacts   : %d\n"
            "DOF (nv)   : %lld    Bodies: %lld\n"
            "RT ratio   : %.2fx%s\n"
            "Arena/Stack: %llu / %llu\n"
            "Recording  : %s\n"
            "--------------------------------",
            step_ms, perf_warn,
            col_ms,   step_ms > 0 ? 100.0 * col_ms   / step_ms : 0.0,
            sol_ms,   step_ms > 0 ? 100.0 * sol_ms   / step_ms : 0.0,
            kin_ms,   step_ms > 0 ? 100.0 * kin_ms   / step_ms : 0.0,
            inert_ms, step_ms > 0 ? 100.0 * inert_ms / step_ms : 0.0,
            d->ncon,
            (long long)m->nv, (long long)m->nbody,
            g_rt_ratio, perf_warn,
            (unsigned long long)d->maxuse_arena, (unsigned long long)d->maxuse_stack,
            app.is_recording ? app.record_path : "OFF");
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buf, NULL,
                    const_cast<mjrContext*>(&app.con));
    } else {
        // Compact one-liner (always visible in sim mode)
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "Step:%.1fms  Con:%d  RT:%.2fx%s%s  P=profiler  R=exit",
            step_ms, d->ncon, g_rt_ratio, perf_warn, rec_tag);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buf, NULL,
                    const_cast<mjrContext*>(&app.con));
    }
}

// ---------------------------------------------------------------------------
// StartRecording
// ---------------------------------------------------------------------------

bool StartRecording(AppState& app, const char* path) {
    if (app.is_recording) {
        std::cerr << "[Record] Already recording — stop first\n";
        return false;
    }
    if (!path || path[0] == '\0') {
        std::cerr << "[Record] Empty output path\n";
        return false;
    }

    // Determine framebuffer size from GLFW window
    int w = 0, h = 0;
    if (app.window) {
        glfwGetFramebufferSize(app.window, &w, &h);
    }
    if (w <= 0 || h <= 0) {
        std::cerr << "[Record] Invalid framebuffer size " << w << "x" << h << "\n";
        return false;
    }

    // Build ffmpeg command:
    //  -vf vflip  — OpenGL returns rows bottom→top; flip to top→bottom for video
    //  -crf 22    — quality (lower = better, 22 is a reasonable default)
    char cmd[1024];
    std::snprintf(cmd, sizeof(cmd),
        "ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size %dx%d "
        "-framerate 60 -i pipe:0 -vf vflip -c:v libx264 -preset fast "
        "-crf 22 \"%s\" 2>nul",
        w, h, path);

    FILE* pipe = popen(cmd, "wb");
    if (!pipe) {
        std::cerr << "[Record] popen(ffmpeg) failed — is ffmpeg installed?\n";
        return false;
    }

    // Allocate pixel buffer (RGB, no depth)
    unsigned char* pixels = static_cast<unsigned char*>(std::malloc(3 * w * h));
    if (!pixels) {
        pclose(pipe);
        std::cerr << "[Record] malloc failed for pixel buffer\n";
        return false;
    }

    app.record_pipe   = pipe;
    app.record_pixels = pixels;
    app.record_width  = w;
    app.record_height = h;
    app.is_recording  = true;
    std::snprintf(app.record_path, sizeof(app.record_path), "%s", path);

    std::cout << "[Record] Started recording " << w << "x" << h
              << " → " << path << "\n";
    return true;
}

// ---------------------------------------------------------------------------
// StopRecording
// ---------------------------------------------------------------------------

void StopRecording(AppState& app) {
    if (!app.is_recording) return;

    app.is_recording = false;

    if (app.record_pipe) {
        pclose(app.record_pipe);
        app.record_pipe = nullptr;
        std::cout << "[Record] Stopped recording → " << app.record_path << "\n";
    }
    if (app.record_pixels) {
        std::free(app.record_pixels);
        app.record_pixels = nullptr;
    }
    app.record_width  = 0;
    app.record_height = 0;
    app.record_path[0] = '\0';
}

// ---------------------------------------------------------------------------
// ToggleRecording — F9 key
// ---------------------------------------------------------------------------

void ToggleRecording(AppState& app) {
    if (app.is_recording) {
        StopRecording(app);
    } else {
        // Generate timestamped filename in current directory
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        char ts[32];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
        char path[512];
        std::snprintf(path, sizeof(path), "chainmaker_%s.mp4", ts);
        StartRecording(app, path);
    }
}

// ---------------------------------------------------------------------------
// CaptureRecordingFrame
// ---------------------------------------------------------------------------

void CaptureRecordingFrame(AppState& app, mjrRect viewport) {
    if (!app.is_recording || !app.record_pipe || !app.record_pixels) return;

    // Re-check if framebuffer size has changed (e.g., window resized)
    int w = viewport.width;
    int h = viewport.height;
    if (w != app.record_width || h != app.record_height) {
        // Size changed — stop recording gracefully
        std::cerr << "[Record] Framebuffer resized; stopping recording\n";
        StopRecording(app);
        return;
    }

    // Read RGB pixels from current (back) framebuffer.
    // mjr_readPixels calls glReadPixels — works on the back buffer before swap.
    mjr_readPixels(app.record_pixels, nullptr, viewport,
                   const_cast<mjrContext*>(&app.con));

    // Write raw RGB frame to ffmpeg stdin
    std::size_t n = static_cast<std::size_t>(w * h * 3);
    if (std::fwrite(app.record_pixels, 1, n, app.record_pipe) != n) {
        std::cerr << "[Record] fwrite failed — stopping recording\n";
        StopRecording(app);
    }
}

