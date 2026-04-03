#include "chainmaker_ipc.h"
#include "chainmaker.h"
#include "chainmaker_io.h"
#include "chainmaker_sim.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Windows socket includes
// ---------------------------------------------------------------------------
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "ws2_32.lib")
using SockHandle = SOCKET;
static constexpr SockHandle kInvalidSock = INVALID_SOCKET;
static void sock_close(SockHandle s) { closesocket(s); }
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <unistd.h>
using SockHandle = int;
static constexpr SockHandle kInvalidSock = -1;
static void sock_close(SockHandle s) { close(s); }
#endif

static constexpr int kIpcPort = 47832;

// ---------------------------------------------------------------------------
// Per-command struct — lives on the heap, shared between threads
// ---------------------------------------------------------------------------

struct IpcCommand {
    std::string              json_in;
    std::promise<std::string> promise;
};

// ---------------------------------------------------------------------------
// IpcServer — definition (opaque outside this file)
// ---------------------------------------------------------------------------

struct IpcServer {
    std::thread             thread;
    std::mutex              mutex;
    std::queue<std::shared_ptr<IpcCommand>> queue;
    SockHandle              server_sock = kInvalidSock;
    std::atomic<bool>       running{false};
};

// ---------------------------------------------------------------------------
// PPM screenshot writer (no external dependencies)
// ---------------------------------------------------------------------------

static bool WriteScreenshot(const char* path,
                            AppState& app, mjrRect viewport) {
    int w = viewport.width;
    int h = viewport.height;
    if (w <= 0 || h <= 0) return false;

    std::vector<unsigned char> pixels(w * h * 3);
    mjr_readPixels(pixels.data(), nullptr, viewport, &app.con);

    // OpenGL gives bottom-to-top rows; PPM/BMP expect top-to-bottom
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int row = h - 1; row >= 0; --row) {
        fwrite(pixels.data() + row * w * 3, 1, w * 3, f);
    }
    fclose(f);
    return true;
}

// ---------------------------------------------------------------------------
// Direction string helpers
// ---------------------------------------------------------------------------

static SpawnFace DirFromString(const std::string& s, bool& ok) {
    ok = true;
    if (s == "POS_X" || s == "+X") return FACE_POS_X;
    if (s == "NEG_X" || s == "-X") return FACE_NEG_X;
    if (s == "POS_Y" || s == "+Y") return FACE_POS_Y;
    if (s == "NEG_Y" || s == "-Y") return FACE_NEG_Y;
    if (s == "POS_Z" || s == "+Z") return FACE_POS_Z;
    if (s == "NEG_Z" || s == "-Z") return FACE_NEG_Z;
    ok = false;
    return FACE_POS_X;
}

static const char* PlaceResultStr(PlaceResult r) {
    switch (r) {
        case PlaceResult::SUCCESS:                         return "SUCCESS";
        case PlaceResult::NO_ACTIVE_CHAIN:                 return "NO_ACTIVE_CHAIN";
        case PlaceResult::TARGET_OCCUPIED_INCOMPATIBLE:    return "TARGET_OCCUPIED_INCOMPATIBLE";
        case PlaceResult::WOULD_REVERSE:                   return "WOULD_REVERSE";
        case PlaceResult::HEAD_WOULD_BECOME_INVALID_TURN:  return "HEAD_WOULD_BECOME_INVALID_TURN";
    }
    return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// State snapshot for get_state command
// ---------------------------------------------------------------------------

static json BuildStateJson(const AppState& app) {
    json s;
    switch (app.mode) {
        case AppMode::BUILD:          s["mode"] = "BUILD";          break;
        case AppMode::SIMULATE:       s["mode"] = "SIMULATE";       break;
        case AppMode::NEW_CHAIN_PICK: s["mode"] = "NEW_CHAIN_PICK"; break;
    }
    s["nchains"]      = (int)app.world.chains.size();
    s["active_chain"] = app.world.active_chain_id;
    s["ngrid"]        = (int)app.world.grid.size();
    s["bead_size"]    = app.world.bead_size;
    s["gap_ratio"]    = app.world.gap_ratio;

    const Chain* active = app.world.ActiveChain();
    if (active) {
        s["nblocks"]  = (int)active->blocks.size();
        if (!active->Empty()) {
            IVec3 h = active->Head();
            s["head"] = {{"x", h.x}, {"y", h.y}, {"z", h.z}};

            // Ghost position (accounts for intersection skip and turn-block blocking)
            IVec3 offset    = FaceToOffset(active->head_direction);
            IVec3 ghost_pos = h + offset;
            bool ghost_blocked = false;
            while (app.world.grid.count(ghost_pos)) {
                const GridCell& cell = app.world.grid.at(ghost_pos);
                if (cell.is_turn) { ghost_blocked = true; break; }
                ghost_pos = ghost_pos + offset;
            }
            if (ghost_blocked) {
                s["ghost"] = nullptr;
            } else {
                s["ghost"] = {{"x", ghost_pos.x}, {"y", ghost_pos.y},
                              {"z", ghost_pos.z}};
            }
        }
        s["direction"] = FaceName(active->head_direction);
    } else {
        s["nblocks"]  = 0;
    }

    // If simulating, expose physics sizes
    if (app.mode == AppMode::SIMULATE && app.sim_model) {
        s["nbody"] = app.sim_model->nbody;
        s["nv"]    = app.sim_model->nv;
    }

    // Camera state (always available)
    s["azimuth"]    = app.cam.azimuth;
    s["elevation"]  = app.cam.elevation;
    s["distance"]   = app.cam.distance;
    s["weld_level"] = static_cast<int>(app.world.weld_level);
    s["root_fixed"] = app.world.root_fixed;

    return s;
}

// ---------------------------------------------------------------------------
// Command execution — runs on the main GL thread
// ---------------------------------------------------------------------------

static json ExecuteCommand(const json& cmd, AppState& app, mjrRect viewport) {
    json resp;
    resp["ok"] = false;  // default; each handler sets true on success

    std::string command;
    try { command = cmd.at("cmd").get<std::string>(); }
    catch (...) { resp["error"] = "missing 'cmd' field"; return resp; }

    // ---- place_block -------------------------------------------------------
    if (command == "place_block") {
        if (app.mode != AppMode::BUILD) {
            resp["error"] = "not in BUILD mode";
            return resp;
        }
        PlaceResult r = PlaceBlock(app.world);
        resp["ok"]     = (r == PlaceResult::SUCCESS);
        resp["result"] = PlaceResultStr(r);
        const Chain* c = app.world.ActiveChain();
        resp["nblocks"] = c ? (int)c->blocks.size() : 0;
        return resp;
    }

    // ---- delete_block ------------------------------------------------------
    if (command == "delete_block") {
        if (app.mode != AppMode::BUILD) {
            resp["error"] = "not in BUILD mode";
            return resp;
        }
        DeleteLastBlock(app.world);
        resp["ok"] = true;
        const Chain* c = app.world.ActiveChain();
        resp["nblocks"] = c ? (int)c->blocks.size() : 0;
        return resp;
    }

    // ---- set_direction ------------------------------------------------------
    if (command == "set_direction") {
        std::string dir;
        try { dir = cmd.at("dir").get<std::string>(); }
        catch (...) { resp["error"] = "missing 'dir' field"; return resp; }
        bool ok = false;
        SpawnFace face = DirFromString(dir, ok);
        if (!ok) { resp["error"] = "unknown direction: " + dir; return resp; }
        SetDirection(app.world, face);
        resp["ok"] = true;
        return resp;
    }

    // ---- get_state ----------------------------------------------------------
    if (command == "get_state") {
        resp = BuildStateJson(app);
        resp["ok"] = true;
        return resp;
    }

    // ---- switch_chain -------------------------------------------------------
    if (command == "switch_chain") {
        int id = -1;
        try { id = cmd.at("id").get<int>(); }
        catch (...) { resp["error"] = "missing 'id' field"; return resp; }
        if (id < 0 || id >= (int)app.world.chains.size()) {
            resp["error"] = "invalid chain id";
            return resp;
        }
        SwitchChain(app.world, id);
        resp["ok"] = true;
        return resp;
    }

    // ---- new_chain_mode -----------------------------------------------------
    if (command == "new_chain_mode") {
        if (app.mode != AppMode::BUILD) {
            resp["error"] = "not in BUILD mode";
            return resp;
        }
        app.mode = AppMode::NEW_CHAIN_PICK;
        app.junction_picked = false;
        resp["ok"] = true;
        return resp;
    }

    // ---- start_chain_at -----------------------------------------------------
    if (command == "start_chain_at") {
        int x = 0, y = 0, z = 0;
        try {
            x = cmd.at("x").get<int>();
            y = cmd.at("y").get<int>();
            z = cmd.at("z").get<int>();
        } catch (...) { resp["error"] = "missing x/y/z"; return resp; }
        IVec3 pos{x, y, z};
        if (app.world.grid.find(pos) == app.world.grid.end()) {
            resp["error"] = "no block at that position";
            return resp;
        }
        app.mode = AppMode::NEW_CHAIN_PICK;
        bool started = StartNewChainFromBlock(app, pos);
        if (started) {
            resp["ok"]       = true;
            resp["chain_id"] = app.world.active_chain_id;
        } else {
            resp["error"] = "StartNewChainFromBlock failed";
        }
        return resp;
    }

    // ---- cancel_pick --------------------------------------------------------
    if (command == "cancel_pick") {
        if (app.mode == AppMode::NEW_CHAIN_PICK) {
            app.mode = AppMode::BUILD;
            app.junction_picked = false;
        }
        resp["ok"] = true;
        return resp;
    }

    // ---- adjust_gap ---------------------------------------------------------
    if (command == "adjust_gap") {
        double delta = 0;
        try { delta = cmd.at("delta").get<double>(); }
        catch (...) { resp["error"] = "missing 'delta' field"; return resp; }
        AdjustGap(app.world, delta);
        resp["ok"]        = true;
        resp["gap_ratio"] = app.world.gap_ratio;
        return resp;
    }

    // ---- set_bead_size ------------------------------------------------------
    if (command == "set_bead_size") {
        double size = 0;
        try { size = cmd.at("size").get<double>(); }
        catch (...) { resp["error"] = "missing 'size' field"; return resp; }
        if (size < 0.001 || size > 0.5) {
            resp["error"] = "bead_size out of range [0.001, 0.5]";
            return resp;
        }
        app.world.bead_size = size;
        resp["ok"]        = true;
        resp["bead_size"] = size;
        return resp;
    }

    // ---- screenshot ---------------------------------------------------------
    if (command == "screenshot") {
        std::string path;
        try { path = cmd.at("path").get<std::string>(); }
        catch (...) {
            // default path
            path = "chainmaker_screenshot.ppm";
        }
        bool ok = WriteScreenshot(path.c_str(), app, viewport);
        resp["ok"]     = ok;
        resp["path"]   = path;
        resp["width"]  = viewport.width;
        resp["height"] = viewport.height;
        if (!ok) resp["error"] = "failed to write screenshot file";
        return resp;
    }

    // ---- enter_simulate -----------------------------------------------------
    if (command == "enter_simulate") {
        if (app.mode == AppMode::SIMULATE) {
            resp["ok"]    = true;
            resp["note"]  = "already simulating";
            return resp;
        }
        EnterSimulation(app);
        resp["ok"] = (app.mode == AppMode::SIMULATE);
        if (app.sim_model) {
            resp["nbody"] = app.sim_model->nbody;
            resp["nv"]    = app.sim_model->nv;
        }
        if (!resp["ok"].get<bool>()) resp["error"] = "compile/enter failed";
        return resp;
    }

    // ---- exit_simulate ------------------------------------------------------
    if (command == "exit_simulate") {
        if (app.mode != AppMode::SIMULATE) {
            resp["ok"]   = true;
            resp["note"] = "not simulating";
            return resp;
        }
        ExitSimulation(app);
        resp["ok"] = true;
        return resp;
    }

    // ---- get_profiler -------------------------------------------------------
    // Returns per-step timer values from d->timer[]. Only valid in SIMULATE mode.
    // Each field is average milliseconds per step for that timer category.
    if (command == "get_profiler") {
        if (app.mode != AppMode::SIMULATE || !app.sim_data || !app.sim_model) {
            resp["ok"]    = false;
            resp["error"] = "not in simulation mode";
            return resp;
        }
        const mjData*  d = app.sim_data;
        const mjModel* m = app.sim_model;
        auto timer_ms = [&](int slot) -> double {
            if (d->timer[slot].number <= 0) return 0.0;
            return d->timer[slot].duration / d->timer[slot].number;
        };
        resp["ok"]          = true;
        resp["step_ms"]     = timer_ms(mjTIMER_STEP);
        resp["collision_ms"]= timer_ms(mjTIMER_POS_COLLISION);
        resp["solver_ms"]   = timer_ms(mjTIMER_CONSTRAINT);
        resp["kinematics_ms"]= timer_ms(mjTIMER_POS_KINEMATICS);
        resp["inertia_ms"]  = timer_ms(mjTIMER_POS_INERTIA);
        resp["ncon"]        = d->ncon;
        resp["neq"]         = (long long)m->neq;
        resp["nbody"]       = (long long)m->nbody;
        resp["nv"]          = (long long)m->nv;
        resp["iterations"]  = m->opt.iterations;
        {
            int p = static_cast<int>(app.world.sim_preset);
            const char* labels[kNumSimPresets] = {"Accurate", "Balanced", "Fast"};
            resp["sim_preset"]       = p;
            resp["sim_preset_label"] = (p >= 0 && p < kNumSimPresets) ? labels[p] : "Unknown";
        }
        resp["weld_level"] = static_cast<int>(app.world.weld_level);
        return resp;
    }

    // ---- set_sim_preset -----------------------------------------------------
    // Sets the simulation speed/accuracy preset (0=Accurate, 1=Balanced, 2=Fast).
    // If simulation is running, applies the preset immediately to the live model.
    if (command == "set_sim_preset") {
        int p = -1;
        try { p = cmd.at("preset").get<int>(); }
        catch (...) { resp["error"] = "missing 'preset' field (int 0-2)"; return resp; }
        if (p < 0 || p >= kNumSimPresets) {
            resp["ok"]    = false;
            resp["error"] = "preset must be 0 (Accurate), 1 (Balanced), or 2 (Fast)";
            return resp;
        }
        app.world.sim_preset = static_cast<SimPreset>(p);
        if (app.mode == AppMode::SIMULATE) {
            ApplySimPreset(app);
        }
        resp["ok"]     = true;
        resp["preset"] = p;
        return resp;
    }

    // ---- set_root_fixed -----------------------------------------------------
    // Toggles whether chain-0's root body has a free joint (root_fixed=false)
    // or is anchored to worldbody (root_fixed=true, default).
    // Takes effect on next Enter Simulation.
    if (command == "set_root_fixed") {
        bool fixed = true;
        try { fixed = cmd.at("fixed").get<bool>(); }
        catch (...) { resp["error"] = "missing 'fixed' field (bool)"; return resp; }
        app.world.root_fixed = fixed;
        resp["ok"]    = true;
        resp["fixed"] = fixed;
        return resp;
    }

    // ---- set_weld_level -----------------------------------------------------
    // Sets the straight-run weld policy (0=None, 2=Full).
    // Takes effect on next CompileWorld call (i.e. next Enter Simulation).
    if (command == "set_weld_level") {
        int lv = -1;
        try { lv = cmd.at("level").get<int>(); }
        catch (...) { resp["error"] = "missing 'level' field (int 0 or 2)"; return resp; }
        if (lv != 0 && lv != 2) {
            resp["ok"]    = false;
            resp["error"] = "level must be 0 (None) or 2 (Full)";
            return resp;
        }
        app.world.weld_level = static_cast<WeldLevel>(lv);
        resp["ok"]    = true;
        resp["level"] = lv;
        return resp;
    }

    // ---- save ---------------------------------------------------------------
    if (command == "save") {
        std::string path;
        try { path = cmd.at("path").get<std::string>(); }
        catch (...) { resp["error"] = "missing 'path' field"; return resp; }
        bool ok = SaveWorldToJSON(app.world, path.c_str());
        resp["ok"] = ok;
        if (!ok) resp["error"] = "save failed";
        return resp;
    }

    // ---- load ---------------------------------------------------------------
    if (command == "load") {
        std::string path;
        try { path = cmd.at("path").get<std::string>(); }
        catch (...) { resp["error"] = "missing 'path' field"; return resp; }
        ChainWorld new_world;
        bool ok = LoadWorldFromJSON(new_world, path.c_str());
        if (ok) {
            app.world = std::move(new_world);
        }
        resp["ok"] = ok;
        if (!ok) resp["error"] = "load failed";
        return resp;
    }

    // ---- reset --------------------------------------------------------------
    if (command == "reset") {
        if (app.mode == AppMode::SIMULATE) ExitSimulation(app);
        CreateFirstChain(app.world);
        resp["ok"] = true;
        return resp;
    }

    // ---- move_camera -------------------------------------------------------
    // Directly calls mjv_moveCamera on the main GL thread.
    // Parameters:
    //   action: "rotate_v" | "rotate_h" | "move_v" | "move_h" | "zoom"
    //   dx, dy: normalised delta (pixels / window_height)
    if (command == "move_camera") {
        std::string act_str;
        double dx = 0.0, dy = 0.0;
        try {
            act_str = cmd.at("action").get<std::string>();
            dx = cmd.value("dx", 0.0);
            dy = cmd.value("dy", 0.0);
        } catch (...) {
            resp["error"] = "missing 'action' field";
            return resp;
        }

        mjtMouse act;
        if      (act_str == "rotate_v") act = mjMOUSE_ROTATE_V;
        else if (act_str == "rotate_h") act = mjMOUSE_ROTATE_H;
        else if (act_str == "move_v")   act = mjMOUSE_MOVE_V;
        else if (act_str == "move_h")   act = mjMOUSE_MOVE_H;
        else if (act_str == "zoom")     act = mjMOUSE_ZOOM;
        else { resp["error"] = "unknown action: " + act_str; return resp; }

        mjModel* m = (app.mode == AppMode::SIMULATE && app.sim_model)
                     ? app.sim_model : app.build_model;
        mjv_moveCamera(m, act, dx, dy, &app.scn, &app.cam);

        // Return current camera state for verification
        resp["ok"]       = true;
        resp["azimuth"]  = app.cam.azimuth;
        resp["elevation"]= app.cam.elevation;
        resp["distance"] = app.cam.distance;
        return resp;
    }

    // ---- quit ---------------------------------------------------------------
    if (command == "quit") {
        if (app.window) glfwSetWindowShouldClose(app.window, 1);
        resp["ok"] = true;
        return resp;
    }

    // ---- start_recording ----------------------------------------------------
    if (command == "start_recording") {
        if (app.is_recording) {
            resp["ok"]   = false;
            resp["error"] = "already recording";
            return resp;
        }
        std::string path = cmd.value("path", "");
        if (path.empty()) {
            // Generate timestamped name
            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            char ts[32];
            std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
            path = std::string("chainmaker_") + ts + ".mp4";
        }
        bool ok = StartRecording(app, path.c_str());
        resp["ok"]   = ok;
        resp["path"] = path;
        if (!ok) resp["error"] = "StartRecording failed (is ffmpeg installed?)";
        return resp;
    }

    // ---- stop_recording -----------------------------------------------------
    if (command == "stop_recording") {
        bool was_recording = app.is_recording;
        StopRecording(app);
        resp["ok"]   = true;
        resp["was_recording"] = was_recording;
        return resp;
    }

    // ---- get_recording_status -----------------------------------------------
    if (command == "get_recording_status") {
        resp["ok"]           = true;
        resp["is_recording"] = app.is_recording;
        if (app.is_recording) {
            resp["path"]   = std::string(app.record_path);
            resp["width"]  = app.record_width;
            resp["height"] = app.record_height;
        }
        return resp;
    }

    // ---- unknown ------------------------------------------------------------
    resp["error"] = "unknown command: " + command;
    return resp;
}

// ---------------------------------------------------------------------------
// Background thread — accept + recv loop
// ---------------------------------------------------------------------------

static void IpcThread(IpcServer* srv) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[IPC] WSAStartup failed\n";
        return;
    }
#endif

    srv->server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv->server_sock == kInvalidSock) {
        std::cerr << "[IPC] socket() failed\n";
#ifdef _WIN32
        WSACleanup();
#endif
        return;
    }

    // Allow immediate reuse of the port after restart
    int reuse = 1;
    setsockopt(srv->server_sock, SOL_SOCKET, SO_REUSEADDR,
               (const char*)&reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(kIpcPort);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // localhost only

    if (bind(srv->server_sock, (sockaddr*)&addr, sizeof(addr)) != 0) {
        std::cerr << "[IPC] bind() failed on port " << kIpcPort << "\n";
        sock_close(srv->server_sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return;
    }

    listen(srv->server_sock, 1);
    std::cout << "[IPC] Test server listening on 127.0.0.1:" << kIpcPort << "\n";

    while (srv->running) {
        // Non-blocking accept with timeout so we can check srv->running
#ifdef _WIN32
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(srv->server_sock, &fds);
        timeval tv{0, 200000};  // 200ms
        int sel = select(0, &fds, nullptr, nullptr, &tv);
        if (sel <= 0) continue;
#endif

        sockaddr_in client_addr{};
        int addrlen = sizeof(client_addr);
        SockHandle client = accept(srv->server_sock,
                                   (sockaddr*)&client_addr, &addrlen);
        if (client == kInvalidSock) continue;

        std::cout << "[IPC] Client connected\n";

        // Read newline-delimited JSON commands from this client
        std::string buf;
        char tmp[4096];

        while (srv->running) {
            int n = recv(client, tmp, sizeof(tmp) - 1, 0);
            if (n <= 0) break;
            tmp[n] = '\0';
            buf += tmp;

            // Process complete lines (commands delimited by '\n')
            size_t pos;
            while ((pos = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, pos);
                buf.erase(0, pos + 1);
                if (line.empty()) continue;

                // Create command + future
                auto cmd_ptr = std::make_shared<IpcCommand>();
                cmd_ptr->json_in = line;
                auto future = cmd_ptr->promise.get_future();

                {
                    std::lock_guard<std::mutex> lk(srv->mutex);
                    srv->queue.push(cmd_ptr);
                }

                // Block this thread until the main thread processes the command
                std::string response;
                try {
                    response = future.get();
                } catch (...) {
                    response = R"({"ok":false,"error":"internal error"})";
                }

                response += "\n";
                send(client, response.c_str(), (int)response.size(), 0);
            }
        }

        sock_close(client);
        std::cout << "[IPC] Client disconnected\n";
    }

    sock_close(srv->server_sock);
    srv->server_sock = kInvalidSock;
#ifdef _WIN32
    WSACleanup();
#endif
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void IpcServerStart(AppState& app) {
    if (app.ipc) return;  // already running
    auto* srv = new IpcServer();
    srv->running = true;
    srv->thread  = std::thread(IpcThread, srv);
    app.ipc = srv;
}

void IpcServerStop(AppState& app) {
    if (!app.ipc) return;
    app.ipc->running = false;
    // Unblock accept() by closing the server socket
    if (app.ipc->server_sock != kInvalidSock) {
        sock_close(app.ipc->server_sock);
        app.ipc->server_sock = kInvalidSock;
    }
    if (app.ipc->thread.joinable()) {
        app.ipc->thread.join();
    }
    delete app.ipc;
    app.ipc = nullptr;
}

void IpcProcessCommands(AppState& app, mjrRect viewport) {
    if (!app.ipc) return;

    // Pop at most one command per frame to keep the render loop responsive
    std::shared_ptr<IpcCommand> cmd_ptr;
    {
        std::lock_guard<std::mutex> lk(app.ipc->mutex);
        if (app.ipc->queue.empty()) return;
        cmd_ptr = app.ipc->queue.front();
        app.ipc->queue.pop();
    }

    // Parse and execute
    json response;
    try {
        json in = json::parse(cmd_ptr->json_in);
        response = ExecuteCommand(in, app, viewport);
    } catch (const std::exception& e) {
        response["ok"]    = false;
        response["error"] = std::string("parse error: ") + e.what();
    }

    cmd_ptr->promise.set_value(response.dump());
}
