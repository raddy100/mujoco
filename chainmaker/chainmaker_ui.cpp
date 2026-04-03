#include "chainmaker_ui.h"
#include "chainmaker_io.h"
#include "chainmaker_sim.h"
#include <cstdio>
#include <cstring>
#include <iostream>

// The global app state is owned by main.cc
extern AppState g_app;

// ---------------------------------------------------------------------------
// Button label constants
// ---------------------------------------------------------------------------

static const char* kBtnPlace    = "Place Block (C)";
static const char* kBtnDelete   = "Delete Block (Del)";
static const char* kBtnNewChain = "New Chain (N)";
static const char* kBtnSave     = "Save (Ctrl+S)";
static const char* kBtnLoad     = "Load (Ctrl+L)";
static const char* kBtnSimulate = "Simulate (P)";

// Sim Speed button prefix — used for identification in HandleUIClick
static const char* kBtnSimSpeedPrefix = "Sim Speed: ";

static const char* kSimSpeedLabels[kNumSimPresets] = {
    "Sim Speed: Accurate",  // 0 — Newton + box
    "Sim Speed: Balanced",  // 1 — CG + box
    "Sim Speed: Fast",      // 2 — CG + sphere
};

// ---------------------------------------------------------------------------
// BuildUI
// ---------------------------------------------------------------------------

void BuildUI(AppState& app) {
    std::memset(&app.uistate, 0, sizeof(mjuiState));
    std::memset(&app.ui, 0, sizeof(mjUI));

    app.ui.spacing  = mjui_themeSpacing(1);
    app.ui.color    = mjui_themeColor(1);
    app.ui.predicate = nullptr;
    app.ui.rectid   = 1;
    app.ui.auxid    = 0;
    app.ui.radiocol = 1;

    // Blue button tint
    app.ui.color.button[0] = 0.15f;
    app.ui.color.button[1] = 0.55f;
    app.ui.color.button[2] = 0.95f;

    mjuiDef def[40];
    int di = 0;

    auto clearDef = [&]() { std::memset(&def[di], 0, sizeof(mjuiDef)); };

    // Section header
    clearDef();
    def[di].type  = mjITEM_SECTION;
    std::snprintf(def[di].name, sizeof(def[di].name), "Chain Controls");
    def[di].state = 1;
    di++;

    // Action buttons
    auto addButton = [&](const char* label) {
        clearDef();
        def[di].type  = mjITEM_BUTTON;
        std::snprintf(def[di].name, sizeof(def[di].name), "%s", label);
        def[di].state = 2;
        di++;
    };

    addButton(kBtnPlace);
    addButton(kBtnDelete);
    addButton(kBtnNewChain);

    // Separator
    clearDef(); def[di].type = mjITEM_SEPARATOR; di++;

    // Bead size slider
    clearDef();
    def[di].type  = mjITEM_SLIDERNUM;
    std::snprintf(def[di].name, sizeof(def[di].name), "Bead Size");
    def[di].state = 2;
    def[di].pdata = &app.world.bead_size;
    std::snprintf(def[di].other, sizeof(def[di].other), "0.01 0.20");
    di++;

    // Gap ratio slider
    clearDef();
    def[di].type  = mjITEM_SLIDERNUM;
    std::snprintf(def[di].name, sizeof(def[di].name), "Gap Ratio");
    def[di].state = 2;
    def[di].pdata = &app.world.gap_ratio;
    std::snprintf(def[di].other, sizeof(def[di].other), "0.00 0.50");
    di++;

    // Sim speed preset — cycling button: click to cycle Accurate → Balanced → Fast → …
    // Label includes current preset name so user always sees active setting.
    {
        int p = static_cast<int>(app.world.sim_preset);
        if (p < 0 || p >= kNumSimPresets) p = 0;
        addButton(kSimSpeedLabels[p]);
    }

    // Separator
    clearDef(); def[di].type = mjITEM_SEPARATOR; di++;

    addButton(kBtnSave);
    addButton(kBtnLoad);

    // Separator
    clearDef(); def[di].type = mjITEM_SEPARATOR; di++;

    addButton(kBtnSimulate);

    // Terminator (required)
    clearDef();
    def[di].type = mjITEM_END;

    mjui_add(&app.ui, def);
    mjui_resize(&app.ui, &app.con);
    mjr_addAux(app.ui.auxid, app.ui.width, app.ui.maxheight,
               app.ui.spacing.samples, &app.con);
}

// ---------------------------------------------------------------------------
// SetupUIRect
// ---------------------------------------------------------------------------

void SetupUIRect(AppState& app, mjrRect viewport) {
    // ui.rectid = 1 is set in BuildUI — keep it, don't override.
    app.uistate.nrect = 2;

    int uiw = app.ui.width;
    if (uiw < app.ui_width) uiw = app.ui_width;

    // rect[0]: full framebuffer viewport (required by mjr_findRect / UiState coord mapping)
    app.uistate.rect[0] = {0, 0, viewport.width, viewport.height};
    // rect[1]: UI panel on the right edge
    app.uistate.rect[1] = {viewport.width - uiw, 0, uiw, viewport.height};
}

// ---------------------------------------------------------------------------
// HandleUIClick
// ---------------------------------------------------------------------------

void HandleUIClick(AppState& app, mjuiItem* item, int glfw_action) {
    if (glfw_action != GLFW_PRESS) return;
    if (item->type != mjITEM_BUTTON) return;

    // Sim Speed cycling button: advance to next preset, rebuild UI next frame
    if (std::strncmp(item->name, kBtnSimSpeedPrefix,
                     std::strlen(kBtnSimSpeedPrefix)) == 0) {
        int next = (static_cast<int>(app.world.sim_preset) + 1) % kNumSimPresets;
        app.world.sim_preset = static_cast<SimPreset>(next);
        if (app.mode == AppMode::SIMULATE) {
            ApplySimPreset(app);
        }
        app.needs_ui_rebuild = true;  // button label updates on next frame
        return;
    }

    if (std::strcmp(item->name, kBtnPlace) == 0) {
        PlaceBlock(app.world);
    } else if (std::strcmp(item->name, kBtnDelete) == 0) {
        DeleteLastBlock(app.world);
    } else if (std::strcmp(item->name, kBtnNewChain) == 0) {
        app.mode = AppMode::NEW_CHAIN_PICK;
        app.junction_picked = false;
    } else if (std::strcmp(item->name, kBtnSave) == 0) {
        app.io_mode = 1;
        app.io_filename[0] = '\0';
    } else if (std::strcmp(item->name, kBtnLoad) == 0) {
        app.io_mode = 2;
        app.io_filename[0] = '\0';
    } else if (std::strcmp(item->name, kBtnSimulate) == 0) {
        EnterSimulation(app);
    }
}

// ---------------------------------------------------------------------------
// HandleFileIOKey
// ---------------------------------------------------------------------------

void HandleFileIOKey(AppState& app, int key, int mods) {
    if (key == GLFW_KEY_ESCAPE) {
        app.io_mode = 0;
        return;
    }
    if (key == GLFW_KEY_ENTER) {
        if (app.io_mode == 1) {
            if (!SaveWorldToJSON(app.world, app.io_filename))
                std::cerr << "Save failed\n";
        } else if (app.io_mode == 2) {
            if (!LoadWorldFromJSON(app.world, app.io_filename))
                std::cerr << "Load failed\n";
        }
        app.io_mode = 0;
        return;
    }
    if (key == GLFW_KEY_BACKSPACE) {
        size_t len = std::strlen(app.io_filename);
        if (len > 0) app.io_filename[len - 1] = '\0';
        return;
    }
    char c = '\0';
    if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
        c = (mods & GLFW_MOD_SHIFT) ? ('A' + key - GLFW_KEY_A) : ('a' + key - GLFW_KEY_A);
    else if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9)
        c = '0' + key - GLFW_KEY_0;
    else if (key == GLFW_KEY_PERIOD)    c = '.';
    else if (key == GLFW_KEY_MINUS)     c = '-';
    else if (key == GLFW_KEY_SLASH)     c = '/';
    else if (key == GLFW_KEY_BACKSLASH) c = '\\';

    if (c != '\0') {
        size_t len = std::strlen(app.io_filename);
        if (len + 1 < sizeof(app.io_filename)) {
            app.io_filename[len]     = c;
            app.io_filename[len + 1] = '\0';
        }
    }
}

// ---------------------------------------------------------------------------
// KeyboardCallback
// ---------------------------------------------------------------------------

void KeyboardCallback(GLFWwindow* /*window*/, int key, int /*scancode*/,
                      int act, int mods) {
    AppState& app = g_app;

    // Update uistate for this keyboard event so mjui_event processes it correctly
    app.uistate.type    = (act == GLFW_RELEASE) ? mjEVENT_RELEASE : mjEVENT_KEY;
    app.uistate.key     = key;
    app.uistate.shift   = (mods & GLFW_MOD_SHIFT)   ? 1 : 0;
    app.uistate.control = (mods & GLFW_MOD_CONTROL) ? 1 : 0;
    app.uistate.alt     = (mods & GLFW_MOD_ALT)     ? 1 : 0;

    // Let UI consume keyboard events first
    if (mjui_event(&app.ui, &app.uistate, &app.con)) return;

    // File I/O capture mode
    if (app.io_mode != 0 && act == GLFW_PRESS) {
        HandleFileIOKey(app, key, mods);
        return;
    }

    if (act != GLFW_PRESS) return;

    // --- Simulation mode ---
    if (app.mode == AppMode::SIMULATE) {
        if (key == GLFW_KEY_R) ExitSimulation(app);
        if (key == GLFW_KEY_P) app.show_profiler = !app.show_profiler;
        if (key == GLFW_KEY_F9) ToggleRecording(app);
        return;
    }

    // --- New-chain-pick mode ---
    if (app.mode == AppMode::NEW_CHAIN_PICK) {
        if (key == GLFW_KEY_ESCAPE) {
            app.mode = AppMode::BUILD;
            app.junction_picked = false;
            app.block_hovered   = false;
            return;
        }
        // 1-9: cancel pick mode and switch to that chain
        if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
            int idx = key - GLFW_KEY_1;
            SwitchChain(app.world, idx);
            app.mode = AppMode::BUILD;
            app.junction_picked = false;
            app.block_hovered   = false;
            return;
        }
        return;
    }

    // --- Build mode ---
    if (app.mode == AppMode::BUILD) {
        switch (key) {
            // Place block
            case GLFW_KEY_C:
            case GLFW_KEY_SPACE:
                PlaceBlock(app.world);
                break;

            // Delete last block
            case GLFW_KEY_DELETE:
            case GLFW_KEY_BACKSPACE:
                DeleteLastBlock(app.world);
                break;

            // New chain
            case GLFW_KEY_N:
                app.mode = AppMode::NEW_CHAIN_PICK;
                app.junction_picked = false;
                app.block_hovered   = false;
                break;

            // Direction keys
            case GLFW_KEY_RIGHT: SetDirection(app.world, FACE_POS_X); break;
            case GLFW_KEY_LEFT:  SetDirection(app.world, FACE_NEG_X); break;
            case GLFW_KEY_UP:    SetDirection(app.world, FACE_POS_Y); break;
            case GLFW_KEY_DOWN:  SetDirection(app.world, FACE_NEG_Y); break;
            case GLFW_KEY_Z:     SetDirection(app.world, FACE_POS_Z); break;
            case GLFW_KEY_X:     SetDirection(app.world, FACE_NEG_Z); break;

            // Chain switch (1-9)
            case GLFW_KEY_1: SwitchChain(app.world, 0); break;
            case GLFW_KEY_2: SwitchChain(app.world, 1); break;
            case GLFW_KEY_3: SwitchChain(app.world, 2); break;
            case GLFW_KEY_4: SwitchChain(app.world, 3); break;
            case GLFW_KEY_5: SwitchChain(app.world, 4); break;
            case GLFW_KEY_6: SwitchChain(app.world, 5); break;
            case GLFW_KEY_7: SwitchChain(app.world, 6); break;
            case GLFW_KEY_8: SwitchChain(app.world, 7); break;
            case GLFW_KEY_9: SwitchChain(app.world, 8); break;

            // Gap adjustment
            case GLFW_KEY_EQUAL:
            case GLFW_KEY_KP_ADD:
                AdjustGap(app.world, +0.01);
                break;
            case GLFW_KEY_MINUS:
            case GLFW_KEY_KP_SUBTRACT:
                AdjustGap(app.world, -0.01);
                break;

            // Save / Load
            case GLFW_KEY_S:
                if (mods & GLFW_MOD_CONTROL) {
                    app.io_mode = 1;
                    app.io_filename[0] = '\0';
                }
                break;
            case GLFW_KEY_L:
                if (mods & GLFW_MOD_CONTROL) {
                    app.io_mode = 2;
                    app.io_filename[0] = '\0';
                }
                break;

            // Simulate
            case GLFW_KEY_P:
                EnterSimulation(app);
                break;

            default:
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// MouseButtonCallback
// ---------------------------------------------------------------------------

void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
    AppState& app = g_app;

    app.mouse_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    app.mouse_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    app.mouse_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);

    double cx, cy;
    glfwGetCursorPos(window, &cx, &cy);

    int fbw, fbh, winw, winh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &winw, &winh);

    // Scale cursor from window coords to framebuffer coords (handles HiDPI)
    double cx_fb = (winw > 0) ? cx * fbw / winw : cx;
    double cy_fb = (winh > 0) ? cy * fbh / winh : cy;

    app.mouse_lastx = cx_fb;
    app.mouse_lasty = cy_fb;

    // Translate GLFW button → mjtButton (required: mjui_event checks uistate.button)
    mjtButton mj_button = mjBUTTON_NONE;
    if (button == GLFW_MOUSE_BUTTON_LEFT)   mj_button = mjBUTTON_LEFT;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)  mj_button = mjBUTTON_RIGHT;
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE) mj_button = mjBUTTON_MIDDLE;

    app.uistate.type    = (act == GLFW_PRESS) ? mjEVENT_PRESS : mjEVENT_RELEASE;
    app.uistate.button  = mj_button;
    app.uistate.left    = app.mouse_left;
    app.uistate.right   = app.mouse_right;
    app.uistate.middle  = app.mouse_middle;
    app.uistate.dx      = cx_fb - app.uistate.x;
    app.uistate.dy      = (fbh - cy_fb) - app.uistate.y;
    app.uistate.x       = cx_fb;
    app.uistate.y       = fbh - cy_fb;   // flip Y for OpenGL origin
    app.uistate.shift   = (mods & GLFW_MOD_SHIFT)   ? 1 : 0;
    app.uistate.control = (mods & GLFW_MOD_CONTROL) ? 1 : 0;
    app.uistate.alt     = (mods & GLFW_MOD_ALT)     ? 1 : 0;

    SetupUIRect(app, {0, 0, fbw, fbh});

    // Determine which rect the cursor is in
    app.uistate.mouserect = mjr_findRect(
        (int)mju_round(cx_fb), (int)mju_round(app.uistate.y),
        app.uistate.nrect - 1, app.uistate.rect + 1) + 1;

    // Track drag rect (needed for slider dragging on mouse-move events)
    if (act == GLFW_PRESS) {
        if (app.uistate.mouserect) {
            app.uistate.dragbutton = mj_button;
            app.uistate.dragrect   = app.uistate.mouserect;
        }
    } else {
        // Release: will be cleared after mjui_event call below
    }

    // Dispatch to UI if event is for the UI rect
    bool for_ui = (app.uistate.dragrect == app.ui.rectid) ||
                  (app.uistate.dragrect == 0 &&
                   app.uistate.mouserect == app.ui.rectid);
    if (for_ui) {
        if (mjuiItem* changed = mjui_event(&app.ui, &app.uistate, &app.con)) {
            HandleUIClick(app, changed, act);
            if (act == GLFW_RELEASE) {
                app.uistate.dragrect   = 0;
                app.uistate.dragbutton = 0;
            }
            return;
        }
    }

    // Clear drag state on release (after UI has seen the event)
    if (act == GLFW_RELEASE) {
        app.uistate.dragrect   = 0;
        app.uistate.dragbutton = 0;
    }

    // Don't let world interact when cursor is in the UI panel
    if (app.uistate.mouserect == app.ui.rectid) return;

    // NEW_CHAIN_PICK: left-click picks a block
    if (app.mode == AppMode::NEW_CHAIN_PICK
        && act == GLFW_PRESS
        && button == GLFW_MOUSE_BUTTON_LEFT) {
        IVec3 hit;
        if (RayPickBlock(app, cx_fb, cy_fb, fbw, fbh, hit)) {
            StartNewChainFromBlock(app, hit);
        }
        return;
    }
}

// ---------------------------------------------------------------------------
// MouseMoveCallback
// ---------------------------------------------------------------------------

void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    AppState& app = g_app;

    int fbw, fbh, winw, winh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &winw, &winh);

    // Scale cursor from window coords to framebuffer coords
    double xfb = (winw > 0) ? xpos * fbw / winw : xpos;
    double yfb = (winh > 0) ? ypos * fbh / winh : ypos;
    double yfb_gl = fbh - yfb;   // flip Y for OpenGL origin

    double dx = xfb - app.mouse_lastx;
    double dy = yfb - app.mouse_lasty;
    app.mouse_lastx = xfb;
    app.mouse_lasty = yfb;

    // Hover tracking for NEW_CHAIN_PICK — runs even without mouse buttons held
    if (app.mode == AppMode::NEW_CHAIN_PICK && !app.junction_picked) {
        IVec3 hit;
        if (RayPickBlock(app, xfb, yfb, fbw, fbh, hit)) {
            app.hovered_block = hit;
            app.block_hovered = true;
        } else {
            app.block_hovered = false;
        }
    }

    if (!app.mouse_left && !app.mouse_middle && !app.mouse_right) return;

    // Update uistate for UI dragging (sliders etc.)
    app.uistate.dx   = xfb - app.uistate.x;
    app.uistate.dy   = yfb_gl - app.uistate.y;
    app.uistate.x    = xfb;
    app.uistate.y    = yfb_gl;
    app.uistate.type = mjEVENT_MOVE;

    SetupUIRect(app, {0, 0, fbw, fbh});
    app.uistate.mouserect = mjr_findRect(
        (int)mju_round(xfb), (int)mju_round(yfb_gl),
        app.uistate.nrect - 1, app.uistate.rect + 1) + 1;

    bool for_ui = (app.uistate.dragrect == app.ui.rectid) ||
                  (app.uistate.dragrect == 0 &&
                   app.uistate.mouserect == app.ui.rectid);
    if (for_ui) {
        if (mjui_event(&app.ui, &app.uistate, &app.con)) return;
    }

    // Don't move camera when cursor is in UI panel and no drag started outside it
    if (app.uistate.mouserect == app.ui.rectid && app.uistate.dragrect == 0) return;

    // Camera control
    mjtMouse action;
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    if (app.mouse_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else if (app.mouse_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else
        action = mjMOUSE_ZOOM;

    mjModel* m = (app.mode == AppMode::SIMULATE && app.sim_model)
                 ? app.sim_model : app.build_model;
    mjv_moveCamera(m, action, dx / fbh, dy / fbh, &app.scn, &app.cam);
}

// ---------------------------------------------------------------------------
// ScrollCallback
// ---------------------------------------------------------------------------

void ScrollCallback(GLFWwindow* window, double /*xoffset*/, double yoffset) {
    AppState& app = g_app;

    int fbw, fbh, winw, winh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &winw, &winh);

    double cx, cy;
    glfwGetCursorPos(window, &cx, &cy);

    double cx_fb = (winw > 0) ? cx * fbw / winw : cx;
    double cy_fb = (winh > 0) ? cy * fbh / winh : cy;

    app.uistate.type = mjEVENT_SCROLL;
    app.uistate.x    = cx_fb;
    app.uistate.y    = fbh - cy_fb;
    app.uistate.sy   = yoffset * ((winh > 0) ? (double)fbh / winh : 1.0);

    SetupUIRect(app, {0, 0, fbw, fbh});
    app.uistate.mouserect = mjr_findRect(
        (int)mju_round(cx_fb), (int)mju_round(app.uistate.y),
        app.uistate.nrect - 1, app.uistate.rect + 1) + 1;

    bool for_ui = (app.uistate.dragrect == app.ui.rectid) ||
                  (app.uistate.dragrect == 0 &&
                   app.uistate.mouserect == app.ui.rectid);
    if (for_ui) {
        if (mjui_event(&app.ui, &app.uistate, &app.con)) return;
    }

    if (app.uistate.mouserect == app.ui.rectid) return;

    mjModel* m = (app.mode == AppMode::SIMULATE && app.sim_model)
                 ? app.sim_model : app.build_model;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &app.scn, &app.cam);
}
