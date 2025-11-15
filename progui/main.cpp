#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
 
#include <iostream>
#include <string>

#include "Result.h"
#include "ProGuiSim.h"
#include "ProGuiChain.h"
#include "ProGuiUI.h"


namespace {
ProGuiChain* gGapChain = nullptr;

void gapLayoutForwarder(double ratio) noexcept {
  if (gGapChain) {
    gGapChain->applyGapLayout(ratio);
  }
}

struct AppContext {
  ProGuiUI* ui{nullptr};
};

AppContext* getContext(GLFWwindow* window) noexcept {
  return static_cast<AppContext*>(glfwGetWindowUserPointer(window));
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if (AppContext* ctx = getContext(window); ctx && ctx->ui) {
    ctx->ui->onKeyboard(key, scancode, action, mods);
  }
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  if (AppContext* ctx = getContext(window); ctx && ctx->ui) {
    ctx->ui->onMouseMove(xpos, ypos);
  }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  if (AppContext* ctx = getContext(window); ctx && ctx->ui) {
    ctx->ui->onMouseButton(button, action, mods);
  }
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  if (AppContext* ctx = getContext(window); ctx && ctx->ui) {
    ctx->ui->onScroll(xoffset, yoffset);
  }
}
} // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

    mjSpec *spec = mj_makeSpec();
    mjsBody *world = mjs_findBody(spec, "world");
    mjsTexture *tex = mjs_addTexture(spec);
  mjs_setName(tex->element, "floor_tex");
  tex->type = mjTEXTURE_2D;
  tex->builtin = mjBUILTIN_CHECKER;
  tex->mark = mjMARK_EDGE;
  // medium gray
  tex->rgb1[0] = 0.55;
  tex->rgb1[1] = 0.55;
  tex->rgb1[2] = 0.55;
  // dark blue
  tex->rgb2[0] = 0.06;
  tex->rgb2[1] = 0.10;
  tex->rgb2[2] = 0.35;
  tex->markrgb[0] = 0.10;
  tex->markrgb[1] = 0.10;
  tex->markrgb[2] = 0.10;
  tex->width = 512;
  tex->height = 512;
  tex->nchannel = 3;

  // material
  mjsMaterial *mat = mjs_addMaterial(spec, nullptr);
  mjs_setName(mat->element, "floor_mat");
  mat->texrepeat[0] = 12.0f; // tile many times so it's easy to see
  mat->texrepeat[1] = 12.0f;
  // bind the RGB texture
  mjs_setInStringVec(mat->textures, mjTEXROLE_RGB, "floor_tex");

  // floor geom
  mjsGeom *floor = mjs_addGeom(world, nullptr);
  floor->type = mjGEOM_PLANE;
  floor->size[0] = 25.0; // plane extents (visualization)
  floor->size[1] = 25.0;
  floor->size[2] = 0.1;
  // assign material with checker texture (keep white so texture shows as-is)
  mjs_setString(floor->material, "floor_mat");
  floor->rgba[0] = 0.5f;
  floor->rgba[1] = 0.5f;
  floor->rgba[2] = 0.5f;
  floor->rgba[3] = 0.5f;
  floor->contype = 1;
  floor->conaffinity = 1 | 2 | 4;
  floor->friction[0] = 1.0;
  floor->friction[1] = 0.005;
  floor->friction[2] = 0.0001;

          auto add_directional_light = [&](double dx, double dy, double dz, float diff, float spec, float amb)
  {
      mjsLight *L = mjs_addLight(world, nullptr);
      L->type = mjLIGHT_DIRECTIONAL; // supported type
      L->active = 1;
      L->dir[0] = dx;
      L->dir[1] = dy;
      L->dir[2] = dz;
      L->castshadow = 1;
      L->diffuse[0] = diff;
      L->diffuse[1] = diff;
      L->diffuse[2] = diff;
      L->specular[0] = spec;
      L->specular[1] = spec;
      L->specular[2] = spec;
      L->ambient[0] = amb;
      L->ambient[1] = amb;
      L->ambient[2] = amb;
  };

  // two opposing directional lights from above for even illumination
  add_directional_light(-0.3, -0.2, -1.0, 1.0f, 0.6f, 0.06f);
  add_directional_light(0.3, 0.2, -1.0, 0.8f, 0.4f, 0.04f);


  if (!spec) {
    std::cerr << "[init] failed to allocate mjSpec\n";
    return 1;
  }

  char error[1024]{0};
  mjModel *model = mj_compile(spec, nullptr);
  if (!model) {
    std::cerr << "[init] mj_makeModel failed: " << error << "\n";
    mj_deleteSpec(spec);
    return 1;
  }

  mjData* data = mj_makeData(model);
  if (!data) {
    std::cerr << "[init] mj_makeData failed\n";
    mj_deleteModel(model);
    mj_deleteSpec(spec);
    return 1;
  }

  mjvScene scene{};
  mjv_defaultScene(&scene);
  mjrContext context{};
  mjr_defaultContext(&context);
  mjUI ui{};
  mjuiState uiState{};
  mjvCamera camera{};
  mjv_defaultCamera(&camera);
  camera.lookat[0] = 0.0;
  camera.lookat[1] = 0.0;
  camera.lookat[2] = 0.5;
  camera.distance = 3.0;
  camera.azimuth = 90.0;
  camera.elevation = -25.0;
  mjvOption option{};
  mjv_defaultOption(&option);

  bool glfwInitialized = false;
  GLFWwindow* window = nullptr;

  auto cleanup = [&]() {
    if (window) {
      glfwDestroyWindow(window);
      window = nullptr;
    }
    if (glfwInitialized) {
      glfwTerminate();
      glfwInitialized = false;
    }
    mjv_freeScene(&scene);
    mjr_freeContext(&context);
    if (data) {
      mj_deleteData(data);
      data = nullptr;
    }
    if (model) {
      mj_deleteModel(model);
      model = nullptr;
    }
    if (spec) {
      mj_deleteSpec(spec);
      spec = nullptr;
    }
    gGapChain = nullptr;
  };

  if (!glfwInit()) {
    std::cerr << "[init] glfwInit failed\n";
    cleanup();
    return 1;
  }
  glfwInitialized = true;

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  window = glfwCreateWindow(1200, 900, "ProGUI - Cube Spawner", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&camera);
  mjv_defaultOption(&option);
  mjv_defaultScene(&scene);
  mjr_defaultContext(&context);

  // Build scene and renderer
  mjv_makeScene(model, &scene, 2000);
  mjr_makeContext(model, &context, mjFONTSCALE_100);

  // Resize UI to new atlas
  mjui_resize(&ui, &context);
 
  ProGuiSim sim(model, data, spec, &scene, &context, &ui, &uiState, &camera, &option);
  // Mark GL readiness so early rebuilds are allowed after a current GLFW context.
  sim.markGlReady(true);
 
  ProGuiChain chain(spec, model, data, &sim);
  gGapChain = &chain;
  sim.setApplyGapLayoutCallback(&gapLayoutForwarder);

  ProGuiUI uiAdapter(window, sim, chain);
  AppContext appCtx{&uiAdapter};
  glfwSetWindowUserPointer(window, &appCtx);
  glfwSetKeyCallback(window, keyCallback);
  glfwSetCursorPosCallback(window, cursorPosCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetScrollCallback(window, scrollCallback);

  // Initialize default GUI buttons
  uiAdapter.initDefaultButtons();
 
  if (!chain.canSpawnCube()) {
    std::cerr << "[init] chain not ready for initial spawn\n";
    cleanup();
    return 1;
  }

  Result initialSpawn = chain.spawnCube();
  if (!initialSpawn.ok()) {
    std::cerr << "[init] initial spawn failed: " << initialSpawn.message << "\n";
    cleanup();
    return 1;
  }

  sim.applyBuildModeOptions();
  mj_forward(model, data);
  sim.rebuildRenderContext();


 
  while (!glfwWindowShouldClose(window)) {
    if (sim.physicsEnabled()) {
      mj_step(model, data);
    } else {
      mj_forward(model, data);
    }

    int fbWidth = 0;
    int fbHeight = 0;
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
    mjrRect viewport{0, 0, fbWidth, fbHeight};

    // Apply mouse-driven camera to mjvCamera before scene update
    sim.updateCamera();
 
    mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);

    // Render GUI overlay (buttons, labels) on top of 3D scene
    uiAdapter.renderGUI(context, viewport);
 
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  cleanup();
  return 0;
}