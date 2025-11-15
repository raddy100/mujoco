#include "ProGuiSim.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <cmath>

// ---------------- Constructors ----------------

ProGuiSim::ProGuiSim(mjModel* model,
                     mjData* data,
                     mjSpec* spec,
                     mjvScene* scene,
                     mjrContext* context,
                     mjUI* ui,
                     mjuiState* uiState,
                     mjvCamera* camera,
                     mjvOption* option) noexcept
  : model_(model),
    data_(data),
    spec_(spec),
    scene_(scene),
    context_(context),
    ui_(ui),
    uiState_(uiState),
    camera_(camera),
    option_(option) {
  if (camera_) {
    camYawDeg_   = camera_->azimuth;
    camPitchDeg_ = camera_->elevation;
    camDistance_ = camera_->distance;
  }
}

ProGuiSim::ProGuiSim(mjModel* model,
                     mjData* data,
                     mjSpec* spec,
                     mjvScene* scene,
                     mjrContext* context,
                     mjUI* ui,
                     mjuiState* uiState,
                     mjvCamera* camera,
                     mjvOption* option,
                     bool validate)
  : ProGuiSim(model, data, spec, scene, context, ui, uiState, camera, option) {
  if (validate) {
    if (!model_ || !data_ || !spec_ || !scene_ || !context_ || !ui_ || !uiState_ || !camera_ || !option_) {
      throw std::invalid_argument("ProGuiSim: null mandatory pointer in validated construction");
    }
  }
}

// ---------------- Private Helpers ----------------

Result ProGuiSim::recompileIfNeeded() {
  // Placeholder: In legacy code many operations recompiled spec -> model; here we assume caller did so.
  if (!model_ || !data_) {
    return Result::notReady("recompileIfNeeded: model/data null");
  }
  return Result::success();
}

void ProGuiSim::applyGapChange() noexcept {
  if (gapRatio_ < 0.0) gapRatio_ = 0.0;
  if (gapRatio_ > 1.0) gapRatio_ = 1.0;
  if (applyGapLayoutFn_) {
    applyGapLayoutFn_(gapRatio_);
  }
}

// ---------------- Public Methods ----------------

Result ProGuiSim::rebuildRenderContext() noexcept {
  if (!model_ || !scene_ || !context_ || !ui_ || !uiState_) {
    return Result::notReady("rebuildRenderContext: required pointers are null");
  }
  // Guard against OpenGL usage before loader/context are ready.
  if (!glReady_) {
    return Result::notReady("rebuildRenderContext: GL not ready");
  }
  // Basic invariants (scene/context structs are value-initialized).
  assert(scene_->ngeom >= 0);
 
  if(!firstInit) {
  // Free old resources if any
  mjv_freeScene(scene_);
  mjr_freeContext(context_);
  } else {
    firstInit = false;
  }
  // Recreate defaults
  mjv_defaultScene(scene_);
  mjr_defaultContext(context_);
 
  // Build scene and renderer
  mjv_makeScene(model_, scene_, 2000);
  mjr_makeContext(model_, context_, mjFONTSCALE_150);
 
  // Resize UI to new atlas
  mjui_resize(ui_, context_);
 
  std::cout << "[render] context rebuilt (fontscale=150)\n";
  return Result::success();
}

Result ProGuiSim::applyBuildModeOptions() noexcept {
  if (!model_) return Result::notReady("applyBuildModeOptions: model null");
  if (!physicsEnabled_) {
    model_->opt.gravity[0] = 0.0;
    model_->opt.gravity[1] = 0.0;
    model_->opt.gravity[2] = 0.0;
    model_->opt.disableflags |= mjDSBL_GRAVITY;
  }
  return Result::success();
}

Result ProGuiSim::increaseGap() {
  gapRatio_ = std::min(1.0, gapRatio_ + 0.05);
  applyGapChange();
  return Result::success();
}

Result ProGuiSim::decreaseGap() {
  gapRatio_ = std::max(0.0, gapRatio_ - 0.05);
  applyGapChange();
  return Result::success();
}

Result ProGuiSim::enablePhysics() {
  if (!model_) return Result::notReady("enablePhysics: model null");
  if (physicsEnabled_) return Result::success();

  // Sanity: previously captured gravity should be finite.
  assert(std::isfinite(savedGravity_[0]) && std::isfinite(savedGravity_[1]) && std::isfinite(savedGravity_[2]));

  // Restore saved gravity
  model_->opt.gravity[0] = savedGravity_[0];
  model_->opt.gravity[1] = savedGravity_[1];
  model_->opt.gravity[2] = savedGravity_[2];
  model_->opt.disableflags &= ~mjDSBL_GRAVITY;
  physicsEnabled_ = true;
  return Result::success();
}

void ProGuiSim::captureCurrentGravity() noexcept {
  if (model_) {
    savedGravity_[0] = model_->opt.gravity[0];
    savedGravity_[1] = model_->opt.gravity[1];
    savedGravity_[2] = model_->opt.gravity[2];
  }
}

Result ProGuiSim::debugPrintProbeContacts(std::string_view probeGeomName,
                                          const std::unordered_map<int, size_t>& bodyIdToIndex) const {
  if (probeGeomName.empty()) return Result::invalidArg("debugPrintProbeContacts: empty probe name");
  if (!model_ || !data_) return Result::notReady("debugPrintProbeContacts: model/data null");

  int probeGeomId = mj_name2id(model_, mjOBJ_GEOM, std::string(probeGeomName).c_str());
  if (probeGeomId < 0) {
    std::cout << "Probe geom not found: " << probeGeomName << "\n";
    return Result::invalidArg("probe geom id not found");
  }

  mj_kinematics(model_, data_);
  mj_collision(model_, data_);

  std::cout << "contacts total: " << data_->ncon << "\n";
  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact& c = data_->contact[i];
    const bool involvesProbe = (c.geom1 == probeGeomId || c.geom2 == probeGeomId);
    if (!involvesProbe) continue;

    int otherGeom = (c.geom1 == probeGeomId ? c.geom2 : c.geom1);
    if (otherGeom < 0 || otherGeom >= model_->ngeom) continue;

    int otherBody = model_->geom_bodyid[otherGeom];
    const char* gname1 = mj_id2name(model_, mjOBJ_GEOM, c.geom1);
    const char* gname2 = mj_id2name(model_, mjOBJ_GEOM, c.geom2);
    const char* bname  = mj_id2name(model_, mjOBJ_BODY, otherBody);

    auto it = bodyIdToIndex.find(otherBody);
    if (it != bodyIdToIndex.end()) {
      std::cout << "Probe contact with chainIndex=" << it->second
                << " body=" << (bname ? bname : "<noname>")
                << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", "
                << (gname2 ? gname2 : "<g2>") << ")\n";
    } else {
      std::cout << "Probe contact with bodyId=" << otherBody
                << " body=" << (bname ? bname : "<noname>")
                << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", "
                << (gname2 ? gname2 : "<g2>") << ")\n";
    }
  }
  return Result::success();
}

std::optional<int> ProGuiSim::probeHitBodyId(std::string_view probeGeomName,
                                             int newestBodyId) const {
  if (probeGeomName.empty() || !model_ || !data_) return std::nullopt;
  int probeId = mj_name2id(model_, mjOBJ_GEOM, std::string(probeGeomName).c_str());
  if (probeId < 0) return std::nullopt;

  mj_kinematics(model_, data_);
  mj_collision(model_, data_);

  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact& c = data_->contact[i];
    int g1 = c.geom1;
    int g2 = c.geom2;
    if (g1 == probeId || g2 == probeId) {
      int otherGeom = (g1 == probeId ? g2 : g1);
      if (otherGeom >= 0 && otherGeom < model_->ngeom) {
        int otherBody = model_->geom_bodyid[otherGeom];
        // Legacy logic: require contype bit & 1 and not newest body.
        if ((model_->geom_contype[otherGeom] & 1) && otherBody != newestBodyId) {
          return otherBody;
        }
      }
    }
  }
  return std::nullopt;
}

Result ProGuiSim::updateBodyIdIndexMap(const std::vector<std::string>& bodyNames,
                                       std::unordered_map<int, size_t>& outMap) {
  if (!model_) return Result::notReady("updateBodyIdIndexMap: model null");
  outMap.clear();
  for (size_t i = 0; i < bodyNames.size(); ++i) {
    const std::string& nm = bodyNames[i];
    int id = mj_name2id(model_, mjOBJ_BODY, nm.c_str());
    if (id >= 0) {
      outMap[id] = i;
    }
  }
  return Result::success();
}

// -------- Camera control (mouse-driven) --------

void ProGuiSim::cameraBeginDrag(double x, double y) noexcept {
  camDragging_ = true;
  camLastX_ = x;
  camLastY_ = y;
}

void ProGuiSim::cameraDrag(double x, double y) noexcept {
  if (!camDragging_) return;
  const double dx = x - camLastX_;
  const double dy = y - camLastY_;
  camLastX_ = x;
  camLastY_ = y;

  // Update yaw/pitch (degrees). Positive dx -> yaw+, positive dy -> pitch- (drag up looks up).
  camYawDeg_   += dx * camRotSensitivity_;
  camPitchDeg_ -= dy * camRotSensitivity_;

  // Normalize yaw and clamp pitch
  if (camYawDeg_ > 360.0 || camYawDeg_ < -360.0) {
    camYawDeg_ = std::fmod(camYawDeg_, 360.0);
  }
  camPitchDeg_ = std::max(-89.0, std::min(89.0, camPitchDeg_));
}

void ProGuiSim::cameraEndDrag() noexcept {
  camDragging_ = false;
}

void ProGuiSim::cameraZoom(double yoffset) noexcept {
  // Exponential zoom for smoothness
  const double factor = std::exp(-yoffset * camZoomSensitivity_);
  camDistance_ *= factor;
  if (camDistance_ < camMinDistance_) camDistance_ = camMinDistance_;
  if (camDistance_ > camMaxDistance_) camDistance_ = camMaxDistance_;
}

void ProGuiSim::updateCamera() noexcept {
  if (!camera_) return;

  // Quaternion-based orientation: yaw (Z) then pitch (X)
  const double yaw   = camYawDeg_   * M_PI / 180.0;
  const double pitch = camPitchDeg_ * M_PI / 180.0;

  const double cz = std::cos(yaw * 0.5),   sz = std::sin(yaw * 0.5);
  const double cx = std::cos(pitch * 0.5), sx = std::sin(pitch * 0.5);

  // qz about Z, qx about X; q = qz * qx
  const double qz_w = cz, qz_x = 0.0, qz_y = 0.0, qz_z = sz;
  const double qx_w = cx, qx_x = sx, qx_y = 0.0, qx_z = 0.0;

  const double q_w = qz_w*qx_w - qz_x*qx_x - qz_y*qx_y - qz_z*qx_z;
  const double q_x = qz_w*qx_x + qz_x*qx_w + qz_y*qx_z - qz_z*qx_y;
  const double q_y = qz_w*qx_y - qz_x*qx_z + qz_y*qx_w + qz_z*qx_x;
  const double q_z = qz_w*qx_z + qz_x*qx_y - qz_y*qx_x + qz_z*qx_w;

  // Rotate forward vector v=(0,0,-1) by quaternion q
  const double vx = 0.0, vy = 0.0, vz = -1.0;
  const double tx = 2.0 * (q_y*vz - q_z*vy);
  const double ty = 2.0 * (q_z*vx - q_x*vz);
  const double tz = 2.0 * (q_x*vy - q_y*vx);
  const double fx = vx + q_w*tx + (q_y*tz - q_z*ty);
  const double fy = vy + q_w*ty + (q_z*tx - q_x*tz);
  const double fz = vz + q_w*tz + (q_x*ty - q_y*tx);
  (void)fx; (void)fy; (void)fz; // Forward vector available if we later position camera via lookat

  // Apply to MuJoCo camera using azimuth/elevation/distance for compatibility
  camera_->azimuth   = static_cast<mjtNum>(camYawDeg_);
  camera_->elevation = static_cast<mjtNum>(camPitchDeg_);
  camera_->distance  = static_cast<mjtNum>(std::max(camMinDistance_, std::min(camMaxDistance_, camDistance_)));
}
