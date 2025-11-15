#include "ProGuiChain.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <cstring>
#include <algorithm>

// ---------------- Constructors ----------------

ProGuiChain::ProGuiChain(mjSpec* spec,
                         mjModel* model,
                         mjData* data,
                         ProGuiSim* sim) noexcept
  : spec_(spec), model_(model), data_(data), sim_(sim) {}

ProGuiChain::ProGuiChain(mjSpec* spec,
                         mjModel* model,
                         mjData* data,
                         ProGuiSim* sim,
                         bool validate)
  : ProGuiChain(spec, model, data, sim) {
  if (validate) {
    if (!spec_ || !model_ || !data_ || !sim_) {
      throw std::invalid_argument("ProGuiChain: null pointer in validated construction");
    }
  }
}

// ---------------- Internal Helpers ----------------

int ProGuiChain::parseCubeId(const char* name) const noexcept {
  if (!name) return -1;
  const char prefix[] = "cube_";
  constexpr size_t kPrefixLen = sizeof(prefix) - 1;
  if (std::strncmp(name, prefix, kPrefixLen) != 0) return -1;
  const char* num = name + kPrefixLen;
  if (!*num) return -1;
  char* endp = nullptr;
  long val = std::strtol(num, &endp, 10);
  if (endp == num || val < 0) return -1;
  return static_cast<int>(val);
}

void ProGuiChain::refreshNextCubeIdFromSpec() {
  if (!spec_) return;
  int maxId = 0;
  for (mjsElement* el = mjs_firstElement(spec_, mjOBJ_BODY); el; el = mjs_nextElement(spec_, el)) {
    const char* nm = mjs_getString(mjs_getName(el));
    int id = parseCubeId(nm);
    if (id > maxId) maxId = id;
  }
  nextCubeId_ = maxId;
}

double ProGuiChain::currentGap(double gapRatio) const noexcept {
  const double boxEdge = 2.0 * ProGuiSim::kBoxHalf;
  return gapRatio * boxEdge;
}

bool ProGuiChain::isTurnToken(const std::string& s) {
  return (s == "left" || s == "right" || s == "up" || s == "down" ||
          s == "Turnfront" || s == "TurnBack");
}

bool ProGuiChain::isLoopToken(const std::string& s) {
  return s.rfind("loop ", 0) == 0;
}

void ProGuiChain::pruneHistoryOnDelete() {
  if (directionHistory_.empty()) return;
  if (directionHistory_.back() == "forward") {
    directionHistory_.pop_back();
  }
  while (!directionHistory_.empty() && isLoopToken(directionHistory_.back())) {
    directionHistory_.pop_back();
  }
  while (!directionHistory_.empty() && isTurnToken(directionHistory_.back())) {
    directionHistory_.pop_back();
  }
}

void ProGuiChain::recordForward() {
  if (suppressDirRecord_) return;
  directionHistory_.push_back("forward");
}

void ProGuiChain::recordTurn(SpawnFace oldFace, SpawnFace newFace) {
  if (oldFace == newFace) return;
  if (suppressDirRecord_) return;

  int oldAxis, oldSign;
  faceToAxisSign(oldFace, oldAxis, oldSign);
  int newAxis, newSign;
  faceToAxisSign(newFace, newAxis, newSign);

  // If turning into Z
  if (newAxis == 2) {
    directionHistory_.push_back(newSign > 0 ? "up" : "down");
    return;
  }
  // Both in XY plane
  if (oldAxis != 2 && newAxis != 2) {
    std::string dir;
    if (oldAxis == 0 && newAxis == 1) {
      int prod = oldSign * newSign;
      dir = (prod > 0) ? "left" : "right";
    } else if (oldAxis == 1 && newAxis == 0) {
      int prod = oldSign * newSign;
      dir = (prod > 0) ? "right" : "left";
    } else {
      return;
    }
    directionHistory_.push_back(dir);
    return;
  }
  // Leaving Z to XY
  if (oldAxis == 2 && (newAxis == 0 || newAxis == 1)) {
    directionHistory_.push_back(newSign > 0 ? "Turnfront" : "TurnBack");
    return;
  }
  // Other cases ignored
}

void ProGuiChain::clearDirectionHistory() noexcept {
  directionHistory_.clear();
}

void ProGuiChain::deleteEqualitiesReferencing(const std::string& bodyName) {
  if (!spec_) return;
  std::vector<mjsElement*> toDelete;
  for (mjsElement* el = mjs_firstElement(spec_, mjOBJ_EQUALITY); el; el = mjs_nextElement(spec_, el)) {
    mjsEquality* eq = mjs_asEquality(el);
    if (!eq) continue;
    const char* n1 = mjs_getString(eq->name1);
    const char* n2 = mjs_getString(eq->name2);
    if ((n1 && bodyName == n1) || (n2 && bodyName == n2)) {
      toDelete.push_back(el);
    }
  }
  for (auto* el : toDelete) {
    mjs_delete(spec_, el);
  }
}

bool ProGuiChain::loopContactCheck(std::string& outTargetBodyName) {
  outTargetBodyName.clear();
  if (!lastBody_ || !model_ || !data_) return false;

  int axisIdx = 0, sign = +1;
  faceToAxisSign(spawnFace_, axisIdx, sign);
  const double boxEdge = 2.0 * ProGuiSim::kBoxHalf;
  const double gap = currentGap(sim_->gapRatio());
  const double step = boxEdge + gap;

  mj_kinematics(model_, data_);

  const int lastId = mj_name2id(model_, mjOBJ_BODY, chain_.empty() ? "" : chain_.back().name.c_str());
  if (lastId < 0) return false;
  const double* lastPos = data_->xpos + 3 * lastId;

  double tgtPos[3] = { lastPos[0], lastPos[1], lastPos[2] };
  tgtPos[axisIdx] += sign * step;
  const double epsAxis = 1e-6;
  const double epsOther = 0.25 * ProGuiSim::kBoxHalf;

  for (const auto& e : chain_) {
    if (!e.specBody || e.specBody == lastBody_) continue;
    const int bid = mj_name2id(model_, mjOBJ_BODY, e.name.c_str());
    if (bid < 0) continue;
    const double* p = data_->xpos + 3 * bid;
    double daxis = std::fabs(p[axisIdx] - tgtPos[axisIdx]);
    double d1 = std::fabs(p[(axisIdx + 1) % 3] - tgtPos[(axisIdx + 1) % 3]);
    double d2 = std::fabs(p[(axisIdx + 2) % 3] - tgtPos[(axisIdx + 2) % 3]);
    if (daxis <= epsAxis && d1 <= epsOther && d2 <= epsOther) {
      outTargetBodyName = e.name;
      return true;
    }
  }
  return false;
}

Result ProGuiChain::loopCreate(int spawnAxis, int spawnSign,
                               const std::string& newBodyName,
                               const std::string& targetBodyName) {
  if (!spec_ || !lastBody_ || targetBodyName.empty())
    return Result::notReady("loopCreate: prerequisites missing");

  auto equalityExists = [&](const std::string& nm) -> bool {
    return mjs_findElement(spec_, mjOBJ_EQUALITY, nm.c_str()) != nullptr;
  };

  double prev_anchor[3] = {0,0,0};
  double new_anchor[3]  = {0,0,0};
  double tgt_anchor_to_prev[3] = {0,0,0};
  double tgt_anchor_to_new[3]  = {0,0,0};

  prev_anchor[spawnAxis]        = spawnSign * ProGuiSim::kBoxHalf;
  new_anchor[spawnAxis]         = -spawnSign * ProGuiSim::kBoxHalf;
  tgt_anchor_to_prev[spawnAxis] = -spawnSign * ProGuiSim::kBoxHalf;
  tgt_anchor_to_new[spawnAxis]  = spawnSign * ProGuiSim::kBoxHalf;

  std::string prevName = chain_.empty() ? std::string() : chain_.back().name;
  std::string ename1 = prevName + "__to__" + targetBodyName + "_connect";
  std::string ename2 = newBodyName + "__to__" + targetBodyName + "_connect";

  if (!prevName.empty() && !equalityExists(ename1)) {
    if (mjsEquality* eq1 = mjs_addEquality(spec_, nullptr)) {
      eq1->type = mjEQ_CONNECT;
      eq1->objtype = mjOBJ_BODY;
      eq1->active = 1;
      eq1->solref[0] = ProGuiSim::kSolref[0];
      eq1->solref[1] = ProGuiSim::kSolref[1];
      for (int i = 0; i < 5; ++i) eq1->solimp[i] = ProGuiSim::kSolimp[i];

      mjs_setString(eq1->name1, prevName.c_str());
      mjs_setString(eq1->name2, targetBodyName.c_str());

      eq1->data[0] = prev_anchor[0];
      eq1->data[1] = prev_anchor[1];
      eq1->data[2] = prev_anchor[2];
      eq1->data[3] = tgt_anchor_to_prev[0];
      eq1->data[4] = tgt_anchor_to_prev[1];
      eq1->data[5] = tgt_anchor_to_prev[2];

      mjs_setName(eq1->element, ename1.c_str());
    }
  }
  if (!equalityExists(ename2)) {
    if (mjsEquality* eq2 = mjs_addEquality(spec_, nullptr)) {
      eq2->type = mjEQ_CONNECT;
      eq2->objtype = mjOBJ_BODY;
      eq2->active = 1;
      eq2->solref[0] = ProGuiSim::kSolref[0];
      eq2->solref[1] = ProGuiSim::kSolref[1];
      for (int i = 0; i < 5; ++i) eq2->solimp[i] = ProGuiSim::kSolimp[i];

      mjs_setString(eq2->name1, newBodyName.c_str());
      mjs_setString(eq2->name2, targetBodyName.c_str());

      eq2->data[0] = new_anchor[0];
      eq2->data[1] = new_anchor[1];
      eq2->data[2] = new_anchor[2];
      eq2->data[3] = tgt_anchor_to_new[0];
      eq2->data[4] = tgt_anchor_to_new[1];
      eq2->data[5] = tgt_anchor_to_new[2];

      mjs_setName(eq2->element, ename2.c_str());
    }
  }

  // Record loop label in direction history referencing target chain index.
  int targetIndex = -1;
  for (size_t i = 0; i < chain_.size(); ++i) {
    if (chain_[i].name == targetBodyName) { targetIndex = static_cast<int>(i); break; }
  }
  if (targetIndex >= 0) {
    int oneBased = targetIndex + 1;
    directionHistory_.push_back(std::string("loop ") + std::to_string(oneBased));
  }
  return Result::success();
}

Result ProGuiChain::updateProbeForFace() {
  if (!spec_ || !model_ || !data_ || !lastBody_ || !probeRect_)
    return Result::notReady("updateProbeForFace: prerequisites missing");

  int axisIdx = 0, sign = +1;
  faceToAxisSign(spawnFace_, axisIdx, sign);

  const double boxEdge = 2.0 * ProGuiSim::kBoxHalf;
  const double gap = currentGap(sim_->gapRatio());

  probeRect_->type = mjGEOM_BOX;
  probeRect_->size[0] = ProGuiSim::kBoxHalf;
  probeRect_->size[1] = ProGuiSim::kBoxHalf;
  probeRect_->size[2] = ProGuiSim::kBoxHalf;

  double ppos[3] = {0,0,0};
  ppos[axisIdx] = sign * (boxEdge + gap);

  probeRect_->pos[0] = ppos[0];
  probeRect_->pos[1] = ppos[1];
  probeRect_->pos[2] = ppos[2];

  probeRect_->rgba[0] = 0.1f;
  probeRect_->rgba[1] = 1.0f;
  probeRect_->rgba[2] = 0.1f;
  probeRect_->rgba[3] = 0.9f;

  probeRect_->contype = 4;
  probeRect_->conaffinity = 1;
  probeRect_->density = 0.0;
  probeRect_->margin = ProGuiSim::kGeomMargin;

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));

  sim_->applyBuildModeOptions();
  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  mj_forward(model_, data_);
  sim_->rebuildRenderContext();
  return Result::success();
}

void ProGuiChain::reconstructChainFromSpec() {
  chain_.clear();
  lastBody_ = nullptr;
  lastMarker_ = nullptr;
  probeRect_ = nullptr;
  probeName_.clear();
  clearDirectionHistory();

  if (!spec_) return;
  mjsBody* world = mjs_findBody(spec_, "world");
  if (!world) {
    refreshNextCubeIdFromSpec();
    return;
  }

  mjsBody* first = nullptr;
  for (mjsElement* el = mjs_firstChild(world, mjOBJ_BODY, 0); el; el = mjs_nextChild(world, el, 0)) {
    if (el->elemtype == mjOBJ_BODY) {
      const char* nm = mjs_getString(mjs_getName(el));
      if (nm && std::strncmp(nm, "cube_", 5) == 0) {
        first = mjs_asBody(el);
        break;
      }
    }
  }
  if (!first) {
    refreshNextCubeIdFromSpec();
    return;
  }

  mjsBody* cur = first;
  mjsBody* parent = world;
  while (cur) {
    const char* nm = mjs_getString(mjs_getName(cur->element));
    std::string name = nm ? nm : std::string();

    int id = parseCubeId(nm);
    if (id >= 0 && id > nextCubeId_) nextCubeId_ = id;

    ChainEntry entry{};
    entry.specBody = cur;
    entry.name = name;
    entry.bodyId = -1;

    if (parent && parent != cur) {
      int axis = 0;
      int sign = +1;
      double axv[3] = { std::fabs(cur->pos[0]), std::fabs(cur->pos[1]), std::fabs(cur->pos[2]) };
      if (axv[1] >= axv[0] && axv[1] >= axv[2]) axis = 1;
      else if (axv[2] >= axv[0] && axv[2] >= axv[1]) axis = 2;
      else axis = 0;
      sign = ((axis == 0 ? cur->pos[0] : (axis == 1 ? cur->pos[1] : cur->pos[2])) >= 0.0) ? +1 : -1;
      entry.axis = axis;
      entry.sign = sign;
      const double boxEdge = 2.0 * ProGuiSim::kBoxHalf;
      const double gap = currentGap(sim_->gapRatio());
      const double unit = boxEdge + gap;
      double posMag = std::fabs(axis == 0 ? cur->pos[0] : axis == 1 ? cur->pos[1] : cur->pos[2]);
      entry.distanceFactor = (posMag > 1.5 * unit) ? 2 : 1;
    } else {
      entry.axis = -1;
      entry.sign = +1;
      entry.distanceFactor = 1;
    }
    chain_.push_back(entry);

    std::string markerName = name + "_spawn_marker";
    if (mjsElement* mel = mjs_findElement(spec_, mjOBJ_GEOM, markerName.c_str())) {
      lastMarker_ = mjs_asGeom(mel);
    }
    std::string probeName = name + "_spawn_probe";
    if (mjsElement* pel = mjs_findElement(spec_, mjOBJ_GEOM, probeName.c_str())) {
      probeRect_ = mjs_asGeom(pel);
      probeName_ = probeName;
    }

    mjsBody* next = nullptr;
    for (mjsElement* el = mjs_firstChild(cur, mjOBJ_BODY, 0); el; el = mjs_nextChild(cur, el, 0)) {
      if (el->elemtype == mjOBJ_BODY) {
        const char* cnm = mjs_getString(mjs_getName(el));
        if (cnm && std::strncmp(cnm, "cube_", 5) == 0) {
          next = mjs_asBody(el);
          break;
        }
      }
    }
    if (!next) {
      lastBody_ = cur;
      break;
    }
    parent = cur;
    cur = next;
  }
}

// ---------------- Public Chain Operations ----------------

Result ProGuiChain::spawnCube() {
  return spawnCubeFromInput("forward");
}

Result ProGuiChain::spawnCubeFromInput(std::string_view dirToken) {
  if (!canSpawnCube()) {
    return Result::notReady("spawnCube: prerequisites missing");
  }
  // Chain invariant: if not empty, lastBody_ should be non-null.
  assert(chain_.empty() || lastBody_);

  // Determine new face based on token relative to current spawnFace_.
  SpawnFace newFace = spawnFace_;
  if (dirToken == "right") {
    switch (spawnFace_) {
      case SpawnFace::PosX: newFace = SpawnFace::NegY; break;
      case SpawnFace::NegX: newFace = SpawnFace::PosY; break;
      case SpawnFace::PosY: newFace = SpawnFace::PosX; break;
      case SpawnFace::NegY: newFace = SpawnFace::NegX; break;
      default: newFace = spawnFace_; break;
    }
  } else if (dirToken == "left") {
    switch (spawnFace_) {
      case SpawnFace::PosX: newFace = SpawnFace::PosY; break;
      case SpawnFace::NegX: newFace = SpawnFace::NegY; break;
      case SpawnFace::PosY: newFace = SpawnFace::NegX; break;
      case SpawnFace::NegY: newFace = SpawnFace::PosX; break;
      default: newFace = spawnFace_; break;
    }
  } else if (dirToken == "up") {
    newFace = SpawnFace::PosZ;
  } else if (dirToken == "down") {
    newFace = SpawnFace::NegZ;
  } else if (dirToken == "Turnfront") {
    if (spawnFace_ == SpawnFace::PosZ || spawnFace_ == SpawnFace::NegZ) newFace = SpawnFace::PosX;
  } else if (dirToken == "TurnBack") {
    if (spawnFace_ == SpawnFace::PosZ || spawnFace_ == SpawnFace::NegZ) newFace = SpawnFace::NegX;
  } else if (dirToken == "forward") {
    // keep same face
  } else {
    // unknown token -> treat as forward
  }

  if (newFace != spawnFace_) {
    recordTurn(spawnFace_, newFace);
    spawnFace_ = newFace;
    updateMarkerOnLastBody();
  }
  return doSpawn(true);
}

Result ProGuiChain::doSpawn(bool /*fromToken*/) {
  if (!spec_ || !model_ || !data_) {
    return Result::notReady("doSpawn: model/spec/data not ready");
  }
  const double boxHalf = ProGuiSim::kBoxHalf;
  const double halfSize[3] = { boxHalf, boxHalf, boxHalf };
  const double boxEdge = 2.0 * boxHalf;
  const double gap = currentGap(sim_->gapRatio());

  mjsBody* world = mjs_findBody(spec_, "world");
  if (!world) return Result::internalError("world body not found");

  mjsBody* parent = lastBody_ ? lastBody_ : world;

  bool obstructed = false;
  std::string targetBodyName;

  mjsBody* body = mjs_addBody(parent, nullptr);
  if (!body) return Result::internalError("add body failed");

  std::string bodyName = std::string("cube_") + std::to_string(++nextCubeId_);
  mjs_setName(body->element, bodyName.c_str());

  int spawnAxis = 0, spawnSign = +1;
  faceToAxisSign(spawnFace_, spawnAxis, spawnSign);

  double delta = boxEdge + gap;
  std::string outTarget;
  obstructed = loopContactCheck(outTarget);
  if (obstructed) {
    targetBodyName = outTarget;
    delta = 2.0 * delta;
  }

  if (!lastBody_) {
    body->pos[0] = 0;
    body->pos[1] = 0;
    body->pos[2] = 1.0;
  } else {
    double off = spawnSign * delta;
    body->pos[0] = (spawnAxis == 0) ? off : 0.0;
    body->pos[1] = (spawnAxis == 1) ? off : 0.0;
    body->pos[2] = (spawnAxis == 2) ? off : 0.0;
  }
  body->quat[0] = 1; body->quat[1] = 0; body->quat[2] = 0; body->quat[3] = 0;

  if (!lastBody_) {
    if (mjsJoint* fj = mjs_addJoint(body, nullptr)) {
      fj->type = mjJNT_FREE;
      fj->damping = ProGuiSim::kJointDamping;
      std::string jname = bodyName + "_free";
      mjs_setName(fj->element, jname.c_str());
    }
  } else {
    if (!obstructed) {
      int faceAxis = 0, faceSign = +1;
      faceToAxisSign(spawnFace_, faceAxis, faceSign);
      const double gap2 = currentGap(sim_->gapRatio());
      double anchor[3] = {0,0,0};
      anchor[faceAxis] = -faceSign * (ProGuiSim::kBoxHalf + 0.5 * gap2);
      if (mjsJoint* bj = mjs_addJoint(body, nullptr)) {
        bj->type = mjJNT_BALL;
        bj->pos[0] = anchor[0];
        bj->pos[1] = anchor[1];
        bj->pos[2] = anchor[2];
        bj->damping = ProGuiSim::kJointDamping;
        std::string jn = bodyName + "_ball";
        mjs_setName(bj->element, jn.c_str());

        const double a = ProGuiSim::kBoxHalf;
        const double g = gap2;
        const double c = 1.0 + (a > 0.0 ? (g / a) : 0.0);
        const double root2 = std::sqrt(2.0);
        double phi = (c >= root2) ? (mjPI / 2.0)
                                  : (std::asin(std::max(0.0, std::min(1.0, c / root2))) - (mjPI / 4.0));
        const double th_min = mjPI * 5.0 / 180.0;
        const double th_max = mjPI * 80.0 / 180.0;
        phi = std::max(th_min, std::min(th_max, phi));
        double phi_deg = phi * 180.0 / mjPI;
        bj->limited = mjLIMITED_TRUE;
        bj->range[0] = 0.0;
        bj->range[1] = phi_deg / 2;
      } else {
        return Result::internalError("add ball joint failed");
      }
    }
  }

  mjsGeom* geom = mjs_addGeom(body, nullptr);
  if (!geom) return Result::internalError("add geom failed");
  geom->type = mjGEOM_BOX;
  geom->size[0] = halfSize[0];
  geom->size[1] = halfSize[1];
  geom->size[2] = halfSize[2];
  geom->density = 1000.0;
  geom->rgba[0] = 1; geom->rgba[1] = 1; geom->rgba[2] = 1; geom->rgba[3] = 1;
  if (sim_->boxesCollideEnabled()) {
    geom->contype = 1;
    geom->conaffinity = 1 | 2 | 4;
    geom->friction[0] = 1.0;
    geom->friction[1] = 0.005;
    geom->friction[2] = 0.0001;
  } else {
    geom->contype = 1;
    geom->conaffinity = 2 | 4;
  }
  geom->margin = ProGuiSim::kGeomMargin;
  geom->solref[0] = ProGuiSim::kSolref[0];
  geom->solref[1] = ProGuiSim::kSolref[1];
  for (int i = 0; i < 5; ++i) geom->solimp[i] = ProGuiSim::kSolimp[i];

  if (mjsGeom* marker = mjs_addGeom(body, nullptr)) {
    marker->type = mjGEOM_SPHERE;
    const double sphereRadius = 0.1 * (2.0 * ProGuiSim::kBoxHalf);
    marker->size[0] = sphereRadius;
    marker->pos[0] = 0; marker->pos[1] = 0; marker->pos[2] = 0;
    int axisIdx = 0, sign = +1;
    faceToAxisSign(spawnFace_, axisIdx, sign);
    marker->pos[axisIdx] = sign * (ProGuiSim::kBoxHalf - 0.5 * sphereRadius);
    marker->rgba[0] = 1; marker->rgba[1] = 0.1f; marker->rgba[2] = 0.1f; marker->rgba[3] = 1;
    marker->contype = 0;
    marker->conaffinity = 0;
    marker->density = 0.0;
    std::string sname = bodyName + "_spawn_marker";
    mjs_setName(marker->element, sname.c_str());
    lastMarker_ = marker;
  }

  if (probeRect_) {
    probeRect_->contype = 0;
    probeRect_->conaffinity = 0;
    probeRect_->rgba[3] = 0.0f;
  }

  if (mjsGeom* probe = mjs_addGeom(body, nullptr)) {
    int axisIdx = 0, sign = +1;
    faceToAxisSign(spawnFace_, axisIdx, sign);
    const double gapL = currentGap(sim_->gapRatio());
    probe->type = mjGEOM_BOX;
    probe->size[0] = ProGuiSim::kBoxHalf;
    probe->size[1] = ProGuiSim::kBoxHalf;
    probe->size[2] = ProGuiSim::kBoxHalf;
    probe->pos[0] = 0; probe->pos[1] = 0; probe->pos[2] = 0;
    probe->pos[axisIdx] = sign * (boxEdge + gapL);
    probe->rgba[0] = 0.1f; probe->rgba[1] = 1.0f; probe->rgba[2] = 0.1f; probe->rgba[3] = 0.9f;
    probe->contype = 4;
    probe->conaffinity = 1;
    probe->density = 0.0;
    probe->margin = ProGuiSim::kGeomMargin;
    probeRect_ = probe;
    probeName_ = bodyName + "_spawn_probe";
    mjs_setName(probe->element, probeName_.c_str());
  }

  if (obstructed && lastBody_ && !targetBodyName.empty()) {
    loopCreate(spawnAxis, spawnSign, bodyName, targetBodyName);
  }

  if (lastBody_ && !suppressDirRecord_) {
    recordForward();
  }

  lastBody_ = body;
  int distFactor = obstructed ? 2 : 1;
  chain_.push_back(ChainEntry{ body, bodyName, -1, spawnAxis, spawnSign, distFactor });

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0) {
    return Result::recompileFailed(mjs_getError(spec_));
  }

  sim_->applyBuildModeOptions();
  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  return Result::success();
}

Result ProGuiChain::deleteLastCube() {
  if (!spec_ || !model_ || !data_ || !sim_) return Result::notReady("deleteLastCube: prerequisites missing");
  if (chain_.size() <= 1) return Result::invalidArg("root cannot be deleted");
  assert(lastBody_); // lastBody_ must align with chain back.

  pruneHistoryOnDelete();

  ChainEntry last = chain_.back();
  ChainEntry prev = chain_[chain_.size() - 2];

  deleteEqualitiesReferencing(last.name);
  if (last.specBody) mjs_delete(spec_, last.specBody->element);

  chain_.pop_back();
  lastBody_ = prev.specBody;
  lastMarker_ = nullptr;
  probeRect_ = nullptr;
  probeName_.clear();

  std::string prevMarkerName = prev.name + "_spawn_marker";
  std::string prevProbeName  = prev.name + "_spawn_probe";

  if (mjsElement* mel = mjs_findElement(spec_, mjOBJ_GEOM, prevMarkerName.c_str())) {
    lastMarker_ = mjs_asGeom(mel);
  }
  if (mjsElement* pel = mjs_findElement(spec_, mjOBJ_GEOM, prevProbeName.c_str())) {
    probeRect_ = mjs_asGeom(pel);
    probeName_ = prevProbeName;
    if (probeRect_) {
      probeRect_->contype = 4;
      probeRect_->conaffinity = 1;
      probeRect_->rgba[0] = 0.1f;
      probeRect_->rgba[1] = 1.0f;
      probeRect_->rgba[2] = 0.1f;
      probeRect_->rgba[3] = 0.9f;
      probeRect_->density = 0.0;
      probeRect_->margin  = ProGuiSim::kGeomMargin;
    }
  }

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));

  sim_->applyBuildModeOptions();
  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  updateMarkerOnLastBody();
  return Result::success();
}

Result ProGuiChain::moveLastCubeX(int dir) {
  if (!spec_ || !model_ || !data_) return Result::notReady("moveLastCubeX: not ready");
  if (chain_.size() < 2) return Result::invalidArg("need >=2 cubes");
  assert(chain_[0].specBody); // root must exist

  const double step = 2.0 * ProGuiSim::kBoxHalf;
  const double dx = (dir > 0 ? step : -step);
  const size_t N = chain_.size();
  const double invN = 1.0 / static_cast<double>(N);

  for (size_t i = 0; i < N; ++i) {
    mjsBody* b = chain_[i].specBody;
    if (!b) continue;
    double f = static_cast<double>(i + 1) * invN;
    b->pos[0] += dx * f;
  }

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));

  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  sim_->applyBuildModeOptions();
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  return Result::success();
}

Result ProGuiChain::moveLastCubeY(int dir) {
  if (!spec_ || !model_ || !data_) return Result::notReady("moveLastCubeY: not ready");
  if (chain_.size() < 2) return Result::invalidArg("need >=2 cubes");
  assert(chain_[0].specBody);

  const double step = 2.0 * ProGuiSim::kBoxHalf;
  const double dy = (dir > 0 ? step : -step);
  const size_t N = chain_.size();
  const double invN = 1.0 / static_cast<double>(N);

  for (size_t i = 0; i < N; ++i) {
    mjsBody* b = chain_[i].specBody;
    if (!b) continue;
    double f = static_cast<double>(i + 1) * invN;
    b->pos[1] += dy * f;
  }

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));

  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  sim_->applyBuildModeOptions();
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  return Result::success();
}

Result ProGuiChain::moveLastCubeZ(int dir) {
  if (!spec_ || !model_ || !data_) return Result::notReady("moveLastCubeZ: not ready");
  if (chain_.size() < 2) return Result::invalidArg("need >=2 cubes");
  assert(chain_[0].specBody);

  const double step = 2.0 * ProGuiSim::kBoxHalf;
  const double dz = (dir > 0 ? step : -step);
  const size_t N = chain_.size();
  const double invN = 1.0 / static_cast<double>(N);

  for (size_t i = 0; i < N; ++i) {
    mjsBody* b = chain_[i].specBody;
    if (!b) continue;
    double f = static_cast<double>(i + 1) * invN;
    b->pos[2] += dz * f;
  }

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));

  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  sim_->applyBuildModeOptions();
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  return Result::success();
}

Result ProGuiChain::enablePhysics() {
  if (chain_.empty()) return Result::invalidArg("enablePhysics: chain empty");
  auto res = sim_->enablePhysics();
  if (!res.ok()) return res;
  sim_->rebuildRenderContext();
  return Result::success();
}

Result ProGuiChain::savePrePhysicsState() {
  savedPrePhysicsChain_.clear();
  for (const auto& e : chain_) {
    if (!e.specBody) continue;
    // Invariant: quaternion first component should be finite.
    assert(std::isfinite(e.specBody->quat[0]));
    SavedBodyPos s{};
    s.name = e.name;
    s.pos[0] = e.specBody->pos[0];
    s.pos[1] = e.specBody->pos[1];
    s.pos[2] = e.specBody->pos[2];
    s.quat[0] = e.specBody->quat[0];
    s.quat[1] = e.specBody->quat[1];
    s.quat[2] = e.specBody->quat[2];
    s.quat[3] = e.specBody->quat[3];
    savedPrePhysicsChain_.push_back(s);
  }
  hasSavedPrePhysicsState_ = !savedPrePhysicsChain_.empty();
  return Result::success();
}

Result ProGuiChain::resetToSavedState() {
  if (!spec_ || !model_ || !data_) return Result::notReady("resetToSavedState: not ready");
  if (!hasSavedPrePhysicsState_ || savedPrePhysicsChain_.empty())
    return Result::invalidArg("no saved pre-physics state");
  assert(savedPrePhysicsChain_.size() > 0);

  // Disable gravity
  sim_->markPhysicsEnabled(false);
  model_->opt.gravity[0] = 0.0;
  model_->opt.gravity[1] = 0.0;
  model_->opt.gravity[2] = 0.0;
  model_->opt.disableflags |= mjDSBL_GRAVITY;

  size_t targetN = savedPrePhysicsChain_.size();
  while (chain_.size() > targetN) {
    auto r = deleteLastCube();
    if (!r.ok()) return r;
  }
  while (chain_.size() < targetN) {
    suppressDirRecord_ = true;
    auto r = spawnCube();
    suppressDirRecord_ = false;
    if (!r.ok()) return r;
  }

  size_t N = std::min(chain_.size(), targetN);
  for (size_t i = 0; i < N; ++i) {
    mjsBody* b = chain_[i].specBody;
    if (!b) continue;
    b->pos[0] = savedPrePhysicsChain_[i].pos[0];
    b->pos[1] = savedPrePhysicsChain_[i].pos[1];
    b->pos[2] = savedPrePhysicsChain_[i].pos[2];
    b->quat[0] = savedPrePhysicsChain_[i].quat[0];
    b->quat[1] = savedPrePhysicsChain_[i].quat[1];
    b->quat[2] = savedPrePhysicsChain_[i].quat[2];
    b->quat[3] = savedPrePhysicsChain_[i].quat[3];
  }

  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));

  mj_resetData(model_, data_);
  sim_->applyBuildModeOptions();
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  updateMarkerOnLastBody();
  return Result::success();
}

// ---------------- Persistence ----------------

Result ProGuiChain::saveDirections(const std::filesystem::path& file) {
  std::filesystem::path out = file;
  if (out.empty()) out = "directions.txt";
  std::ofstream ofs(out, std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) return Result::ioError("saveDirections: could not open file");
  for (const auto& dir : directionHistory_) ofs << dir << '\n';
  return Result::success();
}

Result ProGuiChain::loadDirections(const std::filesystem::path& file) {
  if (file.empty()) return Result::invalidArg("loadDirections: empty filename");
  std::ifstream ifs(file);
  if (!ifs.is_open()) return Result::ioError("loadDirections: could not open file");

  std::vector<std::string> tokens;
  std::string line;
  while (std::getline(ifs, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
      line.pop_back();
    if (line.empty()) continue;
    tokens.push_back(line);
  }
  ifs.close();

  if (!spec_ || !model_ || !data_) return Result::notReady("loadDirections: model/spec not ready");

  while (chain_.size() > 1) {
    auto r = deleteLastCube();
    if (!r.ok()) return r;
  }
  if (chain_.empty()) {
    suppressDirRecord_ = true;
    auto r = spawnCube();
    suppressDirRecord_ = false;
    if (!r.ok()) return r;
  }

  spawnFace_ = SpawnFace::PosX;
  updateMarkerOnLastBody();

  suppressDirRecord_ = true;
  for (size_t ti = 0; ti < tokens.size(); ++ti) {
    auto r = spawnCubeFromInput(tokens[ti]);
    if (!r.ok()) {
      suppressDirRecord_ = false;
      return r;
    }
    // Minimal incremental UI feedback omitted.
  }
  suppressDirRecord_ = false;

  directionHistory_ = tokens;

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0)
    return Result::recompileFailed(mjs_getError(spec_));
  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  mj_forward(model_, data_);
  refreshBodyIdIndexMap();
  sim_->rebuildRenderContext();
  updateMarkerOnLastBody();
  return Result::success();
}

// ---------------- Direction / Face ----------------

Result ProGuiChain::setSpawnFace(SpawnFace newFace) {
  if (newFace == spawnFace_) return Result::success();
  recordTurn(spawnFace_, newFace);
  spawnFace_ = newFace;
  return updateMarkerOnLastBody();
}

Result ProGuiChain::updateMarkerOnLastBody() {
  if (!spec_ || !model_ || !data_ || !lastBody_ || !lastMarker_)
    return Result::notReady("updateMarkerOnLastBody: prerequisites missing");
  assert(lastMarker_->size[0] >= 0.0);

  const double sphereR = 0.1 * (2.0 * ProGuiSim::kBoxHalf);
  double pos[3] = {0,0,0};
  int axisIdx = 0, sign = +1;
  faceToAxisSign(spawnFace_, axisIdx, sign);
  pos[axisIdx] = sign * (ProGuiSim::kBoxHalf - 0.5 * sphereR);

  lastMarker_->pos[0] = pos[0];
  lastMarker_->pos[1] = pos[1];
  lastMarker_->pos[2] = pos[2];

  return updateProbeForFace();
}

// ---------------- Layout / Gap ----------------

void ProGuiChain::applyGapLayout(double gapRatio) {
  if (!spec_) return;
  if (chain_.size() <= 1) return;
  assert(gapRatio >= 0.0 && gapRatio <= 1.0);

  const double boxEdge = 2.0 * ProGuiSim::kBoxHalf;
  const double gap = gapRatio * boxEdge;

  for (size_t i = 1; i < chain_.size(); ++i) {
    ChainEntry& e = chain_[i];
    mjsBody* b = e.specBody;
    if (!b) continue;
    const int axis = e.axis;
    const int sign = e.sign;
    const int factor = e.distanceFactor <= 0 ? 1 : e.distanceFactor;
    b->pos[0] = 0.0; b->pos[1] = 0.0; b->pos[2] = 0.0;
    double off = sign * factor * (boxEdge + gap);
    if (axis == 0) b->pos[0] = off;
    else if (axis == 1) b->pos[1] = off;
    else b->pos[2] = off;
  }
  updateJointLimitsForGap(gapRatio);

  if (probeRect_) {
    int axisIdx = 0, sign = +1;
    faceToAxisSign(spawnFace_, axisIdx, sign);
    probeRect_->pos[0] = 0;
    probeRect_->pos[1] = 0;
    probeRect_->pos[2] = 0;
    probeRect_->pos[axisIdx] = sign * (boxEdge + gap);
  }

  int old_nv = model_->nv, old_na = model_->na;
  if (mj_recompile(spec_, nullptr, model_, data_) != 0) return;
  for (int i = old_nv; i < model_->nv; ++i) data_->qvel[i] = 0;
  for (int i = old_na; i < model_->na; ++i) data_->act[i] = 0;
  sim_->applyBuildModeOptions();
  mj_forward(model_, data_);
  sim_->rebuildRenderContext();
}

void ProGuiChain::updateJointLimitsForGap(double gapRatio) {
  const double gapAbs = currentGap(gapRatio); // gap in absolute units
  for (const auto& e : chain_) {
    std::string jn = e.name + "_ball";
    if (mjsElement* jel = mjs_findElement(spec_, mjOBJ_JOINT, jn.c_str())) {
      mjsJoint* j = mjs_asJoint(jel);
      if (j && j->type == mjJNT_BALL) {
        const double a = ProGuiSim::kBoxHalf;
        const double c = 1.0 + (a > 0.0 ? (gapAbs / a) : 0.0);
        const double root2 = std::sqrt(2.0);
        double phi;
        if (c >= root2) {
          phi = mjPI / 2.0;
        } else {
          double s = c / root2;
          s = std::max(0.0, std::min(1.0, s));
          phi = std::asin(s) - (mjPI / 4.0);
        }
        const double th_min = mjPI * 5.0 / 180.0;
        const double th_max = mjPI * 80.0 / 180.0;
        phi = std::max(th_min, std::min(th_max, phi));
        double phi_deg = phi * 180.0 / mjPI;
        j->limited = mjLIMITED_TRUE;
        j->range[0] = 0.0;
        j->range[1] = phi_deg / 2;
      }
    }
  }
}

// ---------------- Queries & Refresh ----------------

Result ProGuiChain::refreshBodyIdIndexMap() {
  std::vector<std::string> names;
  names.reserve(chain_.size());
  for (auto& e : chain_) names.push_back(e.name);
  auto res = sim_->updateBodyIdIndexMap(names, bodyIdToIndex_);
  if (!res.ok()) return res;
  for (size_t i = 0; i < chain_.size(); ++i) {
    chain_[i].bodyId = mj_name2id(model_, mjOBJ_BODY, chain_[i].name.c_str());
  }
  return Result::success();
}

bool ProGuiChain::canSpawnCube() const noexcept {
  if (!spec_ || !model_ || !data_ || !sim_) {
    return false;
  }
  return mjs_findBody(spec_, "world") != nullptr;
}

bool ProGuiChain::canDeleteTarget() const noexcept {
  return canSpawnCube() && chain_.size() > 1;
}

std::optional<size_t> ProGuiChain::chainIndexForBodyId(int bodyId) const {
  auto it = bodyIdToIndex_.find(bodyId);
  if (it == bodyIdToIndex_.end()) return std::nullopt;
  return it->second;
}

std::optional<int> ProGuiChain::probeHitBodyId() const {
  int newestId = -1;
  if (!chain_.empty()) {
    newestId = mj_name2id(model_, mjOBJ_BODY, chain_.back().name.c_str());
  }
  return sim_->probeHitBodyId(probeName_, newestId);
}