#pragma once

// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_PLUGIN_SDF_SIMPLEBOX_H_
#define MUJOCO_PLUGIN_SDF_SIMPLEBOX_H_

#include <optional>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

struct SimpleBoxAttribute {
  static constexpr int nattribute = 3;
  static constexpr char const* names[nattribute] = {"sizex", "sizey", "sizez"};
  static constexpr mjtNum defaults[nattribute] = {0.5, 0.5, 0.5};
};

class SimpleBox {
 public:
  // Creates a new SimpleBox instance or returns null on failure.
  static std::optional<SimpleBox> Create(const mjModel* m, mjData* d, int instance);
  SimpleBox(SimpleBox&&) = default;
  ~SimpleBox() = default;

  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

  // Visualizer to show gradient steps (used by the mjvPlugin visualizer)
  SdfVisualizer visualizer_;

  // Box dimensions (half-extents)
  mjtNum attribute[SimpleBoxAttribute::nattribute];

 private:
  SimpleBox(const mjModel* m, mjData* d, int instance);
};

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_SIMPLEBOX_H_
