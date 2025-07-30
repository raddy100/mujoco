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

#ifndef MUJOCO_PLUGIN_SDF_SIMPLESPHERE_H_
#define MUJOCO_PLUGIN_SDF_SIMPLESPHERE_H_

#include <optional>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

    struct SimpleSphereAttribute {
        static constexpr int nattribute = 1;
        static constexpr char const* names[nattribute] = { "radius" };
        static constexpr mjtNum defaults[nattribute] = { 0.5 };
    };

    class SimpleSphere {
    public:
        // Creates a new SimpleSphere instance or returns null on failure.
        static std::optional<SimpleSphere> Create(const mjModel* m, mjData* d, int instance);
        SimpleSphere(SimpleSphere&&) = default;
        ~SimpleSphere() = default;

        mjtNum Distance(const mjtNum point[3]) const;
        void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

        static void RegisterPlugin();

        // Visualizer to show gradient steps (used by the mjvPlugin visualizer)
        SdfVisualizer visualizer_;

        // Sphere radius
        mjtNum attribute[SimpleSphereAttribute::nattribute];

    private:
        SimpleSphere(const mjModel* m, mjData* d, int instance);
    };

}  // namespace mujoco::plugin::sdf

#endif  // MUJOCO_PLUGIN_SDF_SIMPLESPHERE_H_// Copyright 2023 DeepMind Technologies Limited
