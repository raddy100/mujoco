#include "SimpleBox.h"

#include <cstdint>
#include <optional>
#include <utility>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {
namespace {

    // Distance function for a box centered at the origin
    static mjtNum distance(const mjtNum point[3], const mjtNum dimensions[3]) {
        // Inside/outside test with analytical SDF
        mjtNum dx = mju_abs(point[0]) - dimensions[0];
        mjtNum dy = mju_abs(point[1]) - dimensions[1];
        mjtNum dz = mju_abs(point[2]) - dimensions[2];

        // Compute the outside distance
        mjtNum outside = mju_sqrt(
            mju_max(dx, 0) * mju_max(dx, 0) +
            mju_max(dy, 0) * mju_max(dy, 0) +
            mju_max(dz, 0) * mju_max(dz, 0)
        );

        // Compute the inside distance (negative)
        mjtNum inside = mju_min(mju_max(dx, mju_max(dy, dz)), 0);

        // Return the signed distance
        return outside + inside;
    }

}  // namespace

// Factory function
std::optional<SimpleBox> SimpleBox::Create(
    const mjModel* m, mjData* d, int instance) {
  // Validate attributes
  if (CheckAttr("sizex", m, instance) && 
      CheckAttr("sizey", m, instance) && 
      CheckAttr("sizez", m, instance)) {
    return SimpleBox(m, d, instance);
  } else {
    mju_warning("Invalid size parameters in SimpleBox plugin");
    return std::nullopt;
  }
}

// Plugin constructor
SimpleBox::SimpleBox(const mjModel* m, mjData* d, int instance) {
  SdfDefault<SimpleBoxAttribute> defattribute;

  // Get attribute values from config or use defaults
  for (int i = 0; i < SimpleBoxAttribute::nattribute; i++) {
    attribute[i] = defattribute.GetDefault(
        SimpleBoxAttribute::names[i],
        mj_getPluginConfig(m, instance, SimpleBoxAttribute::names[i]));
  }
}

// SDF distance function implementation
mjtNum SimpleBox::Distance(const mjtNum point[3]) const {
  return distance(point, attribute);
}
void SimpleBox::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
    mjtNum dx = mju_abs(point[0]) - attribute[0];
    mjtNum dy = mju_abs(point[1]) - attribute[1];
    mjtNum dz = mju_abs(point[2]) - attribute[2];

    // Initialize gradient
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 0;

    // INSIDE BOX CASE
    if (dx <= 0 && dy <= 0 && dz <= 0) {
        // Find the axis with the maximum penetration (closest to surface)
        if (dx >= dy && dx >= dz) {
            grad[0] = (point[0] >= 0) ? 1.0 : -1.0;
        }
        else if (dy >= dx && dy >= dz) {
            grad[1] = (point[1] >= 0) ? 1.0 : -1.0;
        }
        else {
            grad[2] = (point[2] >= 0) ? 1.0 : -1.0;
        }
        return;
    }

    // OUTSIDE BOX CASE - calculate gradient from closest point on box
    mjtNum px = mju_clip(point[0], -attribute[0], attribute[0]);
    mjtNum py = mju_clip(point[1], -attribute[1], attribute[1]);
    mjtNum pz = mju_clip(point[2], -attribute[2], attribute[2]);

    // Vector from nearest point on box to query point
    mjtNum vx = point[0] - px;
    mjtNum vy = point[1] - py;
    mjtNum vz = point[2] - pz;

    // Special case: if on a face, set gradient to face normal
    if (mju_abs(vx) < 1e-10 && mju_abs(vy) < 1e-10 && mju_abs(vz) > 1e-10) {
        grad[2] = (vz > 0) ? 1.0 : -1.0;
        return;
    }
    if (mju_abs(vx) < 1e-10 && mju_abs(vz) < 1e-10 && mju_abs(vy) > 1e-10) {
        grad[1] = (vy > 0) ? 1.0 : -1.0;
        return;
    }
    if (mju_abs(vy) < 1e-10 && mju_abs(vz) < 1e-10 && mju_abs(vx) > 1e-10) {
        grad[0] = (vx > 0) ? 1.0 : -1.0;
        return;
    }

    // Normalize the vector
    mjtNum length = mju_sqrt(vx * vx + vy * vy + vz * vz);
    if (length > 1e-10) {
        grad[0] = vx / length;
        grad[1] = vy / length;
        grad[2] = vz / length;
    }
}

// Plugin registration
void SimpleBox::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.simplebox";
  plugin.capabilityflags |= mjPLUGIN_SDF;

  plugin.nattribute = SimpleBoxAttribute::nattribute;
  plugin.attributes = SimpleBoxAttribute::names;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto sdf_or_null = SimpleBox::Create(m, d, instance);
    if (!sdf_or_null.has_value()) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
        new SimpleBox(std::move(*sdf_or_null)));
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<SimpleBox*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                     int instance) {
    auto sdf = reinterpret_cast<SimpleBox*>(plugin_data);
    sdf->visualizer_.Reset();
  };
  plugin.compute =
      +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* sdf = reinterpret_cast<SimpleBox*>(d->plugin_data[instance]);
        sdf->visualizer_.Next();
      };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                         mjvScene* scn, int instance) {
    auto* sdf = reinterpret_cast<SimpleBox*>(d->plugin_data[instance]);
    sdf->visualizer_.Visualize(m, d, opt, scn, instance);
  };
  plugin.sdf_distance =
      +[](const mjtNum point[3], const mjData* d, int instance) {
        auto* sdf = reinterpret_cast<SimpleBox*>(d->plugin_data[instance]);
        return sdf->Distance(point);
      };
  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3],
                        const mjData* d, int instance) {
    auto* sdf = reinterpret_cast<SimpleBox*>(d->plugin_data[instance]);
    sdf->visualizer_.AddPoint(point);
    sdf->Gradient(gradient, point);
  };
  plugin.sdf_staticdistance =
      +[](const mjtNum point[3], const mjtNum* attributes) {
        return distance(point, attributes);
      };
  plugin.sdf_aabb =
      +[](mjtNum aabb[6], const mjtNum* attributes) {
        // Box is centered at origin, so AABB is just the dimensions
        aabb[0] = 0;
        aabb[1] = 0;
        aabb[2] = 0;
        aabb[3] = attributes[0];   // max x
        aabb[4] =attributes[1];   // max y
        aabb[5] = attributes[2];   // max z


      };
  plugin.sdf_attribute =
      +[](mjtNum attribute[], const char* name[], const char* value[]) {
        SdfDefault<SimpleBoxAttribute> defattribute;
        defattribute.GetDefaults(attribute, name, value);
      };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sdf

