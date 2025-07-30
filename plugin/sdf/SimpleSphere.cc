#include "SimpleSphere.h"

#include <cstdint>
#include <optional>
#include <utility>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {
    namespace {

        // Distance function for a sphere centered at the origin
        // point: 3D coordinates
        // attributes: radius
        static mjtNum distance(const mjtNum point[3], const mjtNum* radius) {
            // For hollow cylinder aligned along z-axis
            // Using radius[0] as outer radius and radius[0]/2 as inner radius
            // Height is set to radius[0]*2
            
            mjtNum outerRadius = radius[0];
            mjtNum innerRadius = radius[0] / 2.0;
            mjtNum halfHeight = radius[0];
            
            // Compute distance in 2D (x,y) - this is the distance from the z-axis
            mjtNum xy_dist = mju_sqrt(point[0] * point[0] + point[1] * point[1]);
            
            // Distance to the outer cylinder surface (positive outside)
            mjtNum outer_dist = xy_dist - outerRadius;
            
            // Distance to the inner cylinder surface (positive inside the hollow part)
            mjtNum inner_dist = innerRadius - xy_dist;
            
            // Distance to the top and bottom caps
            mjtNum z_dist = mju_abs(point[2]) - halfHeight;
            
            // Combine distances for the complete shape
            if (z_dist > 0) {
                // Outside the cylinder height, need to calculate corner distances
                if (outer_dist > 0) {
                    // Outside outer radius and outside height
                    return mju_sqrt(outer_dist * outer_dist + z_dist * z_dist);
                } else if (inner_dist > 0) {
                    // Inside inner radius and outside height
                    return mju_sqrt(inner_dist * inner_dist + z_dist * z_dist);
                } else {
                    // Between inner and outer radius, outside height
                    return z_dist;
                }
            } else {
                // Within the cylinder height
                // Return maximum of inner and outer distance
                return mju_max(outer_dist, inner_dist);
            }
        }

    }  // namespace

    // Factory function
    std::optional<SimpleSphere> SimpleSphere::Create(
        const mjModel* m, mjData* d, int instance) {
        // Validate attributes
        if (CheckAttr("radius", m, instance)) {
            return SimpleSphere(m, d, instance);
        }
        else {
            mju_warning("Invalid radius parameter in SimpleSphere plugin");
            return std::nullopt;
        }
    }

    // Plugin constructor
    SimpleSphere::SimpleSphere(const mjModel* m, mjData* d, int instance) {
        SdfDefault<SimpleSphereAttribute> defattribute;

        // Get attribute values from config or use defaults
        for (int i = 0; i < SimpleSphereAttribute::nattribute; i++) {
            attribute[i] = defattribute.GetDefault(
                SimpleSphereAttribute::names[i],
                mj_getPluginConfig(m, instance, SimpleSphereAttribute::names[i]));
        }
    }

    // SDF distance function implementation
    mjtNum SimpleSphere::Distance(const mjtNum point[3]) const {
        return distance(point, attribute);
    }

    // SDF gradient function implementation for SimpleSphere
    void SimpleSphere::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
        // Using numerical differentiation for the gradient of the hollow cylinder SDF
        mjtNum eps = 1e-8;
        mjtNum dist0 = Distance(point);

        mjtNum pointX[3] = {point[0]+eps, point[1], point[2]};
        mjtNum distX = Distance(pointX);
        
        mjtNum pointY[3] = {point[0], point[1]+eps, point[2]};
        mjtNum distY = Distance(pointY);
        
        mjtNum pointZ[3] = {point[0], point[1], point[2]+eps};
        mjtNum distZ = Distance(pointZ);

        grad[0] = (distX - dist0) / eps;
        grad[1] = (distY - dist0) / eps;
        grad[2] = (distZ - dist0) / eps;
    }

    // Plugin registration
    void SimpleSphere::RegisterPlugin() {
        mjpPlugin plugin;
        mjp_defaultPlugin(&plugin);

        plugin.name = "mujoco.sdf.simplesphere";
        plugin.capabilityflags |= mjPLUGIN_SDF;

        plugin.nattribute = SimpleSphereAttribute::nattribute;
        plugin.attributes = SimpleSphereAttribute::names;
        plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

        plugin.init = +[](const mjModel* m, mjData* d, int instance) {
            auto sdf_or_null = SimpleSphere::Create(m, d, instance);
            if (!sdf_or_null.has_value()) {
                return -1;
            }
            d->plugin_data[instance] = reinterpret_cast<uintptr_t>(
                new SimpleSphere(std::move(*sdf_or_null)));
            return 0;
            };
        plugin.destroy = +[](mjData* d, int instance) {
            delete reinterpret_cast<SimpleSphere*>(d->plugin_data[instance]);
            d->plugin_data[instance] = 0;
            };
        plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
            int instance) {
                auto sdf = reinterpret_cast<SimpleSphere*>(plugin_data);
                sdf->visualizer_.Reset();
            }; 
        plugin.compute =
            +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
            auto* sdf = reinterpret_cast<SimpleSphere*>(d->plugin_data[instance]);
            sdf->visualizer_.Reset();

            // Force sample points for visualization
            if (capability_bit & mjPLUGIN_SDF) {
                // Get cylinder dimensions from attributes
                mjtNum outerRadius = sdf->attribute[0];
                mjtNum halfHeight = outerRadius;
                
                // Sample grid of points in cylinder-appropriate pattern
                for (int ir = -2; ir <= 2; ir++) {
                    for (int itheta = 0; itheta < 8; itheta++) {
                        for (int iz = -2; iz <= 2; iz++) {
                            // Calculate point in cylindrical coordinates
                            mjtNum radius = outerRadius * (0.4 + 0.4 * ir);
                            mjtNum theta = itheta * (2 * mjPI / 8);
                            mjtNum z = iz * halfHeight * 0.5f;
                            
                            // Convert to Cartesian coordinates
                            mjtNum point[3] = {
                                radius * cos(theta),
                                radius * sin(theta),
                                z
                            };
                            
                            mjtNum grad[3];
                            sdf->visualizer_.AddPoint(point);
                            sdf->Gradient(grad, point);
                        }
                    }
                }
            }

            sdf->visualizer_.Next();
            };
        plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
            mjvScene* scn, int instance) {
                auto* sdf = reinterpret_cast<SimpleSphere*>(d->plugin_data[instance]);
                sdf->visualizer_.Visualize(m, d, opt, scn, instance);
            };
        plugin.sdf_distance =
            +[](const mjtNum point[3], const mjData* d, int instance) {
            auto* sdf = reinterpret_cast<SimpleSphere*>(d->plugin_data[instance]);
            return sdf->Distance(point);
            };
        plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3],
            const mjData* d, int instance) {
                auto* sdf = reinterpret_cast<SimpleSphere*>(d->plugin_data[instance]);
                sdf->visualizer_.AddPoint(point);
                sdf->Gradient(gradient, point);
            };
        plugin.sdf_staticdistance =
            +[](const mjtNum point[3], const mjtNum* attributes) {
            return distance(point, attributes);
            };
        plugin.sdf_aabb =
            +[](mjtNum aabb[6], const mjtNum* attributes) {
            // Cylinder is aligned along z-axis
            mjtNum radius = attributes[0];
            aabb[0] = aabb[1] = aabb[2] = 0;  // min bounds
            aabb[3] = aabb[4] = aabb[5] = 2*radius;   // max bounds
            };
        plugin.sdf_attribute =
            +[](mjtNum attribute[], const char* name[], const char* value[]) {
            SdfDefault<SimpleSphereAttribute> defattribute;
            defattribute.GetDefaults(attribute, name, value);
            };

        mjp_registerPlugin(&plugin);
    }

}  // namespace mujoco::plugin::sdf