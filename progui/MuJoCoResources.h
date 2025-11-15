#pragma once
/**
 * @file MuJoCoResources.h
 * @brief RAII wrappers for MuJoCo core resources (mjModel, mjData, mjSpec).
 *
 * These wrappers are optional: existing ProGuiSim and ProGuiChain classes
 * operate on non-owning raw pointers passed from external initialization.
 * Introducing these types enables explicit ownership when integrating the
 * refactored code into an application that loads and unloads MuJoCo models.
 *
 * Usage:
 *   auto owned = MuJoCoResources::loadFromXML(xmlPath);
 *   if (!owned) { ... error ... }
 *   ProGuiSim sim(owned.model.get(), owned.data.get(), owned.spec.get(), ...);
 *
 * If an external system already manages lifetime (e.g., another engine),
 * continue supplying raw pointers without these wrappers.
 *
 * Exception Safety:
 *  - loadFromXML returns std::optional<Owned> avoiding throwing on failures.
 *  - Destruction is noexcept.
 *
 * Thread-Safety:
 *  - Not thread-safe; creation/destruction expected on main thread.
 */

#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include <optional>
#include <string>
#include <utility>
#include <memory>

struct MuJoCoModelDeleter {
  void operator()(mjModel* m) const noexcept {
    if (m) mj_deleteModel(m);
  }
};

struct MuJoCoDataDeleter {
  void operator()(mjData* d) const noexcept {
    if (d) mj_deleteData(d);
  }
};

struct MuJoCoSpecDeleter {
  void operator()(mjSpec* s) const noexcept {
    if (s) mj_deleteSpec(s);
  }
};

/**
 * @brief Aggregate of owned MuJoCo objects.
 *
 * spec may be null when loading directly from XML without editable spec.
 */
struct MuJoCoOwned {
  std::unique_ptr<mjModel, MuJoCoModelDeleter> model;
  std::unique_ptr<mjData,  MuJoCoDataDeleter>  data;
  std::unique_ptr<mjSpec,  MuJoCoSpecDeleter>  spec; // optional
  std::string                                   error; // populated on load failure
  bool ok() const noexcept { return model && data && error.empty(); }
};

/**
 * @brief Helper namespace producing owned MuJoCo resources.
 */
namespace MuJoCoResources {

  /**
   * @brief Load MuJoCo model + data from XML file path.
   * @param xmlPath Path to XML model file.
   * @return std::optional<MuJoCoOwned> (empty if load failed); MuJoCoOwned.error contains diagnostic.
   */
  inline std::optional<MuJoCoOwned> loadFromXML(const std::string& xmlPath) {
    char error[1024] = {0};
    mjModel* m = mj_loadXML(xmlPath.c_str(), nullptr, error, sizeof(error));
    if (!m) {
      MuJoCoOwned fail{};
      fail.error = error;
      return std::optional<MuJoCoOwned>(std::in_place, std::move(fail));
    }
    mjData* d = mj_makeData(m);
    if (!d) {
      MuJoCoOwned fail{};
      fail.error = "mj_makeData failed";
      mj_deleteModel(m);
      return std::optional<MuJoCoOwned>(std::in_place, std::move(fail));
    }
    MuJoCoOwned owned{
      std::unique_ptr<mjModel, MuJoCoModelDeleter>(m),
      std::unique_ptr<mjData,  MuJoCoDataDeleter>(d),
      nullptr,
      {}
    };
    return owned;
  }

  /**
   * @brief Create an empty editable spec (advanced usage).
   * @return std::unique_ptr<mjSpec, MuJoCoSpecDeleter>
   */
  inline std::unique_ptr<mjSpec, MuJoCoSpecDeleter> makeEmptySpec() {
    mjSpec* s = mj_makeSpec();
    return std::unique_ptr<mjSpec, MuJoCoSpecDeleter>(s);
  }

} // namespace MuJoCoResources