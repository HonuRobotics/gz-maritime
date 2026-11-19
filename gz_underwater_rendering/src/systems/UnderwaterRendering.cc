/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <dlfcn.h>

#include <array>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/rendering/Events.hh>

#include "gz/sim/components/UnderwaterRendering.hh"
#include "gz/sim/water_optics/WaterOpticsSampler.hh"

namespace gz::sim::maritime
{
/// \brief The renderer draws the world's water into every camera.
///
/// A visual plugin: put it on a visual of the world (the `underwater_optics`
/// model has one) and every PBS-shaded surface any camera renders, in the
/// server's sensors and in the GUI, is seen through the world's water
/// (gz_water_optics), per fragment, on the GPU, by the same model as the
/// reference camera (gz_underwater_camera). Cameras above the water are
/// untouched. It marks the world with the UnderwaterRendering component, so
/// a reference camera does not apply the water a second time.
class UnderwaterRendering
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  public: void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &,
                         EntityComponentManager &, EventManager &_eventMgr) override
  {
    this->sceneUpdate = _eventMgr.Connect<events::SceneUpdate>(
        [this]() { this->OnSceneUpdate(); });
    this->teardown = _eventMgr.Connect<events::RenderTeardown>(
        [this]() { this->OnTeardown(); });
    const char *dir = std::getenv("GZ_UNDERWATER_HLMS_DIR");
    this->piecesDir = dir ? dir : UW_PIECES_DIR;
  }

  public: void PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    this->sampler.Sync(_ecm);
    // Mark the world: the renderer draws the water. In the GUI's own
    // entity manager this is a no-op that nobody reads.
    const Entity world = worldEntity(_ecm);
    if (world != kNullEntity &&
        !_ecm.Component<components::UnderwaterRendering>(world))
    {
      _ecm.CreateComponent(world, components::UnderwaterRendering());
    }
    const std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->sampler.Valid())
    {
      this->water.reset();
      return;
    }
    const auto &o = this->sampler.Data();
    std::array<float, 13> v{};
    for (int c = 0; c < 3; ++c)
    {
      v[c] = static_cast<float>(o.attenuation[c]);
      v[3 + c] = static_cast<float>(o.backscatter[c]);
      v[6 + c] = static_cast<float>(o.veil[c]);
      v[9 + c] = static_cast<float>(o.diffuseAttenuation[c]);
    }
    v[12] = static_cast<float>(o.waterLevel);
    this->water = v;
  }

  private: void OnSceneUpdate()
  {
    if (this->failed)
      return;
    if (!this->installed)
    {
      auto scene = rendering::sceneFromFirstRenderEngine();
      if (!scene)
        return;
      if (scene->Engine()->Name() != "ogre2")
      {
        gzerr << "UnderwaterRendering: needs the ogre2 render engine, not "
              << scene->Engine()->Name() << "\n";
        this->failed = true;
        return;
      }
      this->handle = dlopen("libunderwater-ogre2-bridge.so",
                            RTLD_NOW | RTLD_LOCAL);
      if (!this->handle)
      {
        gzerr << "UnderwaterRendering: " << dlerror() << "\n";
        this->failed = true;
        return;
      }
      this->install = reinterpret_cast<int (*)(const char *)>(
          dlsym(this->handle, "uw_ogre2_install"));
      this->set = reinterpret_cast<void (*)(const float *)>(
          dlsym(this->handle, "uw_ogre2_set"));
      this->uninstall = reinterpret_cast<void (*)()>(
          dlsym(this->handle, "uw_ogre2_uninstall"));
      if (!this->install || !this->set || !this->uninstall ||
          !this->install(this->piecesDir.c_str()))
      {
        this->failed = true;
        return;
      }
      this->installed = true;
    }
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->set(this->water ? this->water->data() : nullptr);
    if (!this->announced || *this->announced != this->water.has_value())
    {
      gzmsg << "UnderwaterRendering: "
            << (this->water ? "drawing the world's water"
                            : "no water optics in the world; nothing drawn")
            << "\n";
      this->announced = this->water.has_value();
    }
  }

  private: void OnTeardown()
  {
    if (this->installed && this->uninstall)
      this->uninstall();
    this->installed = false;
  }

  private: water_optics::WaterOpticsSampler sampler;
  private: std::mutex mutex;
  private: std::optional<std::array<float, 13>> water;
  private: std::string piecesDir;
  private: void *handle{nullptr};
  private: int (*install)(const char *){nullptr};
  private: void (*set)(const float *){nullptr};
  private: void (*uninstall)(){nullptr};
  private: bool installed{false};
  private: bool failed{false};
  private: std::optional<bool> announced;
  private: common::ConnectionPtr sceneUpdate;
  private: common::ConnectionPtr teardown;
};
}  // namespace gz::sim::maritime

GZ_ADD_PLUGIN(gz::sim::maritime::UnderwaterRendering,
              gz::sim::System,
              gz::sim::maritime::UnderwaterRendering::ISystemConfigure,
              gz::sim::maritime::UnderwaterRendering::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::maritime::UnderwaterRendering,
                    "gz::sim::maritime::UnderwaterRendering")
