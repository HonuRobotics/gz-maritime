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
#include <atomic>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/rendering/Events.hh>

#include <gz/sim/components/Name.hh>

#include "UnderwaterOgre2Bridge.hh"
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
/// untouched. Once the renderer draws the water it marks the world with the
/// UnderwaterRendering component, so a reference camera does not apply the
/// water a second time. If it cannot (another render engine, or Ogre's
/// pieces not as this was written for), the world is not marked and the
/// reference camera draws the water.
class UnderwaterRendering
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override
  {
    this->owner = _entity;
    this->enabled = _sdf->Get<bool>("enabled", true).first;
    // On and off while running, also where the model is included and its
    // <enabled> cannot be set
    std::string world = "default";
    if (const auto *name =
            _ecm.Component<components::Name>(worldEntity(_ecm)))
    {
      world = name->Data();
    }
    std::function<void(const msgs::Boolean &)> onEnable =
        [this](const msgs::Boolean &_msg) { this->enabled = _msg.data(); };
    this->node.Subscribe("/world/" + world + "/underwater_rendering/enable",
                         onEnable);
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
    // Mark the world while the renderer draws the water, and only then:
    // otherwise the reference camera does. In the GUI's own entity manager
    // this is a no-op that nobody reads.
    const Entity world = worldEntity(_ecm);
    const auto *mark = world != kNullEntity ?
        _ecm.Component<components::UnderwaterRendering>(world) : nullptr;
    const bool drawing = this->installed.load() && this->enabled.load();
    if (world != kNullEntity && drawing && !mark)
    {
      _ecm.CreateComponent(world,
                           components::UnderwaterRendering(this->owner));
    }
    else if (mark && !drawing)
    {
      _ecm.RemoveComponent<components::UnderwaterRendering>(world);
    }
    const std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->sampler.Valid())
    {
      this->water.reset();
      return;
    }
    const auto &o = this->sampler.Data();
    UwWater v{};
    for (int c = 0; c < 3; ++c)
    {
      v.attenuation[c] = static_cast<float>(o.attenuation[c]);
      v.backscatter[c] = static_cast<float>(o.backscatter[c]);
      v.veil[c] = static_cast<float>(o.veil[c]);
      v.diffuse[c] = static_cast<float>(o.diffuseAttenuation[c]);
    }
    v.level = static_cast<float>(o.waterLevel);
    v.daylight = static_cast<float>(o.daylight);
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
      const auto version = reinterpret_cast<UwVersionFn>(
          dlsym(this->handle, "uw_ogre2_version"));
      if (!version || version() != UW_OGRE2_BRIDGE_VERSION)
      {
        gzerr << "UnderwaterRendering: the Ogre bridge is not the version "
              << "this plugin was built with; the reference camera draws "
              << "the water instead\n";
        this->failed = true;
        return;
      }
      this->install = reinterpret_cast<UwInstallFn>(
          dlsym(this->handle, "uw_ogre2_install"));
      this->set = reinterpret_cast<UwSetFn>(
          dlsym(this->handle, "uw_ogre2_set"));
      this->uninstall = reinterpret_cast<UwUninstallFn>(
          dlsym(this->handle, "uw_ogre2_uninstall"));
      this->release = reinterpret_cast<UwReleaseFn>(
          dlsym(this->handle, "uw_ogre2_release"));
      if (!this->install || !this->set || !this->uninstall || !this->release ||
          !this->install(this->piecesDir.c_str()))
      {
        gzerr << "UnderwaterRendering: not installed; the reference camera "
              << "draws the water instead\n";
        this->failed = true;
        return;
      }
      this->installed = true;
    }
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->set(this->water && this->enabled ? &*this->water : nullptr);
    if (!this->announced || *this->announced != this->water.has_value())
    {
      gzmsg << "UnderwaterRendering: "
            << (this->water ? "drawing the world's water"
                            : "no water optics in the world; nothing drawn")
            << "\n";
      this->announced = this->water.has_value();
    }
  }

  /// \brief Going, e.g. with its model: the bridge draws nothing more. It
  /// cannot be uninstalled from here, off the render thread.
  public: ~UnderwaterRendering() override
  {
    if (this->installed && this->release)
      this->release();
  }

  private: void OnTeardown()
  {
    if (this->installed && this->uninstall)
      this->uninstall();
    this->installed = false;
  }

  private: water_optics::WaterOpticsSampler sampler;
  private: std::mutex mutex;
  private: std::optional<UwWater> water;
  private: std::string piecesDir;
  private: void *handle{nullptr};
  private: UwInstallFn install{nullptr};
  private: UwSetFn set{nullptr};
  private: UwUninstallFn uninstall{nullptr};
  private: UwReleaseFn release{nullptr};

  /// \brief The entity this plugin is on, named by the world's mark
  private: Entity owner{kNullEntity};

  /// \brief Whether to draw the water: <enabled>, then the enable topic
  private: std::atomic<bool> enabled{true};

  /// \brief For the enable topic
  private: transport::Node node;
  /// \brief Set on the render thread, read on the simulation thread
  private: std::atomic<bool> installed{false};
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
