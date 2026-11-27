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

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/rendering/Events.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>

#include "UnderwaterOgre2Bridge.hh"
#include "WorldMark.hh"
#include "gz/sim/components/UnderwaterRendering.hh"
#include "gz/sim/components/WaterOptics.hh"
#include "gz/sim/components/Wavefield.hh"
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
    this->worldName = world;
    std::function<void(const msgs::Boolean &)> onEnable =
        [this](const msgs::Boolean &_msg) { this->enabled = _msg.data(); };
    this->node.Subscribe("/world/" + world + "/underwater_rendering/enable",
                         onEnable);
    // <quality>: how finely the lamps' beams are marched, how far marine
    // snow is drawn and how many lamps are used. low, medium or high.
    if (_sdf && _sdf->HasElement("quality"))
    {
      const auto q = _sdf->Get<std::string>("quality");
      if (q == "low")
        this->quality = 0;
      else if (q == "medium")
        this->quality = 1;
      else if (q == "high")
        this->quality = 2;
      else
        gzerr << "UnderwaterRendering: <quality> is low, medium or high, "
              << "not [" << q << "]; using medium\n";
    }
    this->sceneUpdate = _eventMgr.Connect<events::SceneUpdate>(
        [this]() { this->OnSceneUpdate(); });
    this->teardown = _eventMgr.Connect<events::RenderTeardown>(
        [this]() { this->OnTeardown(); });
    const char *dir = std::getenv("GZ_UNDERWATER_HLMS_DIR");
    this->piecesDir = dir ? dir : UW_PIECES_DIR;
  }

  public: void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    const Entity world = worldEntity(_ecm);
    this->SyncOptics(_ecm, world);

    // Mark the world while the renderer draws the water, and only then:
    // otherwise the reference camera does. In the GUI's own entity manager
    // this is a no-op that nobody reads.
    UpdateWorldMark(_ecm, this->owner,
                    this->installed.load() && this->enabled.load());
    const std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->sampler.Valid())
    {
      this->water.reset();
      return;
    }
    const auto &o = this->sampler.Data();
    // How high the waves are, for the caustics: Hs = 4 sigma of the surface
    // at 64 points on a golden-angle spiral out to 50 m around the cameras,
    // which spans many wavelengths and phases. The surface is the sampler's:
    // in the GUI too, where the replicated wave field has no engine, it
    // builds one from the recipe. Smoothed over 0.5 s of simulation time,
    // whatever the step.
    const double t = std::chrono::duration<double>(_info.simTime).count();
    // Only for caustics: with none, the default, nothing to sample
    if (o.caustics > 0.0 && _ecm.Component<components::Wavefield>(world))
    {
      const math::Vector3d centre = this->cameraCentre;
      double sum = 0, sum2 = 0;
      const int n = 64;
      for (int i = 0; i < n; ++i)
      {
        const double r = 50.0 * std::sqrt((i + 0.5) / n);
        const double a = i * 2.399963229728653;
        // The surface's height there: the depth of a point at z = 0
        const double e = this->sampler.Depth(
            math::Vector3d(centre.X() + r * std::cos(a),
                           centre.Y() + r * std::sin(a), 0.0),
            _info.simTime) - o.waterLevel;
        sum += e;
        sum2 += e * e;
      }
      const double mean = sum / n;
      const double hs = 4.0 * std::sqrt(std::max(0.0, sum2 / n - mean * mean));
      const double dt = t - this->waveHeightTime;
      if (!this->waveHeightSet || dt < 0.0)
        this->waveHeight = hs;
      else
        this->waveHeight += (hs - this->waveHeight) * (1.0 - std::exp(-dt / 0.5));
      this->waveHeightSet = true;
      this->waveHeightTime = t;
    }
    else
    {
      this->waveHeight = 0.0;
      this->waveHeightSet = false;
    }
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
    // Caustics need waves: full strength from a 0.5 m sea.
    v.caustics = static_cast<float>(o.caustics *
                                    std::clamp(this->waveHeight / 0.5, 0.0, 1.0));
    v.snow = static_cast<float>(o.marineSnow);
    v.time = t;
    v.quality = this->quality;
    this->water = v;
  }

  /// \brief Catch up with the world's water. A GUI can miss it: the first
  /// state may come before this library registered the component, and an
  /// unchanged component is not sent again. Then the world's state is
  /// asked for once, and read until the component arrives.
  /// \param[in] _ecm This process's entity component manager.
  /// \param[in] _world The world entity.
  private: void SyncOptics(const EntityComponentManager &_ecm, Entity _world)
  {
    const bool here = _world != kNullEntity &&
        _ecm.Component<components::WaterOptics>(_world);
    if (!here && _world != kNullEntity && !this->askedState)
    {
      this->askedState = true;
      std::shared_ptr<Pulled> into = this->pulled;
      std::function<void(const msgs::SerializedStepMap &, const bool)> cb =
          [into](const msgs::SerializedStepMap &_rep, const bool _ok)
          {
            if (!_ok)
              return;
            auto ecm = std::make_unique<EntityComponentManager>();
            ecm->SetState(_rep.state());
            const std::lock_guard<std::mutex> lock(into->mutex);
            into->ecm = std::move(ecm);
          };
      this->node.Request("/world/" + this->worldName + "/state", cb);
    }
    if (!here)
    {
      const std::lock_guard<std::mutex> lock(this->pulled->mutex);
      if (this->pulled->ecm)
      {
        this->sampler.Sync(*this->pulled->ecm);
        return;
      }
    }
    this->sampler.Sync(_ecm);
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
    // Where the cameras are, for the wave height around them
    math::Vector3d centre;
    int cameras = 0;
    if (auto scene = rendering::sceneFromFirstRenderEngine())
    {
      for (unsigned int i = 0; i < scene->SensorCount(); ++i)
      {
        if (auto cam = std::dynamic_pointer_cast<rendering::Camera>(
                scene->SensorByIndex(i)))
        {
          centre += cam->WorldPosition();
          ++cameras;
        }
      }
    }
    const std::lock_guard<std::mutex> lock(this->mutex);
    if (cameras > 0)
      this->cameraCentre = centre / cameras;
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

  /// \brief The world's state, asked for once, while this process's
  /// entity manager has no water optics. Shared with the reply's callback.
  private: struct Pulled
  {
    std::mutex mutex;
    std::unique_ptr<EntityComponentManager> ecm;
  };
  private: std::shared_ptr<Pulled> pulled{std::make_shared<Pulled>()};
  private: bool askedState{false};
  private: std::string worldName{"default"};

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
  /// \brief <quality>: 0 low, 1 medium, 2 high.
  private: int quality{1};
  /// \brief The waves' significant height around the cameras, smoothed
  private: double waveHeight{0.0};
  private: bool waveHeightSet{false};

  /// \brief The simulation time of waveHeight, s
  private: double waveHeightTime{0.0};

  /// \brief The cameras' mean position, from the render thread; guarded by
  /// mutex
  private: math::Vector3d cameraCentre;
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
