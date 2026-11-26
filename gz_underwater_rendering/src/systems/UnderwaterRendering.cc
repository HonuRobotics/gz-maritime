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
#include <vector>

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
#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/waves/Eval.hh"
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
  public: void Configure(const Entity &,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &, EventManager &_eventMgr) override
  {
    // <quality>: how finely the lamps' beams are marched, how far marine
    // snow is drawn and how many lamps are used. low, medium or high.
    if (_sdf && _sdf->HasElement("quality"))
    {
      const auto q = _sdf->Get<std::string>("quality");
      if (q == "low")
        this->quality = 0.0f;
      else if (q == "medium")
        this->quality = 1.0f;
      else if (q == "high")
        this->quality = 2.0f;
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
    this->sampler.Sync(_ecm);
    // Mark the world while the renderer draws the water, and only then:
    // otherwise the reference camera does. In the GUI's own entity manager
    // this is a no-op that nobody reads.
    const Entity world = worldEntity(_ecm);
    const bool marked = world != kNullEntity &&
        _ecm.Component<components::UnderwaterRendering>(world);
    const bool drawing = this->installed.load();
    if (world != kNullEntity && drawing && !marked)
      _ecm.CreateComponent(world, components::UnderwaterRendering());
    else if (marked && !drawing)
      _ecm.RemoveComponent<components::UnderwaterRendering>(world);
    const std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->sampler.Valid())
    {
      this->water.reset();
      return;
    }
    const auto &o = this->sampler.Data();
    // How high the waves are, for the caustics: Hs = 4 sigma of the surface
    // at 64 points on a golden-angle spiral out to 50 m, which spans many
    // wavelengths and phases, smoothed over the steps.
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (const auto *f = _ecm.Component<components::Wavefield>(world);
        f && f->Data().simulation)
    {
      double sum = 0, sum2 = 0;
      const int n = 64;
      for (int i = 0; i < n; ++i)
      {
        const double r = 50.0 * std::sqrt((i + 0.5) / n);
        const double a = i * 2.399963229728653;
        const double e = waves::SurfaceElevation(f->Data(), r * std::cos(a),
                                                 r * std::sin(a), t);
        sum += e;
        sum2 += e * e;
      }
      const double mean = sum / n;
      const double hs = 4.0 * std::sqrt(std::max(0.0, sum2 / n - mean * mean));
      this->waveHeight = this->waveHeightSet
          ? 0.98 * this->waveHeight + 0.02 * hs : hs;
      this->waveHeightSet = true;
    }
    // The wave over each camera the renderer drew with: depth is measured
    // below the water's surface there, as the reference camera does.
    this->elevations.clear();
    if (const auto *f = _ecm.Component<components::Wavefield>(world);
        f && f->Data().simulation)
    {
      for (std::size_t i = 0; i + 2 < this->cameras.size(); i += 3)
      {
        this->elevations.push_back(this->cameras[i]);
        this->elevations.push_back(static_cast<float>(
            waves::SurfaceElevation(f->Data(), this->cameras[i + 1],
                                    this->cameras[i + 2], t)));
      }
    }
    std::array<float, 18> v{};
    for (int c = 0; c < 3; ++c)
    {
      v[c] = static_cast<float>(o.attenuation[c]);
      v[3 + c] = static_cast<float>(o.backscatter[c]);
      v[6 + c] = static_cast<float>(o.veil[c]);
      v[9 + c] = static_cast<float>(o.diffuseAttenuation[c]);
    }
    v[12] = static_cast<float>(o.waterLevel);
    v[13] = static_cast<float>(o.daylight);
    // Caustics need waves: full strength from a 0.5 m sea.
    v[14] = static_cast<float>(o.caustics *
                               std::clamp(this->waveHeight / 0.5, 0.0, 1.0));
    v[15] = static_cast<float>(o.marineSnow);
    v[16] = static_cast<float>(std::fmod(t, 1000.0));
    v[17] = this->quality;
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
      this->listCameras = reinterpret_cast<int (*)(float *, int)>(
          dlsym(this->handle, "uw_ogre2_cameras"));
      this->setElevations = reinterpret_cast<void (*)(const float *, int)>(
          dlsym(this->handle, "uw_ogre2_set_elevations"));
      if (!this->install || !this->set || !this->uninstall ||
          !this->listCameras || !this->setElevations ||
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
    this->set(this->water ? this->water->data() : nullptr);
    // Where the cameras are, for PreUpdate to ask the wave field, and what
    // it last answered.
    this->cameras.resize(3 * kMaxCameras);
    this->cameras.resize(3 * this->listCameras(this->cameras.data(),
                                               kMaxCameras));
    this->setElevations(this->elevations.data(),
                        static_cast<int>(this->elevations.size() / 2));
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
  private: std::optional<std::array<float, 18>> water;
  private: std::string piecesDir;
  private: void *handle{nullptr};
  private: int (*install)(const char *){nullptr};
  private: void (*set)(const float *){nullptr};
  private: void (*uninstall)(){nullptr};
  private: int (*listCameras)(float *, int){nullptr};
  private: void (*setElevations)(const float *, int){nullptr};
  /// \brief The cameras the renderer drew with: id, x, y each (the bridge's
  /// uw_ogre2_cameras). Guarded by mutex.
  private: std::vector<float> cameras;
  /// \brief The wave over each of them: id, height each. Guarded by mutex.
  private: std::vector<float> elevations;
  private: static constexpr int kMaxCameras = 32;
  /// \brief Set on the render thread, read on the simulation thread
  private: std::atomic<bool> installed{false};
  private: bool failed{false};
  private: std::optional<bool> announced;
  /// \brief <quality>: 0 low, 1 medium, 2 high.
  private: float quality{1.0f};
  private: double waveHeight{0.0};
  private: bool waveHeightSet{false};
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
