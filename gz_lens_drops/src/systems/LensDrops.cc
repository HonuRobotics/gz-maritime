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
#include "LensDrops.hh"

#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include "Ogre2LensDropsBridge.hh"
#include "gz/sim/lens_drops/DropMap.hh"
#include "gz/sim/lens_drops/Drops.hh"
#include "gz/sim/wind/WindSampler.hh"

namespace
{
/// \brief The bridge ABI this system speaks.
constexpr int kBridgeAbi = 2;

/// \brief The dlopen'd bridge: the library handle and its functions.
struct BridgeApi
{
  /// \brief dlopen handle of liblens-drops-ogre2-bridge.so.
  void *handle{nullptr};

  /// \brief lens_drops_ogre2_abi_version.
  int (*abiVersion)(void){nullptr};

  /// \brief lens_drops_ogre2_create.
  lens_drops_pass_t (*create)(const char *, const char *, int, int){nullptr};

  /// \brief lens_drops_ogre2_attach.
  int (*attach)(lens_drops_pass_t, void *){nullptr};

  /// \brief lens_drops_ogre2_set_map.
  int (*setMap)(lens_drops_pass_t, const float *, int, int, float){nullptr};

  /// \brief lens_drops_ogre2_destroy.
  void (*destroy)(lens_drops_pass_t, void *){nullptr};

  /// \brief True once the library, every function and the right ABI
  /// version were found.
  bool loaded{false};
};

/// \brief Load the bridge once, on the render thread, after the render
/// engine. The only point where Ogre symbols enter the process from here.
/// \return The bridge; `loaded` false if it could not be loaded.
const BridgeApi &LoadBridge()
{
  static BridgeApi api;
  static std::once_flag flag;
  std::call_once(flag, [&]()
  {
    api.handle = dlopen("liblens-drops-ogre2-bridge.so", RTLD_NOW | RTLD_LOCAL);
    if (!api.handle)
    {
      const char *error = dlerror();
      gzerr << "[LensDrops] failed to dlopen liblens-drops-ogre2-bridge.so: "
            << (error ? error : "unknown") << "\n";
      return;
    }
    api.abiVersion = reinterpret_cast<int (*)(void)>(
        dlsym(api.handle, "lens_drops_ogre2_abi_version"));
    api.create = reinterpret_cast<lens_drops_pass_t (*)(const char *,
        const char *, int, int)>(dlsym(api.handle, "lens_drops_ogre2_create"));
    api.attach = reinterpret_cast<int (*)(lens_drops_pass_t, void *)>(
        dlsym(api.handle, "lens_drops_ogre2_attach"));
    api.setMap = reinterpret_cast<int (*)(lens_drops_pass_t, const float *,
        int, int, float)>(dlsym(api.handle, "lens_drops_ogre2_set_map"));
    api.destroy = reinterpret_cast<void (*)(lens_drops_pass_t, void *)>(
        dlsym(api.handle, "lens_drops_ogre2_destroy"));
    if (!api.abiVersion || !api.create || !api.attach || !api.setMap ||
        !api.destroy)
    {
      gzerr << "[LensDrops] liblens-drops-ogre2-bridge.so is missing one or "
            << "more functions\n";
      return;
    }
    if (api.abiVersion() != kBridgeAbi)
    {
      gzerr << "[LensDrops] liblens-drops-ogre2-bridge.so speaks ABI "
            << api.abiVersion() << ", this system needs " << kBridgeAbi
            << "\n";
      return;
    }
    api.loaded = true;
  });
  return api;
}

/// \brief Read a whole file.
/// \param[in] _path The file.
/// \return Its text, or empty if it cannot be opened.
std::string ReadFile(const std::string &_path)
{
  std::ifstream in(_path);
  if (!in)
    return std::string();
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

/// \brief Where the shaders are: GZ_LENS_DROPS_SHADER_PATH if set and not
/// empty, else where this package installs them.
/// \return The directory.
std::string ShaderDir()
{
  const char *env = std::getenv("GZ_LENS_DROPS_SHADER_PATH");
  if (env && *env)
    return env;
  return LENS_DROPS_SHADER_DIR;
}

/// \brief A message value as a number.
/// \param[in] _value The value.
/// \param[out] _out The number.
/// \return False for a type that is not a number or a value not finite.
bool Number(const gz::msgs::Any &_value, double &_out)
{
  double d{0.0};
  switch (_value.type())
  {
    case gz::msgs::Any::DOUBLE: d = _value.double_value(); break;
    case gz::msgs::Any::INT32: d = _value.int_value(); break;
    case gz::msgs::Any::BOOLEAN: d = _value.bool_value() ? 1.0 : 0.0; break;
    default: return false;
  }
  if (!std::isfinite(d))
    return false;
  _out = d;
  return true;
}

/// \brief Add a number to a Param message.
/// \param[in,out] _msg The message.
/// \param[in] _key The key.
/// \param[in] _value The value.
void SetDouble(gz::msgs::Param &_msg, const std::string &_key, double _value)
{
  gz::msgs::Any &any = (*_msg.mutable_params())[_key];
  any.set_type(gz::msgs::Any::DOUBLE);
  any.set_double_value(_value);
}
}  // namespace

namespace gz::sim::maritime
{
/// \brief Private data of LensDrops.
class LensDropsPrivate
{
  /// \brief A message on the set topic. Transport thread.
  /// \param[in] _msg The message.
  public: void OnSet(const msgs::Param &_msg);

  /// \brief Add the render pass once, then hand it the drops. Render
  /// thread.
  public: void OnPreRender();

  /// \brief Take the pass off the camera before the scene goes. Render
  /// thread.
  public: void OnTeardown();

  /// \brief Guards drops, attached, seed, covered and simTime.
  public: std::mutex mutex;

  /// \brief The drops on the lens.
  public: lens_drops::Drops drops;

  /// \brief True once the pass is on the camera.
  public: bool attached{false};

  /// \brief The seed, before the camera's name is mixed in.
  public: std::uint32_t seed{0};

  /// \brief Share of the last map with water on it, from the render
  /// thread.
  public: double covered{0.0};

  /// \brief Simulation time of the last step, for the mask's stamp.
  public: std::chrono::steady_clock::duration simTime{0};

  /// \brief Map size over image size. Set in Configure.
  public: double mapScale{0.5};

  /// \brief True to publish the mask. Set in Configure.
  public: bool publishMask{false};

  /// \brief The drop map. Render thread only; made on attach.
  public: std::unique_ptr<lens_drops::DropMap> map;

  /// \brief Generation of the drops last drawn. Render thread only.
  public: std::uint64_t drawn{0};

  /// \brief True once a map was drawn. Render thread only.
  public: bool drewOnce{false};

  /// \brief The mask, reused. Render thread only.
  public: std::vector<std::uint8_t> mask;

  /// \brief Publisher of the mask.
  public: transport::Node::Publisher maskPub;

  /// \brief What the world does to the lens this step, from the sensor's
  /// pose, its motion and the wind. Sim thread only.
  /// \param[in] _info The step.
  /// \param[in] _ecm The entity component manager.
  /// \return The environment.
  public: lens_drops::Environment Environment(const UpdateInfo &_info,
                                             const EntityComponentManager &_ecm);

  /// \brief The sensor entity. Set in Configure.
  public: Entity sensor{kNullEntity};

  /// \brief The link the sensor is on. Set in Configure.
  public: Entity link{kNullEntity};

  /// \brief True to take the world's wind. Set in Configure.
  public: bool useWind{true};

  /// \brief The world's wind. Sim thread only.
  public: wind::WindSampler sampler;

  /// \brief The sensor's world position at the last step. Sim thread only.
  public: math::Vector3d lastPosition;

  /// \brief True once lastPosition holds a position. Sim thread only.
  public: bool havePosition{false};

  /// \brief The camera's name in the rendering scene. Set in Configure.
  public: std::string cameraName;

  /// \brief The base of the topics. Set in Configure.
  public: std::string base;

  /// \brief True once Configure found a camera and a topic.
  public: bool valid{false};

  /// \brief The pass. Render thread only.
  public: lens_drops_pass_t pass{nullptr};

  /// \brief The camera the pass is on. Render thread only.
  public: rendering::CameraPtr camera;

  /// \brief True once the render path gave up. Render thread only.
  public: bool failed{false};

  /// \brief Frames spent looking for the camera. Render thread only.
  public: int tries{0};

  /// \brief When the state was last published. Sim thread only.
  public: std::chrono::steady_clock::duration lastState{0};

  /// \brief True once the state was published. Sim thread only.
  public: bool published{false};

  /// \brief Connection to the PreRender event.
  public: common::ConnectionPtr preRenderConn;

  /// \brief Connection to the RenderTeardown event.
  public: common::ConnectionPtr teardownConn;

  /// \brief Publisher of the state.
  public: transport::Node::Publisher statePub;

  /// \brief Transport node. Last, so it is destroyed first.
  public: transport::Node node;
};

//////////////////////////////////////////////////
LensDrops::LensDrops()
  : dataPtr(std::make_unique<LensDropsPrivate>())
{
}

//////////////////////////////////////////////////
LensDrops::~LensDrops()
{
  // No call into the bridge here: this may run on another thread than the
  // render thread. A pass the teardown did not take off is left to the
  // process's exit, with the scene.
  this->dataPtr->preRenderConn.reset();
  this->dataPtr->teardownConn.reset();
}

//////////////////////////////////////////////////
void LensDrops::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  if (!_ecm.Component<components::Camera>(_entity))
  {
    gzerr << "[LensDrops] must be inside a <sensor type=\"camera\">; it does "
          << "nothing here\n";
    return;
  }

  this->dataPtr->sensor = _entity;
  if (auto *parent = _ecm.Component<components::ParentEntity>(_entity))
    this->dataPtr->link = parent->Data();
  this->dataPtr->useWind = _sdf->Get<bool>("wind", true).first;

  lens_drops::Params params;
  params.rainRate = _sdf->Get<double>("rain_rate", params.rainRate).first;
  params.exposure = _sdf->Get<double>("exposure", params.exposure).first;
  params.focus = _sdf->Get<double>("focus", params.focus).first;
  params.dryTime = _sdf->Get<double>("dry_time", params.dryTime).first;
  params.contactAngle =
      _sdf->Get<double>("contact_angle", params.contactAngle).first;
  params.slideRadius =
      _sdf->Get<double>("slide_radius", params.slideRadius).first;
  const double mapScale = _sdf->Get<double>("map_scale", 0.5).first;
  this->dataPtr->mapScale =
      std::isfinite(mapScale) ? std::clamp(mapScale, 0.125, 1.0) : 0.5;
  this->dataPtr->publishMask = _sdf->Get<bool>("mask", false).first;
  std::uint32_t seed = _sdf->Get<unsigned int>("seed", 0u).first;
  if (seed == 0u)
    seed = std::random_device{}();

  const std::string scoped = scopedName(_entity, _ecm);
  this->dataPtr->cameraName =
      removeParentScope(scopedName(_entity, _ecm, "::", false), "::");
  this->dataPtr->base =
      transport::TopicUtils::AsValidTopic("/" + scoped + "/lens_drops");
  if (this->dataPtr->base.empty())
  {
    gzerr << "[LensDrops] no valid topic for [" << scoped << "]\n";
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->seed = seed;
    this->dataPtr->drops = lens_drops::Drops(
        seed ^ lens_drops::Hash(this->dataPtr->cameraName), params);
  }

  this->dataPtr->node.Subscribe(this->dataPtr->base + "/set",
      &LensDropsPrivate::OnSet, this->dataPtr.get());
  this->dataPtr->statePub = this->dataPtr->node.Advertise<msgs::Param>(
      this->dataPtr->base + "/state");
  if (this->dataPtr->publishMask)
  {
    this->dataPtr->maskPub = this->dataPtr->node.Advertise<msgs::Image>(
        this->dataPtr->base + "/mask");
  }

  this->dataPtr->preRenderConn = _eventMgr.Connect<events::PreRender>(
      std::bind(&LensDropsPrivate::OnPreRender, this->dataPtr.get()));
  this->dataPtr->teardownConn = _eventMgr.Connect<events::RenderTeardown>(
      std::bind(&LensDropsPrivate::OnTeardown, this->dataPtr.get()));

  this->dataPtr->valid = true;
  gzmsg << "[LensDrops] on [" << this->dataPtr->cameraName << "], set on ["
        << this->dataPtr->base << "/set]\n";
}

//////////////////////////////////////////////////
void LensDrops::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  if (!this->dataPtr->valid)
    return;

  if (_info.simTime < this->dataPtr->lastState)
    this->dataPtr->lastState = std::chrono::steady_clock::duration::zero();

  // Gravity, air and rain on the lens, from the sensor's pose and motion.
  std::optional<lens_drops::Environment> env;
  if (!_info.paused)
    env = this->dataPtr->Environment(_info, _ecm);

  lens_drops::Params params;
  lens_drops::Environment used;
  std::size_t count{0};
  std::size_t sliding{0};
  double covered{0.0};
  bool attached{false};
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (env)
      this->dataPtr->drops.SetEnvironment(*env);
    if (!_info.paused)
    {
      this->dataPtr->drops.Step(
          std::chrono::duration<double>(_info.dt).count());
      this->dataPtr->simTime = _info.simTime;
    }
    params = this->dataPtr->drops.Parameters();
    used = this->dataPtr->drops.Env();
    count = this->dataPtr->drops.List().size();
    sliding = this->dataPtr->drops.Sliding();
    covered = this->dataPtr->covered;
    attached = this->dataPtr->attached;
  }

  if (_info.paused)
    return;
  if (this->dataPtr->published &&
      _info.simTime - this->dataPtr->lastState < std::chrono::milliseconds(200))
  {
    return;
  }

  msgs::Param msg;
  *msg.mutable_header()->mutable_stamp() = convert<msgs::Time>(_info.simTime);
  SetDouble(msg, "rain_rate", params.rainRate);
  SetDouble(msg, "exposure", params.exposure);
  SetDouble(msg, "focus", params.focus);
  SetDouble(msg, "dry_time", params.dryTime);
  SetDouble(msg, "contact_angle", params.contactAngle);
  SetDouble(msg, "slide_radius", params.slideRadius);
  SetDouble(msg, "covered", covered);
  SetDouble(msg, "flux", used.flux);
  SetDouble(msg, "gravity_x", used.gravityX);
  SetDouble(msg, "gravity_y", used.gravityY);
  msgs::Any &slidingValue = (*msg.mutable_params())["sliding"];
  slidingValue.set_type(msgs::Any::INT32);
  slidingValue.set_int_value(static_cast<int>(sliding));
  msgs::Any &countValue = (*msg.mutable_params())["count"];
  countValue.set_type(msgs::Any::INT32);
  countValue.set_int_value(static_cast<int>(count));
  msgs::Any &attachedValue = (*msg.mutable_params())["attached"];
  attachedValue.set_type(msgs::Any::BOOLEAN);
  attachedValue.set_bool_value(attached);
  this->dataPtr->statePub.Publish(msg);

  this->dataPtr->lastState = _info.simTime;
  this->dataPtr->published = true;
}

//////////////////////////////////////////////////
void LensDrops::Reset(const UpdateInfo &, EntityComponentManager &)
{
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->drops.Reset(
        this->dataPtr->seed ^ lens_drops::Hash(this->dataPtr->cameraName));
  }
  this->dataPtr->lastState = std::chrono::steady_clock::duration::zero();
  this->dataPtr->published = false;
  this->dataPtr->havePosition = false;
}

//////////////////////////////////////////////////
lens_drops::Environment LensDropsPrivate::Environment(
    const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  lens_drops::Environment env;
  if (this->link == kNullEntity)
    return env;

  // The sensor's world pose. A Gazebo camera looks along +X; the image's
  // right is -Y and its down is -Z.
  math::Pose3d pose = worldPose(this->link, _ecm);
  if (auto *local = _ecm.Component<components::Pose>(this->sensor))
    pose = pose * local->Data();
  const math::Quaterniond &q = pose.Rot();
  const math::Vector3d right = q.RotateVector(-math::Vector3d::UnitY);
  const math::Vector3d down = q.RotateVector(-math::Vector3d::UnitZ);
  const math::Vector3d forward = q.RotateVector(math::Vector3d::UnitX);
  const math::Vector3d position = pose.Pos();

  // Its velocity from the last step; none on the first step or after a
  // jump, such as a teleport.
  const double dt = std::chrono::duration<double>(_info.dt).count();
  math::Vector3d velocity = math::Vector3d::Zero;
  if (this->havePosition && dt > 0.0 &&
      (position - this->lastPosition).Length() <= 50.0)
  {
    velocity = (position - this->lastPosition) / dt;
  }
  this->lastPosition = position;
  this->havePosition = true;

  math::Vector3d wind = math::Vector3d::Zero;
  if (this->useWind && this->sampler.Sync(_ecm))
    wind = this->sampler.At(position, _info.simTime);

  const math::Vector3d gravity(0.0, 0.0, -1.0);
  env.gravityX = gravity.Dot(right);
  env.gravityY = gravity.Dot(down);
  const math::Vector3d air = wind - velocity;
  env.airX = air.Dot(right);
  env.airY = air.Dot(down);
  // Rain falls at its terminal speed and drifts with the wind; what hits
  // the lens is what moves into it.
  const math::Vector3d rain =
      wind + math::Vector3d(0.0, 0.0, -lens_drops::kRainTerminalSpeed);
  const math::Vector3d relative = rain - velocity;
  env.flux = std::max(0.0, -relative.Dot(forward)) /
      lens_drops::kRainTerminalSpeed;
  return env;
}

//////////////////////////////////////////////////
void LensDropsPrivate::OnSet(const msgs::Param &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  lens_drops::Params params = this->drops.Parameters();
  bool clear = false;
  std::map<std::string, double> add;
  for (const auto &[key, value] : _msg.params())
  {
    double number{0.0};
    const bool known = key == "rain_rate" || key == "exposure" ||
        key == "focus" || key == "dry_time" || key == "clear" ||
        key == "contact_angle" || key == "slide_radius" ||
        key == "add_x" || key == "add_y" || key == "add_radius";
    if (!known)
    {
      gzwarn << "[LensDrops] unknown key [" << key << "] on " << this->base
             << "/set\n";
      continue;
    }
    if (!Number(value, number))
    {
      gzwarn << "[LensDrops] [" << key << "] is not a finite number\n";
      continue;
    }
    if (key == "rain_rate")
      params.rainRate = number;
    else if (key == "exposure")
      params.exposure = number;
    else if (key == "focus")
      params.focus = number;
    else if (key == "dry_time")
      params.dryTime = number;
    else if (key == "contact_angle")
      params.contactAngle = number;
    else if (key == "slide_radius")
      params.slideRadius = number;
    else if (key == "clear")
      clear = number != 0.0;
    else
      add[key] = number;
  }
  this->drops.SetParams(params);
  if (clear)
    this->drops.Clear();
  if (!add.empty())
  {
    if (add.size() != 3)
    {
      gzwarn << "[LensDrops] add_x, add_y and add_radius go together; no "
             << "drop added\n";
    }
    else
    {
      lens_drops::Drop drop;
      drop.x = add["add_x"];
      drop.y = add["add_y"];
      drop.radius = std::clamp(add["add_radius"], 0.0,
          lens_drops::kMaxMergedRadius);
      this->drops.Add(drop);
    }
  }
}

//////////////////////////////////////////////////
void LensDropsPrivate::OnPreRender()
{
  if (!this->valid || this->failed)
    return;

  if (!this->pass)
  {
    rendering::ScenePtr scene = rendering::sceneFromFirstRenderEngine();
    if (!scene || !scene->IsInitialized())
      return;
    this->camera = std::dynamic_pointer_cast<rendering::Camera>(
        scene->SensorByName(this->cameraName));
    if (!this->camera)
    {
      // The sensors system makes the camera a few frames in; keep looking.
      if (++this->tries == 300)
      {
        gzwarn << "[LensDrops] camera [" << this->cameraName
               << "] not found in the scene yet\n";
      }
      return;
    }
    if (scene->Engine()->Name() != "ogre2")
    {
      gzerr << "[LensDrops] needs the ogre2 render engine, not ["
            << scene->Engine()->Name() << "]; no drops are drawn\n";
      this->failed = true;
      this->camera.reset();
      return;
    }
    const BridgeApi &api = LoadBridge();
    if (!api.loaded)
    {
      this->failed = true;
      this->camera.reset();
      return;
    }
    const std::string dir = ShaderDir();
    const std::string vsPath = dir + "/lens_drops_vs_330.glsl";
    const std::string fsPath = dir + "/lens_drops_fs_330.glsl";
    const std::string vs = ReadFile(vsPath);
    const std::string fs = ReadFile(fsPath);
    if (vs.empty() || fs.empty())
    {
      gzerr << "[LensDrops] shader missing or empty: ["
            << (vs.empty() ? vsPath : fsPath) << "]\n";
      this->failed = true;
      this->camera.reset();
      return;
    }
    const auto side = [this](unsigned int _pixels)
    {
      return std::clamp(static_cast<int>(std::lround(_pixels * this->mapScale)),
          1, lens_drops::kMaxMapSize);
    };
    const int mapWidth = side(this->camera->ImageWidth());
    const int mapHeight = side(this->camera->ImageHeight());
    this->pass = api.create(vs.c_str(), fs.c_str(), mapWidth, mapHeight);
    if (!this->pass || !api.attach(this->pass, this->camera.get()))
    {
      api.destroy(this->pass, nullptr);
      this->pass = nullptr;
      gzerr << "[LensDrops] could not add the render pass to ["
            << this->cameraName << "]\n";
      this->failed = true;
      this->camera.reset();
      return;
    }
    this->map = std::make_unique<lens_drops::DropMap>(mapWidth, mapHeight);
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      this->attached = true;
      lens_drops::Params params = this->drops.Parameters();
      params.aspect = static_cast<double>(this->camera->ImageWidth()) /
          std::max(1u, this->camera->ImageHeight());
      this->drops.SetParams(params);
    }
    gzmsg << "[LensDrops] render pass on [" << this->cameraName << "], map "
          << mapWidth << "x" << mapHeight << "\n";
  }

  // Draw the map again only when the drops changed. The copy is taken
  // under the lock; the drawing, the upload and the mask are not.
  std::optional<lens_drops::Drops> copy;
  std::chrono::steady_clock::duration stamp{0};
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    if (this->drewOnce && this->drops.Generation() == this->drawn)
      return;
    copy.emplace(this->drops);
    stamp = this->simTime;
    this->drawn = this->drops.Generation();
  }
  this->drewOnce = true;
  this->map->Render(*copy);
  const BridgeApi &api = LoadBridge();
  api.setMap(this->pass, this->map->Data().data(), this->map->Width(),
      this->map->Height(), static_cast<float>(copy->Parameters().focus));

  const double texels =
      static_cast<double>(this->map->Width()) * this->map->Height();
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->covered = static_cast<double>(this->map->Covered()) / texels;
  }

  if (this->publishMask)
  {
    this->map->Mask(this->mask);
    msgs::Image image;
    *image.mutable_header()->mutable_stamp() = convert<msgs::Time>(stamp);
    image.set_width(static_cast<unsigned int>(this->map->Width()));
    image.set_height(static_cast<unsigned int>(this->map->Height()));
    image.set_step(static_cast<unsigned int>(this->map->Width()));
    image.set_pixel_format_type(msgs::PixelFormatType::L_INT8);
    image.set_data(this->mask.data(), this->mask.size());
    this->maskPub.Publish(image);
  }
}

//////////////////////////////////////////////////
void LensDropsPrivate::OnTeardown()
{
  if (this->pass)
  {
    const BridgeApi &api = LoadBridge();
    if (api.loaded)
      api.destroy(this->pass, this->camera.get());
  }
  this->pass = nullptr;
  this->camera.reset();
  this->failed = true;
  const std::lock_guard<std::mutex> lock(this->mutex);
  this->attached = false;
}
}  // namespace gz::sim::maritime

GZ_ADD_PLUGIN(gz::sim::maritime::LensDrops,
              gz::sim::System,
              gz::sim::maritime::LensDrops::ISystemConfigure,
              gz::sim::maritime::LensDrops::ISystemPostUpdate,
              gz::sim::maritime::LensDrops::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::maritime::LensDrops,
                    "gz::sim::maritime::LensDrops")
