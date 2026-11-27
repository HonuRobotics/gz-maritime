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

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/param.pb.h>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/GraphicsAPI.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>
#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

#include "Ogre2LensDropsBridge.hh"
#include "gz/sim/lens_drops/DropMap.hh"
#include "gz/sim/lens_drops/Drops.hh"
#include "gz/sim/wind/WindSampler.hh"

namespace
{
/// \brief Standard gravity, m/s^2: the drops' gravity is in g.
constexpr double kStandardGravity = 9.80665;

/// \brief The dlopen'd bridge: the library handle and its functions, typed
/// by the bridge's own declarations so the compiler checks them.
struct BridgeApi
{
  /// \brief dlopen handle of liblens-drops-ogre2-bridge.so.
  void *handle{nullptr};

  /// \brief The bridge's functions.
  decltype(&lens_drops_ogre2_abi_version) abiVersion{nullptr};
  decltype(&lens_drops_ogre2_create) create{nullptr};
  decltype(&lens_drops_ogre2_attach) attach{nullptr};
  decltype(&lens_drops_ogre2_destroy) destroy{nullptr};

  /// \brief True once the library, every function and the right ABI
  /// version were found.
  bool loaded{false};
};

//////////////////////////////////////////////////
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
    api.abiVersion = reinterpret_cast<decltype(api.abiVersion)>(
        dlsym(api.handle, "lens_drops_ogre2_abi_version"));
    api.create = reinterpret_cast<decltype(api.create)>(
        dlsym(api.handle, "lens_drops_ogre2_create"));
    api.attach = reinterpret_cast<decltype(api.attach)>(
        dlsym(api.handle, "lens_drops_ogre2_attach"));
    api.destroy = reinterpret_cast<decltype(api.destroy)>(
        dlsym(api.handle, "lens_drops_ogre2_destroy"));
    if (!api.abiVersion || !api.create || !api.attach || !api.destroy)
    {
      gzerr << "[LensDrops] liblens-drops-ogre2-bridge.so is missing one or "
            << "more functions\n";
      return;
    }
    if (api.abiVersion() != LENS_DROPS_OGRE2_ABI)
    {
      gzerr << "[LensDrops] liblens-drops-ogre2-bridge.so speaks ABI "
            << api.abiVersion() << ", this system needs "
            << LENS_DROPS_OGRE2_ABI << "\n";
      return;
    }
    api.loaded = true;
  });
  return api;
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
/// \brief Where the shaders are: where this package installs them, or
/// GZ_LENS_DROPS_SHADER_PATH, if set and not empty, for development.
/// \return The directory.
std::string ShaderDir()
{
  const char *env = std::getenv("GZ_LENS_DROPS_SHADER_PATH");
  if (env && *env)
    return env;
  return LENS_DROPS_SHADER_DIR;
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
/// \brief Add an integer to a Param message.
/// \param[in,out] _msg The message.
/// \param[in] _key The key.
/// \param[in] _value The value.
void SetInt(gz::msgs::Param &_msg, const std::string &_key, int _value)
{
  gz::msgs::Any &any = (*_msg.mutable_params())[_key];
  any.set_type(gz::msgs::Any::INT32);
  any.set_int_value(_value);
}

//////////////////////////////////////////////////
/// \brief Add a flag to a Param message.
/// \param[in,out] _msg The message.
/// \param[in] _key The key.
/// \param[in] _value The value.
void SetBool(gz::msgs::Param &_msg, const std::string &_key, bool _value)
{
  gz::msgs::Any &any = (*_msg.mutable_params())[_key];
  any.set_type(gz::msgs::Any::BOOLEAN);
  any.set_bool_value(_value);
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

  /// \brief Put the render pass on the camera once it is in the scene.
  /// Render thread.
  public: void OnSceneUpdate();

  /// \brief Let go of the pass before the scene goes. Render thread.
  public: void OnTeardown();

  /// \brief Let go of the pass: it never calls back once this returns.
  /// Any thread.
  public: void Release();

  /// \brief What the pass asks for right before the camera draws a frame.
  /// Render thread.
  /// \param[in] _context This.
  /// \param[in] _stamp The frame's simulation time, s.
  /// \param[out] _focus 0 blurred to 1 sharp.
  /// \param[out] _enabled Whether the pass draws.
  /// \return The map if it was drawn again, else null.
  public: static const float *Pull(void *_context, double _stamp,
                                   float *_focus, int *_enabled);

  /// \brief Whether the next frame needs the pass: the lens wet and the
  /// drops enabled. The bridge asks it before the frame's passes are
  /// chained. Render thread.
  /// \param[in] _context This.
  /// \return Non-zero if so.
  public: static int Wanted(void *_context);

  /// \brief What the world does to the lens this step, from the sensor's
  /// pose, its link's motion, the world's gravity and its wind. Sim thread.
  /// \param[in] _info The step.
  /// \param[in] _ecm The entity component manager.
  /// \return The environment.
  public: lens_drops::Environment Environment(const UpdateInfo &_info,
                                             const EntityComponentManager &_ecm);

  /// \brief Guards drops, enabled, attached, seed, covered and draws.
  public: std::mutex mutex;

  /// \brief The drops on the lens.
  public: lens_drops::Drops drops;

  /// \brief Whether to draw the drops: <enabled>, then the set topic.
  public: bool enabled{true};

  /// \brief True once the pass is on the camera.
  public: bool attached{false};

  /// \brief The seed, before the camera's name is mixed in.
  public: std::uint32_t seed{1};

  /// \brief Share of the last map with water on it.
  public: double covered{0.0};

  /// \brief Maps drawn so far.
  public: std::uint64_t draws{0};

  /// \brief Whether the pass was wanted the last time the renderer asked,
  /// as it chained the frame's passes: on, or off with a dry lens or the
  /// drops disabled.
  public: bool drawing{false};

  /// \brief The drop map's width, texels. Set in Configure.
  public: int mapWidth{1};

  /// \brief The drop map's height, texels. Set in Configure.
  public: int mapHeight{1};

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

  /// \brief The sensor entity. Set in Configure.
  public: Entity sensor{kNullEntity};

  /// \brief The link the sensor is on. Set in Configure.
  public: Entity link{kNullEntity};

  /// \brief True to take the world's wind. Set in Configure.
  public: bool useWind{true};

  /// \brief The world's wind. Sim thread only.
  public: wind::WindSampler sampler;

  /// \brief Take the camera's names and the topics, and start the drops,
  /// once the sensor is under the world: a model spawned at run time is
  /// not yet in Configure, and its names would then lack the world's.
  /// \param[in] _ecm The entity component manager.
  /// \return True once named.
  public: bool Name(const EntityComponentManager &_ecm);

  /// \brief The parameters and seed from the SDF, until Name starts the
  /// drops with them.
  public: lens_drops::Params params;

  /// \brief Drops refused on the set topic so far. Under the mutex.
  public: std::uint64_t refused{0};

  /// \brief The refusal count at which to warn next. Under the mutex.
  public: std::uint64_t nextRefusalWarning{1};

  /// \brief True once Configure found a camera. Sim thread only.
  public: bool configured{false};

  /// \brief The camera's name in the rendering scene. Set by Name.
  public: std::string cameraName;

  /// \brief The base of the topics. Set by Name.
  public: std::string base;

  /// \brief True once named: the topics are up and the drops started.
  /// Written after cameraName and base, read on the render thread.
  public: std::atomic<bool> valid{false};

  /// \brief The pass's handle. Guarded by passMutex.
  public: lens_drops_pass_t pass{nullptr};

  /// \brief Guards pass, between the render thread and the destructor.
  public: std::mutex passMutex;

  /// \brief True once the render path gave up. Render thread only.
  public: bool failed{false};

  /// \brief Frames spent looking for the camera. Render thread only.
  public: int tries{0};

  /// \brief When the state was last published. Sim thread only.
  public: std::chrono::steady_clock::duration lastState{0};

  /// \brief True once the state was published. Sim thread only.
  public: bool published{false};

  /// \brief Connection to the SceneUpdate event.
  public: common::ConnectionPtr sceneUpdateConn;

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
  // No more scene updates, then the pass lets go of this, waiting for a
  // pull in progress: from here nothing on the render thread touches this.
  // The pass stays with its camera, which goes with the sensor.
  this->dataPtr->sceneUpdateConn.reset();
  this->dataPtr->teardownConn.reset();
  this->dataPtr->Release();
}

//////////////////////////////////////////////////
void LensDropsPrivate::Release()
{
  const std::lock_guard<std::mutex> lock(this->passMutex);
  if (this->pass)
  {
    const BridgeApi &api = LoadBridge();
    if (api.loaded)
      api.destroy(this->pass);
  }
  this->pass = nullptr;
}

//////////////////////////////////////////////////
void LensDrops::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  const auto *camera = _ecm.Component<components::Camera>(_entity);
  if (!camera || !camera->Data().CameraSensor())
  {
    gzerr << "[LensDrops] must be inside a <sensor type=\"camera\">; it does "
          << "nothing here\n";
    return;
  }

  this->dataPtr->sensor = _entity;
  if (auto *parent = _ecm.Component<components::ParentEntity>(_entity))
  {
    this->dataPtr->link = parent->Data();
    // The link's velocity, worked out by gz-sim
    Link(this->dataPtr->link).EnableVelocityChecks(_ecm, true);
  }
  this->dataPtr->useWind = _sdf->Get<bool>("wind", true).first;

  lens_drops::Params params;
  params.rainRate = _sdf->Get<double>("rain_rate", params.rainRate).first;
  params.exposure = _sdf->Get<double>("exposure", params.exposure).first;
  params.focus = _sdf->Get<double>("focus", params.focus).first;
  params.dryTime = _sdf->Get<double>("dry_time", params.dryTime).first;
  params.contactAngle =
      _sdf->Get<double>("contact_angle", params.contactAngle).first;
  if (params.contactAngle > lens_drops::kMaxContactAngle)
  {
    gzwarn << "[LensDrops] <contact_angle> " << params.contactAngle
           << " is past pi/2, which a drop map cannot hold; using pi/2\n";
  }
  params.slideRadius =
      _sdf->Get<double>("slide_radius", params.slideRadius).first;

  // The image's shape, and the map's, from the sensor: known before
  // anything renders, so a headless run and a rendered one agree
  const sdf::Camera &image = *camera->Data().CameraSensor();
  const double width = std::max(1u, image.ImageWidth());
  const double height = std::max(1u, image.ImageHeight());
  params.aspect = width / height;
  double mapScale = _sdf->Get<double>("map_scale", 0.5).first;
  mapScale = std::isfinite(mapScale) ? std::clamp(mapScale, 0.125, 1.0) : 0.5;
  const auto side = [mapScale](double _pixels)
  {
    return std::clamp(static_cast<int>(std::lround(_pixels * mapScale)), 1,
                      lens_drops::kMaxMapSize);
  };
  this->dataPtr->mapWidth = side(width);
  this->dataPtr->mapHeight = side(height);
  // Drops under a texel across cannot be seen: not kept
  params.minRadius = 0.5 / this->dataPtr->mapHeight;

  this->dataPtr->enabled = _sdf->Get<bool>("enabled", true).first;
  this->dataPtr->publishMask = _sdf->Get<bool>("mask", false).first;
  std::uint32_t seed = _sdf->Get<unsigned int>("seed", 1u).first;
  if (seed == 0u)
  {
    seed = std::random_device{}();
    gzmsg << "[LensDrops] <seed> 0: drew " << seed << "\n";
  }

  this->dataPtr->seed = seed;
  this->dataPtr->params = params;
  this->dataPtr->configured = true;

  // Scene updates, not PreRender: a PreRender connection makes the sensors
  // system update the scene on every physics step. Nothing is drawn until
  // the camera is named.
  this->dataPtr->sceneUpdateConn = _eventMgr.Connect<events::SceneUpdate>(
      std::bind(&LensDropsPrivate::OnSceneUpdate, this->dataPtr.get()));
  this->dataPtr->teardownConn = _eventMgr.Connect<events::RenderTeardown>(
      std::bind(&LensDropsPrivate::OnTeardown, this->dataPtr.get()));

  // A camera in the world file is under the world now; a spawned one is
  // named on the first step that finds it there
  this->dataPtr->Name(_ecm);
}

//////////////////////////////////////////////////
bool LensDropsPrivate::Name(const EntityComponentManager &_ecm)
{
  if (this->valid)
    return true;
  // Up the parents to the world
  Entity top = this->sensor;
  while (const auto *parent = _ecm.Component<components::ParentEntity>(top))
    top = parent->Data();
  if (!_ecm.Component<components::World>(top))
    return false;

  const std::string scoped = scopedName(this->sensor, _ecm);
  const std::string name =
      removeParentScope(scopedName(this->sensor, _ecm, "::", false), "::");
  const std::string topic =
      transport::TopicUtils::AsValidTopic("/" + scoped + "/lens_drops");
  if (topic.empty())
  {
    gzerr << "[LensDrops] no valid topic for [" << scoped << "]\n";
    this->configured = false;
    return false;
  }
  this->cameraName = name;
  this->base = topic;
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->drops = lens_drops::Drops(
        this->seed ^ lens_drops::Hash(this->cameraName), this->params);
  }
  this->node.Subscribe(this->base + "/set", &LensDropsPrivate::OnSet, this);
  this->statePub = this->node.Advertise<msgs::Param>(this->base + "/state");
  if (this->publishMask)
    this->maskPub = this->node.Advertise<msgs::Image>(this->base + "/mask");

  this->valid = true;
  gzmsg << "[LensDrops] on [" << this->cameraName << "], seed " << this->seed
        << ", set on [" << this->base << "/set]\n";
  return true;
}

//////////////////////////////////////////////////
void LensDrops::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  if (!this->dataPtr->configured || _info.paused)
    return;
  if (!this->dataPtr->Name(_ecm))
    return;

  if (_info.simTime < this->dataPtr->lastState)
    this->dataPtr->lastState = std::chrono::steady_clock::duration::zero();

  // With no rain and a dry lens nothing happens: the world is asked only
  // for the state, five times a second
  const bool due = !this->dataPtr->published ||
      _info.simTime - this->dataPtr->lastState >= std::chrono::milliseconds(200);
  bool idle{false};
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    idle = this->dataPtr->drops.Parameters().rainRate <= 0.0 &&
        this->dataPtr->drops.List().empty() &&
        this->dataPtr->drops.Trails().empty();
  }
  std::optional<lens_drops::Environment> env;
  if (!idle || due)
    env = this->dataPtr->Environment(_info, _ecm);
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (env)
      this->dataPtr->drops.SetEnvironment(*env);
    this->dataPtr->drops.Step(std::chrono::duration<double>(_info.dt).count());
  }

  if (!due)
    return;

  msgs::Param msg;
  *msg.mutable_header()->mutable_stamp() = convert<msgs::Time>(_info.simTime);
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    const lens_drops::Params &params = this->dataPtr->drops.Parameters();
    const lens_drops::Environment &used = this->dataPtr->drops.Env();
    SetDouble(msg, "rain_rate", params.rainRate);
    SetDouble(msg, "exposure", params.exposure);
    SetDouble(msg, "focus", params.focus);
    SetDouble(msg, "dry_time", params.dryTime);
    SetDouble(msg, "contact_angle", params.contactAngle);
    SetDouble(msg, "slide_radius", params.slideRadius);
    SetDouble(msg, "covered", this->dataPtr->covered);
    SetDouble(msg, "flux", used.flux);
    SetDouble(msg, "gravity_x", used.gravityX);
    SetDouble(msg, "gravity_y", used.gravityY);
    SetInt(msg, "sliding", static_cast<int>(this->dataPtr->drops.Sliding()));
    SetInt(msg, "count",
           static_cast<int>(this->dataPtr->drops.List().size()));
    SetInt(msg, "draws", static_cast<int>(this->dataPtr->draws));
    SetBool(msg, "enabled", this->dataPtr->enabled);
    SetBool(msg, "attached", this->dataPtr->attached);
    SetBool(msg, "drawing", this->dataPtr->drawing);
  }
  this->dataPtr->statePub.Publish(msg);

  this->dataPtr->lastState = _info.simTime;
  this->dataPtr->published = true;
}

//////////////////////////////////////////////////
void LensDrops::Reset(const UpdateInfo &, EntityComponentManager &)
{
  if (!this->dataPtr->valid)
    return;
  {
    const std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->drops.Reset(
        this->dataPtr->seed ^ lens_drops::Hash(this->dataPtr->cameraName));
  }
  this->dataPtr->lastState = std::chrono::steady_clock::duration::zero();
  this->dataPtr->published = false;
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
  const math::Pose3d pose = worldPose(this->sensor, _ecm);
  const math::Quaterniond &q = pose.Rot();
  const math::Vector3d right = q.RotateVector(-math::Vector3d::UnitY);
  const math::Vector3d down = q.RotateVector(-math::Vector3d::UnitZ);
  const math::Vector3d forward = q.RotateVector(math::Vector3d::UnitX);

  // Its link's velocity, from gz-sim: no differencing, so no first step and
  // no teleport to special-case
  const math::Vector3d velocity =
      Link(this->link).WorldLinearVelocity(_ecm).value_or(math::Vector3d::Zero);

  math::Vector3d wind = math::Vector3d::Zero;
  if (this->useWind && this->sampler.Sync(_ecm))
    wind = this->sampler.At(pose.Pos(), _info.simTime);

  // The world's gravity, in g: none pins the drops
  math::Vector3d gravity(0.0, 0.0, -kStandardGravity);
  if (const auto *g = _ecm.Component<components::Gravity>(worldEntity(_ecm)))
    gravity = g->Data();
  gravity /= kStandardGravity;
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
  bool changed = false;
  bool clear = false;
  std::map<std::string, double> add;
  for (const auto &[key, value] : _msg.params())
  {
    double number{0.0};
    const bool known = key == "enabled" || key == "rain_rate" ||
        key == "exposure" || key == "focus" || key == "dry_time" ||
        key == "clear" || key == "contact_angle" || key == "slide_radius" ||
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
    if (key == "enabled")
      this->enabled = number != 0.0;
    else if (key == "clear")
      clear = number != 0.0;
    else if (key == "add_x" || key == "add_y" || key == "add_radius")
      add[key] = number;
    else
    {
      changed = true;
      if (key == "rain_rate")
        params.rainRate = number;
      else if (key == "exposure")
        params.exposure = number;
      else if (key == "focus")
        params.focus = number;
      else if (key == "dry_time")
        params.dryTime = number;
      else if (key == "slide_radius")
        params.slideRadius = number;
      else
      {
        params.contactAngle = number;
        if (number > lens_drops::kMaxContactAngle)
        {
          gzwarn << "[LensDrops] contact_angle " << number << " is past "
                 << "pi/2, which a drop map cannot hold; using pi/2\n";
        }
      }
    }
  }
  if (changed)
    this->drops.SetParams(params);
  if (clear)
    this->drops.Clear();
  if (!add.empty())
  {
    const bool partial = add.size() != 3;
    if (partial ||
        !this->drops.Add(add["add_x"], add["add_y"], add["add_radius"]))
    {
      // The 1st, 10th, 100th... refusal warns, with the count so far: a
      // stream of bad drops does not flood the log
      ++this->refused;
      if (this->refused >= this->nextRefusalWarning)
      {
        this->nextRefusalWarning *= 10;
        if (partial)
        {
          gzwarn << "[LensDrops] no drop added: add_x, add_y and add_radius "
                 << "go together; " << this->refused << " refused so far\n";
        }
        else
        {
          gzwarn << "[LensDrops] no drop added: add_x and add_y go in "
                 << "[0, 1], add_radius in (0, "
                 << lens_drops::kMaxMergedRadius << "] image heights, and "
                 << "the lens holds " << lens_drops::kMaxDrops
                 << " drops at most; " << this->refused
                 << " refused so far\n";
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void LensDropsPrivate::OnSceneUpdate()
{
  if (!this->valid || this->failed)
    return;
  {
    const std::lock_guard<std::mutex> lock(this->passMutex);
    if (this->pass)
      return;
  }

  rendering::ScenePtr scene = rendering::sceneFromFirstRenderEngine();
  if (!scene || !scene->IsInitialized())
    return;
  auto camera = std::dynamic_pointer_cast<rendering::Camera>(
      scene->SensorByName(this->cameraName));
  if (!camera)
  {
    // The sensors system makes the camera a few frames in; keep looking.
    if (++this->tries == 300)
    {
      gzwarn << "[LensDrops] camera [" << this->cameraName
             << "] not found in the scene yet\n";
    }
    return;
  }
  if (scene->Engine()->Name() != "ogre2" ||
      scene->Engine()->GraphicsAPI() != rendering::GraphicsAPI::OPENGL)
  {
    gzerr << "[LensDrops] needs the ogre2 render engine on OpenGL (its "
          << "shaders are GLSL); no drops are drawn\n";
    this->failed = true;
    return;
  }
  const BridgeApi &api = LoadBridge();
  if (!api.loaded)
  {
    this->failed = true;
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
    return;
  }
  this->map = std::make_unique<lens_drops::DropMap>(this->mapWidth,
                                                    this->mapHeight);
  lens_drops_pass_t made = api.create(vs.c_str(), fs.c_str(), this->mapWidth,
      this->mapHeight, &LensDropsPrivate::Pull, &LensDropsPrivate::Wanted,
      this);
  if (!made || !api.attach(made, camera.get()))
  {
    api.destroy(made);
    gzerr << "[LensDrops] could not add the render pass to ["
          << this->cameraName << "]\n";
    this->failed = true;
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(this->passMutex);
    this->pass = made;
  }
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->attached = true;
  }
  gzmsg << "[LensDrops] render pass on [" << this->cameraName << "], map "
        << this->mapWidth << "x" << this->mapHeight << "\n";
}

//////////////////////////////////////////////////
int LensDropsPrivate::Wanted(void *_context)
{
  auto *self = static_cast<LensDropsPrivate *>(_context);
  const std::lock_guard<std::mutex> lock(self->mutex);
  const bool wet =
      !self->drops.List().empty() || !self->drops.Trails().empty();
  self->drawing = self->enabled && wet;
  return self->drawing ? 1 : 0;
}

//////////////////////////////////////////////////
const float *LensDropsPrivate::Pull(void *_context, double _stamp,
    float *_focus, int *_enabled)
{
  auto *self = static_cast<LensDropsPrivate *>(_context);
  // The drops are copied under the lock; the map is drawn outside it
  std::optional<lens_drops::Drops> copy;
  bool wet{false};
  {
    const std::lock_guard<std::mutex> lock(self->mutex);
    *_focus = static_cast<float>(self->drops.Parameters().focus);
    wet = !self->drops.List().empty() || !self->drops.Trails().empty();
    *_enabled = self->enabled && wet ? 1 : 0;
    if (!self->drewOnce || self->drops.Generation() != self->drawn)
    {
      copy.emplace(self->drops);
      self->drawn = self->drops.Generation();
    }
  }
  const float *drawnMap{nullptr};
  if (copy)
  {
    self->drewOnce = true;
    self->map->Render(*copy);
    drawnMap = self->map->Data().data();
    const double texels =
        static_cast<double>(self->map->Width()) * self->map->Height();
    const std::lock_guard<std::mutex> lock(self->mutex);
    self->covered = static_cast<double>(self->map->Covered()) / texels;
    ++self->draws;
  }

  // The mask of this frame, stamped as it, while someone listens and the
  // drops are drawn
  if (self->publishMask && *_enabled && self->maskPub.HasConnections())
  {
    self->map->Mask(self->mask);
    msgs::Image image;
    *image.mutable_header()->mutable_stamp() =
        convert<msgs::Time>(std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(_stamp)));
    image.set_width(static_cast<unsigned int>(self->map->Width()));
    image.set_height(static_cast<unsigned int>(self->map->Height()));
    image.set_step(static_cast<unsigned int>(self->map->Width()));
    image.set_pixel_format_type(msgs::PixelFormatType::L_INT8);
    image.set_data(self->mask.data(), self->mask.size());
    self->maskPub.Publish(image);
  }
  return drawnMap;
}

//////////////////////////////////////////////////
void LensDropsPrivate::OnTeardown()
{
  this->Release();
  this->failed = true;
  const std::lock_guard<std::mutex> lock(this->mutex);
  this->attached = false;
  this->drawing = false;
}
}  // namespace gz::sim::maritime

GZ_ADD_PLUGIN(gz::sim::maritime::LensDrops,
              gz::sim::System,
              gz::sim::maritime::LensDrops::ISystemConfigure,
              gz::sim::maritime::LensDrops::ISystemPostUpdate,
              gz::sim::maritime::LensDrops::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::maritime::LensDrops,
                    "gz::sim::maritime::LensDrops")
