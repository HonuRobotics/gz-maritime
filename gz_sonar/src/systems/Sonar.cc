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
#include "Sonar.hh"

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/SegmentationCamera.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/rendering/Events.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>

#include "SonarSdf.hh"
#include "SonarSensor.hh"
#include "gz/maritime/sonar/Water.hh"

namespace gz::sim::maritime
{
namespace sonar = gz::maritime::sonar;
using std::chrono::nanoseconds;

namespace
{
/// \brief The user datum gz_waves_rendering's WaterVisual puts on the
/// water surface and its tiles, so the sonar can hide them.
constexpr const char *kWaterDatum = "gz-maritime-water";

/// \brief Label ids are 8 bits; this one stands for "any other surface"
/// once a world has used up the rest.
constexpr int kOtherLabel = 255;

/// \brief The outermost model an entity belongs to.
Entity TopModel(const EntityComponentManager &_ecm, Entity _e)
{
  const Entity world = worldEntity(_ecm);
  Entity top = kNullEntity;
  while (_e != kNullEntity && _e != world)
  {
    if (_ecm.Component<components::Model>(_e))
      top = _e;
    const auto *p = _ecm.Component<components::ParentEntity>(_e);
    _e = p ? p->Data() : kNullEntity;
  }
  return top;
}

/// \brief Models from the outermost down to the one that holds a link.
std::vector<Entity> ModelChain(const EntityComponentManager &_ecm, Entity _e)
{
  std::vector<Entity> chain;
  const Entity world = worldEntity(_ecm);
  while (_e != kNullEntity && _e != world)
  {
    if (_ecm.Component<components::Model>(_e))
      chain.insert(chain.begin(), _e);
    const auto *p = _ecm.Component<components::ParentEntity>(_e);
    _e = p ? p->Data() : kNullEntity;
  }
  return chain;
}

/// \brief Every visual in a subtree of the scene.
void CollectVisuals(const rendering::VisualPtr &_v,
  std::vector<rendering::VisualPtr> &_out)
{
  if (!_v)
    return;
  _out.push_back(_v);
  for (unsigned int i = 0; i < _v->ChildCount(); ++i)
  {
    CollectVisuals(
      std::dynamic_pointer_cast<rendering::Visual>(_v->ChildByIndex(i)),
      _out);
  }
}

/// \brief A visual's resolved marks, handed to the rendering thread.
struct VisualMark
{
  Entity visual{kNullEntity};
  Entity topModel{kNullEntity};
  int label{0};
  bool transparent{false};
};

/// \brief A sonar to give cameras to.
struct RigRequest
{
  Entity sensor{kNullEntity};
  Entity link{kNullEntity};
  Entity topModel{kNullEntity};
  math::Pose3d pose;
  std::shared_ptr<SonarSensor> sonar;
};

/// \brief The cameras of one sonar, on the rendering thread.
struct Rig
{
  RigRequest request;
  std::vector<rendering::DepthCameraPtr> depth;
  std::vector<rendering::SegmentationCameraPtr> seg;
  std::vector<common::ConnectionPtr> connections;
  std::vector<std::vector<float>> depthBuffers;
  std::vector<std::vector<std::uint8_t>> segBuffers;
  unsigned int channels{3};
  /// \brief The next ping and the time it is due.
  std::uint64_t next{0};
  nanoseconds lastStamp{-1};
};
}  // namespace

class SonarPrivate
{
  /// \brief Find sonars: all of them (at the first update, after a reset)
  /// or the ones noted since.
  public: void FindSonars(EntityComponentManager &_ecm, bool _all);

  /// \brief Resolve the marks of new visuals.
  public: void FindVisuals(const EntityComponentManager &_ecm, bool _all);

  /// \brief Rendering thread callbacks.
  public: void OnSceneUpdate();
  public: void OnPreRender();
  public: void OnRender();
  public: void OnTeardown();

  /// \brief Build the cameras of one sonar under its link visual.
  public: bool BuildRig(const RigRequest &_request, Rig &_rig);

  /// \brief Map of entity to rendering visual, rebuilt when the scene
  /// changes size.
  public: rendering::VisualPtr VisualOf(Entity _e);
  public: void RefreshVisuals();

  // ---- configuration
  public: sonar::Water water;
  public: std::uint64_t seed{0};
  public: std::set<std::string> seabedModels;
  public: AcousticMarks seabedMarks;
  public: AcousticMarks defaultMarks;
  public: sonar::AcousticProperties defaults;

  // ---- simulation thread
  public: EventManager *events{nullptr};
  public: bool rescan{true};
  public: std::vector<Entity> newSensors;
  public: std::vector<Entity> removedSensors;
  public: std::vector<Entity> newVisuals;
  public: std::map<Entity, std::shared_ptr<SonarSensor>> sonars;
  public: std::map<std::tuple<Entity, double, double, double, bool>, int>
    labelOf;
  public: bool warnedLabels{false};

  // ---- shared, under mutex
  public: std::mutex mutex;
  public: std::vector<RigRequest> rigRequests;
  public: std::vector<Entity> rigRemovals;
  public: std::vector<VisualMark> markRequests;
  public: std::shared_ptr<const LabelTable> table;
  public: bool clearRigs{false};
  public: std::atomic<bool> haveSonars{false};
  public: std::atomic<bool> sceneSeen{false};

  // ---- rendering thread
  public: common::ConnectionPtr sceneUpdateConn;
  public: common::ConnectionPtr teardownConn;
  public: common::ConnectionPtr preRenderConn;
  public: common::ConnectionPtr renderConn;
  public: rendering::ScenePtr scene;
  public: std::map<Entity, Rig> rigs;
  public: std::vector<RigRequest> waitingRigs;
  public: std::vector<VisualMark> waitingMarks;
  public: std::unordered_map<Entity, VisualMark> marks;
  public: std::unordered_map<Entity, rendering::VisualPtr> visualByEntity;
  public: std::vector<rendering::VisualPtr> waterVisuals;
  public: std::size_t knownVisualCount{0};
};

//////////////////////////////////////////////////
Sonar::Sonar() : dataPtr(std::make_unique<SonarPrivate>())
{
}

//////////////////////////////////////////////////
Sonar::~Sonar() = default;

//////////////////////////////////////////////////
void Sonar::Configure(const Entity &,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &, EventManager &_eventMgr)
{
  auto &d = *this->dataPtr;
  d.events = &_eventMgr;
  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);
  d.water.temperature = sdf->Get<double>("temperature", 10.0).first;
  d.water.salinity = sdf->Get<double>("salinity", 35.0).first;
  d.water.soundSpeed = sdf->Get<double>("sound_speed", 1500.0).first;
  d.seed = static_cast<std::uint64_t>(sdf->Get<int>("seed", 0).first);

  if (sdf->HasElement("seabed"))
  {
    const auto seabed = sdf->FindElement("seabed");
    d.seabedMarks = ReadAcousticValues(seabed);
    for (auto m = seabed->FindElement("model"); m;
         m = m->GetNextElement("model"))
    {
      d.seabedModels.insert(m->Get<std::string>());
    }
  }
  if (sdf->HasElement("default"))
    d.defaultMarks = ReadAcousticValues(sdf->FindElement("default"));
  ApplyMarks(d.defaults, d.defaultMarks);

  auto table = std::make_shared<LabelTable>();
  table->properties.push_back(d.defaults);
  table->names.emplace_back();
  d.table = table;

  // Only the scene update and teardown now. The render events make the
  // rendering thread run every step, so they are joined from the scene
  // update, on the rendering thread, once a sonar exists.
  d.sceneUpdateConn = _eventMgr.Connect<events::SceneUpdate>(
    [&d]() { d.OnSceneUpdate(); });
  d.teardownConn = _eventMgr.Connect<events::RenderTeardown>(
    [&d]() { d.OnTeardown(); });
}

//////////////////////////////////////////////////
void Sonar::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  const bool all = d.rescan;
  d.rescan = false;
  d.FindSonars(_ecm, all);
  d.FindVisuals(_ecm, all);

  // With no stock rendering sensor in the world, the Sensors system has no
  // reason to start the render engine until asked.
  if (d.haveSonars && !d.sceneSeen)
    d.events->Emit<events::ForceRender>();
}

//////////////////////////////////////////////////
void Sonar::PostUpdate(const UpdateInfo &, const EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  // EachNew only sees an entity in the step it was made in; note it here,
  // after every PreUpdate, and resolve it at the next.
  _ecm.EachNew<components::CustomSensor>(
    [&](const Entity &_e, const components::CustomSensor *) -> bool
    {
      d.newSensors.push_back(_e);
      return true;
    });
  _ecm.EachNew<components::Visual>(
    [&](const Entity &_e, const components::Visual *) -> bool
    {
      d.newVisuals.push_back(_e);
      return true;
    });
  _ecm.EachRemoved<components::CustomSensor>(
    [&](const Entity &_e, const components::CustomSensor *) -> bool
    {
      d.removedSensors.push_back(_e);
      return true;
    });
}

//////////////////////////////////////////////////
void Sonar::Reset(const UpdateInfo &, EntityComponentManager &)
{
  auto &d = *this->dataPtr;
  d.sonars.clear();
  d.newSensors.clear();
  d.newVisuals.clear();
  d.removedSensors.clear();
  d.rescan = true;
  std::lock_guard<std::mutex> lock(d.mutex);
  d.rigRequests.clear();
  d.markRequests.clear();
  d.clearRigs = true;
}

//////////////////////////////////////////////////
void SonarPrivate::FindSonars(EntityComponentManager &_ecm, bool _all)
{
  std::vector<Entity> candidates;
  candidates.swap(this->newSensors);
  if (_all)
  {
    candidates.clear();
    _ecm.Each<components::CustomSensor>(
      [&](const Entity &_e, const components::CustomSensor *) -> bool
      {
        candidates.push_back(_e);
        return true;
      });
  }

  for (const Entity e : this->removedSensors)
  {
    if (this->sonars.erase(e))
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->rigRemovals.push_back(e);
    }
  }
  this->removedSensors.clear();

  for (const Entity e : candidates)
  {
    if (this->sonars.count(e))
      continue;
    const auto *custom = _ecm.Component<components::CustomSensor>(e);
    const auto *parent = _ecm.Component<components::ParentEntity>(e);
    const auto *pose = _ecm.Component<components::Pose>(e);
    if (!custom || !parent || !pose)
      continue;
    const std::string type = sensors::customType(custom->Data());
    if (!IsSonarType(type))
      continue;

    sdf::Sensor data = custom->Data();
    data.SetName(scopedName(e, _ecm, "::", false));
    if (data.Topic().empty())
      data.SetTopic("/" + scopedName(e, _ecm, "/", true) + "/image");
    auto sensor = std::make_shared<SonarSensor>();
    if (!sensor->Load(data, type, this->water, this->seed))
    {
      gzerr << "Sonar: cannot create [" << data.Name() << "]\n";
      continue;
    }
    this->sonars[e] = sensor;

    RigRequest r;
    r.sensor = e;
    r.link = parent->Data();
    r.topModel = TopModel(_ecm, e);
    r.pose = pose->Data();
    r.sonar = sensor;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->rigRequests.push_back(r);
  }
  this->haveSonars = !this->sonars.empty();
}

//////////////////////////////////////////////////
void SonarPrivate::FindVisuals(const EntityComponentManager &_ecm, bool _all)
{
  std::vector<Entity> candidates;
  candidates.swap(this->newVisuals);
  if (_all)
  {
    candidates.clear();
    this->labelOf.clear();
    _ecm.Each<components::Visual>(
      [&](const Entity &_e, const components::Visual *) -> bool
      {
        candidates.push_back(_e);
        return true;
      });
  }
  if (candidates.empty())
    return;

  auto table = std::make_shared<LabelTable>(*this->table);
  if (_all)
  {
    table->properties.resize(1);
    table->names.resize(1);
  }
  std::vector<VisualMark> found;
  for (const Entity v : candidates)
  {
    const auto *name = _ecm.Component<components::Name>(v);
    const auto *parent = _ecm.Component<components::ParentEntity>(v);
    // Lights have visuals too; only a link's visuals are surfaces.
    if (!name || !parent || !_ecm.Component<components::Link>(parent->Data()))
      continue;
    const Entity link = parent->Data();
    const auto *linkName = _ecm.Component<components::Name>(link);
    const auto chain = ModelChain(_ecm, link);
    if (!linkName || chain.empty())
      continue;

    // Precedence, lowest first: built in, the world default, the seabed
    // entry, then each model from the outermost in, then the visual.
    sonar::AcousticProperties props = this->defaults;
    const auto *topName = _ecm.Component<components::Name>(chain.front());
    if (topName && this->seabedModels.count(topName->Data()))
      ApplyMarks(props, this->seabedMarks);
    bool transparent = false;
    for (const Entity m : chain)
    {
      if (const auto *sdfModel = _ecm.Component<components::ModelSdf>(m))
      {
        const auto marks = ReadAcousticMarks(sdfModel->Data().Element());
        ApplyMarks(props, marks);
        transparent = transparent || marks.transparent;
      }
    }
    if (const auto *sdfModel = _ecm.Component<components::ModelSdf>(chain.back()))
    {
      const sdf::Link *l = sdfModel->Data().LinkByName(linkName->Data());
      const sdf::Visual *vs = l ? l->VisualByName(name->Data()) : nullptr;
      if (vs)
      {
        const auto marks = ReadAcousticMarks(vs->Element());
        ApplyMarks(props, marks);
        transparent = transparent || marks.transparent;
      }
    }

    // One label per model and property set.
    const auto key = std::make_tuple(chain.front(), props.lambertDb,
      props.specularDb.value_or(-1000.0), props.specularWidth,
      props.specularDb.has_value());
    int label;
    const auto it = this->labelOf.find(key);
    if (it != this->labelOf.end())
    {
      label = it->second;
    }
    else if (table->properties.size() < static_cast<std::size_t>(kOtherLabel))
    {
      label = static_cast<int>(table->properties.size());
      table->properties.push_back(props);
      table->names.push_back(scopedName(chain.front(), _ecm, "::", false));
      this->labelOf[key] = label;
    }
    else
    {
      // Past 254 surfaces, the rest share one id with the default
      // properties; labeling per ping by range is the way past it.
      if (!this->warnedLabels)
      {
        gzwarn << "Sonar: more than 254 acoustic surfaces; the rest share "
               << "the default properties and one ground-truth label\n";
        this->warnedLabels = true;
      }
      if (table->properties.size() == static_cast<std::size_t>(kOtherLabel))
      {
        table->properties.push_back(this->defaults);
        table->names.emplace_back("other");
      }
      label = kOtherLabel;
    }
    found.push_back({v, chain.front(), label, transparent});
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->table = table;
  this->markRequests.insert(this->markRequests.end(), found.begin(),
    found.end());
}

//////////////////////////////////////////////////
void SonarPrivate::OnSceneUpdate()
{
  if (this->renderConn || !this->haveSonars)
    return;
  // Joined here, on the rendering thread, so the emits that run on this
  // thread never race the connection.
  this->preRenderConn = this->events->Connect<events::PreRender>(
    [this]() { this->OnPreRender(); });
  this->renderConn = this->events->Connect<events::Render>(
    [this]() { this->OnRender(); });
}

//////////////////////////////////////////////////
void SonarPrivate::RefreshVisuals()
{
  std::vector<rendering::VisualPtr> all;
  CollectVisuals(this->scene->RootVisual(), all);
  if (all.size() == this->knownVisualCount)
    return;
  this->knownVisualCount = all.size();
  this->visualByEntity.clear();
  this->waterVisuals.clear();
  for (const auto &v : all)
  {
    if (v->HasUserData(kWaterDatum))
      this->waterVisuals.push_back(v);
    if (!v->HasUserData("gazebo-entity"))
      continue;
    const auto datum = v->UserData("gazebo-entity");
    if (const auto *id = std::get_if<std::uint64_t>(&datum))
      this->visualByEntity[static_cast<Entity>(*id)] = v;
  }
}

//////////////////////////////////////////////////
rendering::VisualPtr SonarPrivate::VisualOf(Entity _e)
{
  const auto it = this->visualByEntity.find(_e);
  return it == this->visualByEntity.end() ? nullptr : it->second;
}

//////////////////////////////////////////////////
bool SonarPrivate::BuildRig(const RigRequest &_request, Rig &_rig)
{
  const auto linkVisual = this->VisualOf(_request.link);
  if (!linkVisual)
    return false;
  _rig.request = _request;
  const auto &layout = _request.sonar->Layout();
  const std::string base = _request.sonar->Name();
  const std::size_t n = layout.yaws.size();
  _rig.depthBuffers.resize(n);
  _rig.segBuffers.resize(n);
  for (std::size_t c = 0; c < n; ++c)
  {
    const math::Pose3d local = _request.pose *
      math::Pose3d(0, 0, 0, 0, 0, layout.yaws[c]);
    const std::string tag = base + "_sonar_" + std::to_string(c);

    auto depth = this->scene->CreateDepthCamera(tag + "_depth");
    depth->SetImageWidth(layout.width);
    depth->SetImageHeight(layout.height);
    depth->SetImageFormat(rendering::PF_FLOAT32_R);
    depth->SetHFOV(math::Angle(layout.hfov));
    depth->SetAspectRatio(static_cast<double>(layout.width) / layout.height);
    depth->SetNearClipPlane(layout.nearClip);
    depth->SetFarClipPlane(layout.farClip);
    depth->SetAntiAliasing(0);
    depth->CreateDepthTexture();
    depth->SetLocalPose(local);
    linkVisual->AddChild(depth);
    auto *buf = &_rig.depthBuffers[c];
    _rig.connections.push_back(depth->ConnectNewDepthFrame(
      [buf](const float *_d, unsigned int _w, unsigned int _h, unsigned int,
            const std::string &)
      {
        buf->assign(_d, _d + static_cast<std::size_t>(_w) * _h);
      }));
    _rig.depth.push_back(depth);

    auto seg = this->scene->CreateSegmentationCamera(tag + "_seg");
    seg->SetImageWidth(layout.width);
    seg->SetImageHeight(layout.height);
    seg->SetHFOV(math::Angle(layout.hfov));
    seg->SetAspectRatio(static_cast<double>(layout.width) / layout.height);
    seg->SetNearClipPlane(layout.nearClip);
    seg->SetFarClipPlane(layout.farClip);
    seg->SetSegmentationType(rendering::SegmentationType::ST_SEMANTIC);
    seg->EnableColoredMap(false);
    seg->SetBackgroundLabel(0);
    seg->CreateSegmentationTexture();
    seg->SetLocalPose(local);
    linkVisual->AddChild(seg);
    auto *sbuf = &_rig.segBuffers[c];
    auto *channels = &_rig.channels;
    _rig.connections.push_back(seg->ConnectNewSegmentationFrame(
      [sbuf, channels](const std::uint8_t *_d, unsigned int _w,
                       unsigned int _h, unsigned int _c, const std::string &)
      {
        *channels = _c;
        sbuf->assign(_d, _d + static_cast<std::size_t>(_w) * _h * _c);
      }));
    _rig.seg.push_back(seg);
  }
  return true;
}

//////////////////////////////////////////////////
void SonarPrivate::OnPreRender()
{
  if (!this->scene)
    this->scene = rendering::sceneFromFirstRenderEngine();
  if (!this->scene || !this->scene->IsInitialized())
    return;
  this->sceneSeen = true;

  bool clear = false;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->waitingRigs.insert(this->waitingRigs.end(),
      this->rigRequests.begin(), this->rigRequests.end());
    this->rigRequests.clear();
    this->waitingMarks.insert(this->waitingMarks.end(),
      this->markRequests.begin(), this->markRequests.end());
    this->markRequests.clear();
    for (const Entity e : this->rigRemovals)
      this->rigs.erase(e);
    this->rigRemovals.clear();
    std::swap(clear, this->clearRigs);
  }
  if (clear)
  {
    this->rigs.clear();
    this->marks.clear();
  }

  this->RefreshVisuals();

  // Visuals and sonars whose rendering visual is not in the scene yet stay
  // queued until it is.
  for (auto it = this->waitingMarks.begin(); it != this->waitingMarks.end();)
  {
    if (this->VisualOf(it->visual))
    {
      this->marks[it->visual] = *it;
      it = this->waitingMarks.erase(it);
    }
    else
      ++it;
  }
  // Built in place, so the camera callbacks can keep pointers to the rig's
  // buffers: map nodes do not move.
  for (auto it = this->waitingRigs.begin(); it != this->waitingRigs.end();)
  {
    const Entity e = it->sensor;
    this->rigs.erase(e);
    Rig &rig = this->rigs[e];
    if (this->BuildRig(*it, rig))
    {
      it = this->waitingRigs.erase(it);
    }
    else
    {
      this->rigs.erase(e);
      ++it;
    }
  }
}

//////////////////////////////////////////////////
void SonarPrivate::OnRender()
{
  if (!this->scene || !this->scene->IsInitialized() || this->rigs.empty())
    return;
  const auto t = std::chrono::duration_cast<nanoseconds>(this->scene->Time());

  std::shared_ptr<const LabelTable> table;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    table = this->table;
  }

  for (auto &[entity, rig] : this->rigs)
  {
    const double rate = rig.request.sonar->Rate();
    const auto due = nanoseconds(static_cast<std::int64_t>(
      std::llround(static_cast<double>(rig.next) * 1e9 / rate)));
    if (t < due)
      continue;
    const std::uint64_t ping = rig.next;

    // Labels on, and hide what the sonar must not see: the water, the
    // transparent surfaces and its own vehicle.
    std::vector<std::pair<rendering::VisualPtr, rendering::Variant>> saved;
    std::vector<rendering::VisualPtr> hidden;
    auto label = [&](const rendering::VisualPtr &_v, int _l) {
      saved.emplace_back(_v, _v->UserData("label"));
      _v->SetUserData("label", _l);
    };
    for (const auto &[e, mark] : this->marks)
    {
      const auto v = this->VisualOf(e);
      if (!v)
        continue;
      if (mark.transparent || mark.topModel == rig.request.topModel)
      {
        v->SetVisible(false);
        hidden.push_back(v);
        continue;
      }
      label(v, mark.label);
      for (unsigned int i = 0; i < v->ChildCount(); ++i)
      {
        if (auto child = std::dynamic_pointer_cast<rendering::Visual>(
              v->ChildByIndex(i)))
        {
          label(child, mark.label);
        }
      }
    }
    for (const auto &w : this->waterVisuals)
    {
      w->SetVisible(false);
      hidden.push_back(w);
    }

    for (std::size_t c = 0; c < rig.depth.size(); ++c)
    {
      rig.depth[c]->Render();
      rig.depth[c]->PostRender();
      rig.seg[c]->Render();
      rig.seg[c]->PostRender();
    }

    for (auto &v : hidden)
      v->SetVisible(true);
    // Put back whatever was there; an empty datum reads as background, as
    // it did before.
    for (auto it = saved.rbegin(); it != saved.rend(); ++it)
      it->first->SetUserData("label", it->second);

    PingFrame frame;
    frame.ping = ping;
    frame.stamp = t;
    frame.dt = rig.lastStamp.count() < 0 ? 1.0 / rate :
      std::chrono::duration<double>(t - rig.lastStamp).count();
    frame.depth = rig.depthBuffers;
    frame.labels = rig.segBuffers;
    frame.channels = rig.channels;
    frame.table = table;
    rig.request.sonar->Push(std::move(frame));
    rig.lastStamp = t;

    // The next ping: the first one due after this state.
    rig.next = static_cast<std::uint64_t>(std::floor(
      std::chrono::duration<double>(t).count() * rate)) + 1;
  }
}

//////////////////////////////////////////////////
void SonarPrivate::OnTeardown()
{
  this->rigs.clear();
  this->marks.clear();
  this->visualByEntity.clear();
  this->waterVisuals.clear();
  this->knownVisualCount = 0;
  this->scene.reset();
}
}  // namespace gz::sim::maritime

GZ_ADD_PLUGIN(gz::sim::maritime::Sonar,
              gz::sim::System,
              gz::sim::maritime::Sonar::ISystemConfigure,
              gz::sim::maritime::Sonar::ISystemPreUpdate,
              gz::sim::maritime::Sonar::ISystemPostUpdate,
              gz::sim::maritime::Sonar::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(gz::sim::maritime::Sonar, "gz::sim::maritime::Sonar")
