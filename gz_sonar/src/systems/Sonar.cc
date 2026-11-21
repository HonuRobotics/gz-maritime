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

#include <algorithm>
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
#include <gz/sim/components/World.hh>
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
  /// \brief The visual is gone: forget its mark.
  bool removed{false};
  /// \brief Sim time of the update that made it; see Stamped.
  std::chrono::nanoseconds stamp{0};
};

/// \brief A sonar to give cameras to.
struct RigRequest
{
  Entity sensor{kNullEntity};
  Entity link{kNullEntity};
  Entity topModel{kNullEntity};
  math::Pose3d pose;
  std::shared_ptr<SonarSensor> sonar;
  /// \brief Sim time of the update that made it; see Stamped.
  std::chrono::nanoseconds stamp{0};
};

/// \brief A label table, or a sonar to drop, with the sim time of the
/// update that made it.
///
/// Every change the simulation thread hands the rendering thread is
/// stamped, and the rendering thread takes it up at the first frame whose
/// time has reached the stamp. The Sensors system renders frame S while the
/// simulation thread runs PreUpdate(S+1), so without the stamp a change made
/// there would land in frame S or S+1 depending on thread timing, and two
/// runs with the same seed would differ.
struct StampedTable
{
  std::shared_ptr<const LabelTable> table;
  std::chrono::nanoseconds stamp{0};
};
struct StampedRemoval
{
  Entity sensor{kNullEntity};
  std::chrono::nanoseconds stamp{0};
};

/// \brief The first ping on a sonar's rate grid at or after _t.
std::uint64_t FirstPingAt(nanoseconds _t, double _rate)
{
  const double k = std::chrono::duration<double>(_t).count() * _rate;
  return static_cast<std::uint64_t>(std::ceil(k - 1e-6));
}

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

  /// \brief Disconnect a rig and destroy its cameras, so a sonar that is
  /// removed, reset or spawned again leaves nothing behind in the scene.
  public: void DestroyRig(Rig &_rig);

  /// \brief Destroy one rig's cameras and drop it.
  public: void EraseRig(Entity _sensor);

  /// \brief Destroy every rig's cameras and drop them all.
  public: void ClearRigs();

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
  public: std::chrono::nanoseconds now{0};
  public: std::vector<Entity> removedVisuals;
  /// \brief Labels: one per model and property set, held while any of its
  /// visuals exists and then free for the next one, so a world that spawns
  /// and removes targets does not run out of the 254.
  public: using LabelKey = std::tuple<Entity, double, double, double, bool>;
  public: std::map<LabelKey, int> labelOf;
  public: std::map<LabelKey, int> labelUsers;
  public: std::map<Entity, LabelKey> keyOfVisual;
  public: std::set<int> freeLabels;
  public: bool warnedLabels{false};
  /// \brief The system is not attached to the world and does nothing.
  public: bool disabled{false};

  // ---- shared, under mutex
  public: std::mutex mutex;
  public: std::vector<RigRequest> rigRequests;
  public: std::vector<StampedRemoval> rigRemovals;
  public: std::vector<VisualMark> markRequests;
  public: std::vector<StampedTable> tableRequests;
  /// \brief The latest table, on the simulation thread.
  public: std::shared_ptr<const LabelTable> table;
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
  public: std::vector<StampedRemoval> waitingRemovals;
  public: std::vector<StampedTable> waitingTables;
  /// \brief The table the frames use, and the last frame's time: time going
  /// back means the world was reset.
  public: std::shared_ptr<const LabelTable> renderTable;
  public: nanoseconds lastFrame{-1};
  public: std::unordered_map<Entity, VisualMark> marks;
  public: std::unordered_map<Entity, rendering::VisualPtr> visualByEntity;
  public: std::vector<rendering::VisualPtr> waterVisuals;
  public: std::vector<unsigned int> knownVisualIds;

  /// \brief Cameras built so far, to give each a name never used before:
  /// gz-rendering refuses a name that is taken.
  public: std::uint64_t camerasBuilt{0};
};

//////////////////////////////////////////////////
Sonar::Sonar() : dataPtr(std::make_unique<SonarPrivate>())
{
}

//////////////////////////////////////////////////
Sonar::~Sonar()
{
  // Leave the events before anything their callbacks use goes away.
  this->dataPtr->renderConn.reset();
  this->dataPtr->preRenderConn.reset();
  this->dataPtr->sceneUpdateConn.reset();
  this->dataPtr->teardownConn.reset();
}

//////////////////////////////////////////////////
void Sonar::Configure(const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  auto &d = *this->dataPtr;
  // One per world: it drives every sonar there, so a second copy on a model
  // would render each of them twice.
  if (!_ecm.Component<components::World>(_entity))
  {
    gzerr << "Sonar: load gz-maritime-sonar-system in the <world>, not in a "
          << "model; this copy does nothing\n";
    d.disabled = true;
    return;
  }
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
  d.renderTable = table;

  // Only the scene update and teardown now. The render events make the
  // rendering thread run every step, so they are joined from the scene
  // update, on the rendering thread, once a sonar exists.
  d.sceneUpdateConn = _eventMgr.Connect<events::SceneUpdate>(
    [&d]() { d.OnSceneUpdate(); });
  d.teardownConn = _eventMgr.Connect<events::RenderTeardown>(
    [&d]() { d.OnTeardown(); });
}

//////////////////////////////////////////////////
void Sonar::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (d.disabled)
    return;
  d.now = std::chrono::duration_cast<nanoseconds>(_info.simTime);
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
  if (d.disabled)
    return;
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
  _ecm.EachRemoved<components::Visual>(
    [&](const Entity &_e, const components::Visual *) -> bool
    {
      d.removedVisuals.push_back(_e);
      return true;
    });
}

//////////////////////////////////////////////////
void Sonar::Reset(const UpdateInfo &, EntityComponentManager &)
{
  // Keep every sonar that still exists, with its cameras: building them
  // again is slow and gz-rendering does not reliably survive destroying and
  // recreating depth cameras in one frame. The rescan drops the sonars whose
  // sensors are gone and adds any new ones; each kept sonar starts over on
  // the rate grid again, which the rendering thread does when it sees sim
  // time go back, so a run after a reset repeats the run from the start.
  auto &d = *this->dataPtr;
  if (d.disabled)
    return;
  d.newSensors.clear();
  d.newVisuals.clear();
  d.removedSensors.clear();
  d.removedVisuals.clear();
  d.rescan = true;
  for (auto &[entity, sonar] : d.sonars)
    sonar->ResetState();
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

  if (_all)
  {
    // A rescan (after a reset) also drops the sonars that are gone.
    const std::set<Entity> present(candidates.begin(), candidates.end());
    for (const auto &[e, sonar] : this->sonars)
    {
      if (!present.count(e))
        this->removedSensors.push_back(e);
    }
  }
  for (const Entity e : this->removedSensors)
  {
    if (this->sonars.erase(e))
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->rigRemovals.push_back({e, this->now});
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
    r.stamp = this->now;
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
  std::vector<Entity> removed;
  removed.swap(this->removedVisuals);
  if (_all)
  {
    candidates.clear();
    removed.clear();
    this->labelOf.clear();
    this->labelUsers.clear();
    this->keyOfVisual.clear();
    this->freeLabels.clear();
    _ecm.Each<components::Visual>(
      [&](const Entity &_e, const components::Visual *) -> bool
      {
        candidates.push_back(_e);
        return true;
      });
  }
  if (candidates.empty() && removed.empty())
    return;

  auto table = std::make_shared<LabelTable>(*this->table);
  if (_all)
  {
    table->properties.resize(1);
    table->names.resize(1);
  }
  std::vector<VisualMark> found;

  // A label whose last visual is gone is free again.
  for (const Entity v : removed)
  {
    const auto k = this->keyOfVisual.find(v);
    if (k == this->keyOfVisual.end())
      continue;
    const LabelKey key = k->second;
    this->keyOfVisual.erase(k);
    VisualMark gone;
    gone.visual = v;
    gone.removed = true;
    gone.stamp = this->now;
    found.push_back(gone);
    const auto u = this->labelUsers.find(key);
    if (u == this->labelUsers.end() || --u->second > 0)
      continue;
    this->labelUsers.erase(u);
    const auto l = this->labelOf.find(key);
    if (l == this->labelOf.end())
      continue;
    const auto i = static_cast<std::size_t>(l->second);
    table->properties[i] = this->defaults;
    table->names[i].clear();
    this->freeLabels.insert(l->second);
    this->labelOf.erase(l);
  }

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
    else if (!this->freeLabels.empty())
    {
      // The lowest free id, so the same removals and spawns give the same
      // ids in every run.
      label = *this->freeLabels.begin();
      this->freeLabels.erase(this->freeLabels.begin());
      table->properties[static_cast<std::size_t>(label)] = props;
      table->names[static_cast<std::size_t>(label)] =
        scopedName(chain.front(), _ecm, "::", false);
      this->labelOf[key] = label;
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
      // Past 254 surfaces at once, the rest share one id with the default
      // properties; labeling per ping by range is the way past it.
      if (!this->warnedLabels)
      {
        gzwarn << "Sonar: more than 254 acoustic surfaces at once; the rest "
               << "share the default properties and one ground-truth label\n";
        this->warnedLabels = true;
      }
      if (table->properties.size() == static_cast<std::size_t>(kOtherLabel))
      {
        table->properties.push_back(this->defaults);
        table->names.emplace_back("other");
      }
      label = kOtherLabel;
    }
    if (label != kOtherLabel)
    {
      this->keyOfVisual[v] = key;
      ++this->labelUsers[key];
    }
    else
    {
      this->keyOfVisual.erase(v);
    }
    VisualMark mark;
    mark.visual = v;
    mark.topModel = chain.front();
    mark.label = label;
    mark.transparent = transparent;
    mark.stamp = this->now;
    found.push_back(mark);
  }

  this->table = table;
  std::lock_guard<std::mutex> lock(this->mutex);
  this->tableRequests.push_back({table, this->now});
  this->markRequests.insert(this->markRequests.end(), found.begin(),
    found.end());
}

//////////////////////////////////////////////////
void SonarPrivate::OnSceneUpdate()
{
  if (this->renderConn)
  {
    // Leave the render events once the last sonar and its cameras are gone,
    // so a world without a sonar stops rendering every step for nothing.
    if (this->haveSonars || !this->rigs.empty() || !this->waitingRigs.empty())
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->rigRequests.empty() || !this->rigRemovals.empty())
      return;
    this->preRenderConn.reset();
    this->renderConn.reset();
    return;
  }
  if (!this->haveSonars)
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
  // Rebuilt whenever the set of visuals changes, not only their number: a
  // model removed and another spawned in one step keeps the count.
  std::vector<unsigned int> ids;
  ids.reserve(all.size());
  for (const auto &v : all)
    ids.push_back(v->Id());
  if (ids == this->knownVisualIds)
    return;
  this->knownVisualIds = std::move(ids);
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
void SonarPrivate::DestroyRig(Rig &_rig)
{
  // Disconnect first, so no frame lands in a buffer that is going away.
  _rig.connections.clear();
  auto destroy = [this](const rendering::SensorPtr &_camera) {
    if (!_camera)
      return;
    if (auto parent = _camera->Parent())
      parent->RemoveChild(_camera);
    if (this->scene)
      this->scene->DestroySensor(_camera);
  };
  for (const auto &c : _rig.depth)
    destroy(c);
  for (const auto &c : _rig.seg)
    destroy(c);
  _rig.depth.clear();
  _rig.seg.clear();
}

//////////////////////////////////////////////////
void SonarPrivate::EraseRig(Entity _sensor)
{
  auto it = this->rigs.find(_sensor);
  if (it == this->rigs.end())
    return;
  this->DestroyRig(it->second);
  this->rigs.erase(it);
}

//////////////////////////////////////////////////
void SonarPrivate::ClearRigs()
{
  for (auto &[entity, rig] : this->rigs)
    this->DestroyRig(rig);
  this->rigs.clear();
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
    const std::string tag = base + "_sonar_" + std::to_string(c) + "_" +
      std::to_string(this->camerasBuilt++);

    auto depth = this->scene->CreateDepthCamera(tag + "_depth");
    auto seg = this->scene->CreateSegmentationCamera(tag + "_seg");
    if (!depth || !seg)
    {
      gzerr << "Sonar [" << base << "]: cannot create its cameras\n";
      if (depth)
        this->scene->DestroySensor(depth);
      if (seg)
        this->scene->DestroySensor(seg);
      this->DestroyRig(_rig);
      return false;
    }
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
  const auto t = std::chrono::duration_cast<nanoseconds>(this->scene->Time());

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->waitingRigs.insert(this->waitingRigs.end(),
      this->rigRequests.begin(), this->rigRequests.end());
    this->rigRequests.clear();
    this->waitingMarks.insert(this->waitingMarks.end(),
      this->markRequests.begin(), this->markRequests.end());
    this->markRequests.clear();
    this->waitingRemovals.insert(this->waitingRemovals.end(),
      this->rigRemovals.begin(), this->rigRemovals.end());
    this->rigRemovals.clear();
    this->waitingTables.insert(this->waitingTables.end(),
      this->tableRequests.begin(), this->tableRequests.end());
    this->tableRequests.clear();
  }

  // Time going back is a world reset: every sonar starts over on its rate
  // grid, and whatever was queued before it is due now.
  if (this->lastFrame.count() >= 0 && t < this->lastFrame)
  {
    for (auto &[entity, rig] : this->rigs)
    {
      rig.next = FirstPingAt(t, rig.request.sonar->Rate());
      rig.lastStamp = nanoseconds(-1);
    }
    for (auto &r : this->waitingRigs)
      r.stamp = std::min(r.stamp, t);
    for (auto &m : this->waitingMarks)
      m.stamp = std::min(m.stamp, t);
    for (auto &r : this->waitingRemovals)
      r.stamp = std::min(r.stamp, t);
    for (auto &w : this->waitingTables)
      w.stamp = std::min(w.stamp, t);
  }
  this->lastFrame = t;

  for (auto it = this->waitingRemovals.begin();
       it != this->waitingRemovals.end();)
  {
    if (it->stamp > t)
    {
      ++it;
      continue;
    }
    this->EraseRig(it->sensor);
    it = this->waitingRemovals.erase(it);
  }
  for (auto it = this->waitingTables.begin(); it != this->waitingTables.end();)
  {
    if (it->stamp > t)
    {
      ++it;
      continue;
    }
    this->renderTable = it->table;
    it = this->waitingTables.erase(it);
  }

  this->RefreshVisuals();

  // Marks, then sonars, that are due; one whose rendering visual is not in
  // the scene yet stays queued until it is.
  for (auto it = this->waitingMarks.begin(); it != this->waitingMarks.end();)
  {
    if (it->stamp > t)
    {
      ++it;
      continue;
    }
    if (it->removed)
    {
      this->marks.erase(it->visual);
      it = this->waitingMarks.erase(it);
    }
    else if (this->VisualOf(it->visual))
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
    if (it->stamp > t)
    {
      ++it;
      continue;
    }
    const Entity e = it->sensor;
    this->EraseRig(e);
    Rig &rig = this->rigs[e];
    if (this->BuildRig(*it, rig))
    {
      // The first ping on the rate grid from here, as every later one is.
      rig.next = FirstPingAt(t, it->sonar->Rate());
      it = this->waitingRigs.erase(it);
    }
    else
    {
      this->EraseRig(e);
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

  const std::shared_ptr<const LabelTable> table = this->renderTable;

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

    // gz-rendering has no visibility getter, so whatever was hidden here is
    // shown again; nothing else in these worlds hides a visual.
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
  this->ClearRigs();
  this->marks.clear();
  this->visualByEntity.clear();
  this->waterVisuals.clear();
  this->knownVisualIds.clear();
  this->lastFrame = nanoseconds(-1);
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
