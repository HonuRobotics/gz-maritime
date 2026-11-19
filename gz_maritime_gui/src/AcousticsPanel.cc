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
#include "AcousticsPanel.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <tinyxml2.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/param.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/maritime_gui/Acoustics.hh"
#include "gz/sim/maritime_gui/FieldControl.hh"
#include "gz/sim/maritime_gui/MarkerLayer.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

namespace
{
  /// \brief The name the plugin logs under.
  const char kLog[] = "AcousticsPanel";

  /// \brief The marker namespace of the drawings.
  const std::string kNs{"gz_maritime_gui/acoustics"};

  /// \brief Updates before the panel says where its pingers come from, so
  /// the GUI's state, which comes a moment after the panel, has come.
  constexpr int kSettled{30};

  /// \brief Period between two frames of the rings, of simulation time and
  /// of wall time both: a world running faster than real time does not
  /// send more frames.
  constexpr std::chrono::milliseconds kRingPeriod{50};


  /// \brief Period of wall time between two refreshes of the lists.
  constexpr std::chrono::milliseconds kListPeriod{250};

  /// \brief A colour as #rrggbb, for QML.
  /// \param[in] _c The colour.
  /// \return The text.
  QString Hex(const math::Color &_c)
  {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "#%02x%02x%02x",
        static_cast<int>(std::lround(_c.R() * 255.0f)),
        static_cast<int>(std::lround(_c.G() * 255.0f)),
        static_cast<int>(std::lround(_c.B() * 255.0f)));
    return QString(buf);
  }

  /// \brief A receiver's scoped name, short: without the world, model and
  /// sensor only.
  /// \param[in] _scoped The scoped name, from the world.
  /// \param[in] _world The world's name.
  /// \return The short name.
  std::string ShortName(const std::string &_scoped, const std::string &_world)
  {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true)
    {
      const auto end = _scoped.find("::", start);
      parts.push_back(_scoped.substr(start, end - start));
      if (end == std::string::npos)
        break;
      start = end + 2;
    }
    if (!parts.empty() && parts.front() == _world)
      parts.erase(parts.begin());
    if (parts.size() <= 1)
      return parts.empty() ? _scoped : parts.front();
    return parts.front() + " / " + parts.back();
  }

  /// \brief Read a child element's boolean.
  /// \param[in] _elem The plugin element.
  /// \param[in] _name The child's name.
  /// \param[in,out] _value Set when the child is a boolean.
  void ReadBool(const tinyxml2::XMLElement *_elem, const char *_name,
                bool &_value)
  {
    if (const auto *child = _elem->FirstChildElement(_name))
      child->QueryBoolText(&_value);
  }

  /// \brief Read a child element's number.
  /// \param[in] _elem The plugin element.
  /// \param[in] _name The child's name.
  /// \param[in,out] _value Set when the child is a number.
  void ReadDouble(const tinyxml2::XMLElement *_elem, const char *_name,
                  double &_value)
  {
    if (const auto *child = _elem->FirstChildElement(_name))
      child->QueryDoubleText(&_value);
  }
}

class gz::sim::maritime_gui::AcousticsPanelPrivate
{
  /// \brief Publish a `set` message, now or once the topic has a
  /// subscriber.
  /// \param[in] _topic The topic.
  /// \param[in] _msg The message.
  public: void Send(const std::string &_topic, const msgs::Param &_msg);

  /// \brief Send what waits for a subscriber.
  public: void SendPending();

  /// \brief The pingers: those the acoustics system lists, each as the
  /// GUI's ECM has it when it does. A panel loaded with the GUI finds them
  /// all in the ECM; one loaded later, or a world without pingers, takes
  /// what the list says, and asks nobody.
  /// \param[in] _ecm The GUI's ECM.
  /// \return The pingers.
  public: std::vector<PingerView> Pingers(const EntityComponentManager &_ecm);

  /// \brief The receivers' world poses, by topic.
  /// \param[in] _ecm The GUI's ECM.
  /// \param[in] _receivers The receivers.
  /// \return The poses of those found.
  public: std::map<std::string, math::Pose3d> ReceiverPoses(
      const EntityComponentManager &_ecm,
      const std::vector<ReceiverInfo> &_receivers);

  /// \brief The world's name.
  public: std::string world;

  /// \brief The acoustics system's topics.
  public: AcousticListener listener;

  /// \brief Make the drawings, on a MarkerManager service.
  /// \param[in] _service The service.
  public: void MakeLayers(const std::string &_service)
  {
    this->reachLayer = std::make_unique<MarkerLayer>(kNs + "/reach", _service);
    this->pingLayer = std::make_unique<MarkerLayer>(kNs + "/pings", _service);
    this->rayLayer =
        std::make_unique<MarkerLayer>(kNs + "/readings", _service);
  }

  /// \brief The drawings.
  public: std::unique_ptr<MarkerLayer> reachLayer;
  public: std::unique_ptr<MarkerLayer> pingLayer;
  public: std::unique_ptr<MarkerLayer> rayLayer;

  /// \brief What to draw.
  public: bool showReach{false};
  public: bool showPings{false};
  public: bool showReadings{false};

  /// \brief A drawing setting changed since the last draw.
  public: bool dirty{true};

  /// \brief Since when each pinger pings as it does.
  public: PingerChanges changes;

  /// \brief Each receiver's last reading with the pose it was heard at.
  public: std::map<std::string, BearingRay> rays;

  /// \brief The pingers as the rings were last drawn from, as text; with
  /// their positions only while the world was paused.
  public: std::string ringsKey;

  /// \brief m/s, the world's speed of sound.
  public: double speedOfSound{1500.0};

  /// \brief How many times slower than sound the rings spread.
  public: double slowDown{50.0};

  /// \brief m, farthest ring for a pinger with no reach.
  public: double maxRingRadius{200.0};

  /// \brief m, length of a reading's ray.
  public: double rayLength{30.0};

  /// \brief Simulation time a ray stays.
  public: std::chrono::steady_clock::duration rayLifetime{
      std::chrono::milliseconds(1500)};

  /// \brief World z of the water.
  public: double waterLevel{0.0};

  /// \brief What the reach and the pings are drawn on.
  public: Plane plane{Plane::kWater};

  /// \brief m under the water, for the depth plane.
  public: double planeDepth{5.0};

  /// \brief The topic of the receiver the receiver plane follows; empty
  /// for the first listed.
  public: std::string planeReceiver;

  /// \brief The world z drawn at, and the text that says it.
  public: double planeZ{0.0};
  public: QString planeText;

  /// \brief Simulation time and wall time of the last frame of the rings.
  public: std::optional<std::chrono::steady_clock::duration> lastRings;
  public: std::chrono::steady_clock::time_point lastRingsWall;

  /// \brief Wall time of the last refresh of the lists.
  public: std::chrono::steady_clock::time_point lastList;

  /// \brief The lists as QML has them, and their text form, to tell a
  /// change.
  public: QVariantList pingers;
  public: QVariantList receivers;
  public: QVariantMap readings;
  public: std::string pingersKey;
  public: std::string receiversKey;
  public: std::string readingsKey;

  /// \brief The status.
  public: QString status;

  /// \brief Whether the pingers and the receivers were logged as found,
  /// for the test, and the updates so far.
  public: bool logged{false};
  public: bool loggedReceivers{false};

  /// \brief The receiver whose depth was last logged as drawn at.
  public: std::string loggedPlane;
  public: int updates{0};

  /// \brief Receivers' sensor entities, by scoped name.
  public: std::unordered_map<std::string, Entity> sensorEntities;

  /// \brief Messages waiting for their topic's subscriber, and the
  /// publishers.
  public: std::map<std::string, msgs::Param> pending;
  public: std::map<std::string, transport::Node::Publisher> publishers;

  /// \brief Transport node. Last, so it is destroyed first.
  public: transport::Node node;
};

//////////////////////////////////////////////////
void AcousticsPanelPrivate::Send(const std::string &_topic,
    const msgs::Param &_msg)
{
  auto it = this->publishers.find(_topic);
  if (it == this->publishers.end())
  {
    it = this->publishers.emplace(_topic,
        this->node.Advertise<msgs::Param>(_topic)).first;
  }
  // A new publisher has no subscriber yet: the message waits for one, the
  // last one per topic, so a click is never lost to discovery.
  if (it->second.HasConnections())
  {
    it->second.Publish(_msg);
    this->pending.erase(_topic);
  }
  else
  {
    this->pending[_topic] = _msg;
  }
}

//////////////////////////////////////////////////
void AcousticsPanelPrivate::SendPending()
{
  for (auto it = this->pending.begin(); it != this->pending.end();)
  {
    auto &pub = this->publishers[it->first];
    if (pub.HasConnections())
    {
      pub.Publish(it->second);
      it = this->pending.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

//////////////////////////////////////////////////
std::vector<PingerView> AcousticsPanelPrivate::Pingers(
    const EntityComponentManager &_ecm)
{
  const auto state = ReadPingers(_ecm);
  auto pingers = MergePingers(this->listener.Pingers(), state);
  if (!this->logged && !pingers.empty() && ++this->updates > kSettled)
  {
    if (state.size() == pingers.size())
    {
      gzdbg << kLog << ": pingers from the GUI's state: " << pingers.size()
            << "\n";
    }
    else
    {
      gzdbg << kLog << ": pingers from the acoustics system's list: "
            << pingers.size() << ", " << state.size()
            << " of them in the GUI's state\n";
    }
    this->logged = true;
  }
  return pingers;
}

//////////////////////////////////////////////////
std::map<std::string, math::Pose3d> AcousticsPanelPrivate::ReceiverPoses(
    const EntityComponentManager &_ecm,
    const std::vector<ReceiverInfo> &_receivers)
{
  std::map<std::string, math::Pose3d> out;
  for (const auto &r : _receivers)
  {
    auto it = this->sensorEntities.find(r.name);
    if (it == this->sensorEntities.end() || !_ecm.HasEntity(it->second))
    {
      // The scoped name starts with the world's name; look it up without.
      std::string name = r.name;
      const std::string prefix = this->world + "::";
      if (name.rfind(prefix, 0) == 0)
        name = name.substr(prefix.size());
      const auto found = entitiesFromScopedName(name, _ecm);
      if (found.size() != 1u)
        continue;
      it = this->sensorEntities.insert_or_assign(r.name, *found.begin())
          .first;
    }
    out[r.topic] = worldPose(it->second, _ecm);
  }
  return out;
}

//////////////////////////////////////////////////
AcousticsPanel::AcousticsPanel()
  : dataPtr(std::make_unique<AcousticsPanelPrivate>())
{
  this->dataPtr->MakeLayers(kMarkerService);
}

//////////////////////////////////////////////////
AcousticsPanel::~AcousticsPanel() = default;

//////////////////////////////////////////////////
void AcousticsPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Acoustics";
  auto &d = *this->dataPtr;
  if (_pluginElem)
  {
    ReadBool(_pluginElem, "reach", d.showReach);
    ReadBool(_pluginElem, "pings", d.showPings);
    ReadBool(_pluginElem, "readings", d.showReadings);
    ReadDouble(_pluginElem, "speed_of_sound", d.speedOfSound);
    double slowDown = d.slowDown;
    ReadDouble(_pluginElem, "slow_down", slowDown);
    if (slowDown >= 1.0)
      d.slowDown = slowDown;
    ReadDouble(_pluginElem, "max_ring_radius", d.maxRingRadius);
    ReadDouble(_pluginElem, "ray_length", d.rayLength);
    double lifetime = 1.5;
    ReadDouble(_pluginElem, "ray_lifetime", lifetime);
    if (lifetime > 0.0)
    {
      d.rayLifetime = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(lifetime));
    }
    ReadDouble(_pluginElem, "water_level", d.waterLevel);
    if (const auto *elem = _pluginElem->FirstChildElement("plane"))
    {
      const char *text = elem->GetText();
      if (!text || !ParsePlane(text, d.plane))
        gzwarn << kLog << ": <plane> is water, depth, receiver or 3d\n";
    }
    double depth = d.planeDepth;
    ReadDouble(_pluginElem, "plane_depth", depth);
    if (depth >= 0.0)
      d.planeDepth = depth;
    if (const auto *elem = _pluginElem->FirstChildElement("plane_receiver"))
    {
      if (const char *text = elem->GetText())
        d.planeReceiver = text;
    }
    if (const auto *elem = _pluginElem->FirstChildElement("marker_service"))
    {
      if (const char *text = elem->GetText())
        d.MakeLayers(text);
    }
  }
  emit this->ShowReachChanged();
  emit this->ShowPingsChanged();
  emit this->ShowReadingsChanged();
  emit this->SlowDownChanged();
  emit this->PlaneChanged();
  emit this->PlaneDepthChanged();
  emit this->PlaneReceiverChanged();
}

//////////////////////////////////////////////////
void AcousticsPanel::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (d.world.empty())
  {
    const Entity world = _ecm.EntityByComponents(components::World());
    if (const auto *name = _ecm.Component<components::Name>(world))
    {
      d.world = name->Data();
      d.listener.Listen(d.world);
    }
  }
  d.SendPending();

  const auto pingers = d.Pingers(_ecm);
  const auto receivers = d.listener.Receivers();
  const auto last = d.listener.Last();
  if (!d.loggedReceivers && !receivers.empty())
  {
    gzdbg << kLog << ": receivers listed by the acoustics system: "
          << receivers.size() << "\n";
    d.loggedReceivers = true;
  }
  const auto now = _info.simTime;

  // The lists, a few times a second of wall time, and only when they
  // changed, so QML does not rebuild a row the user is typing in.
  const auto wall = std::chrono::steady_clock::now();
  if (wall - d.lastList >= kListPeriod)
  {
    d.lastList = wall;

    std::ostringstream pkey;
    QVariantList plist;
    for (const auto &p : pingers)
    {
      const auto &v = p.data;
      // What the row shows: a pinger from the acoustics system's list has
      // no generation to tell a change by.
      pkey << v.name << ' ' << v.frequency << ' ' << v.period << ' '
           << v.offset << ' ' << v.reach << ' ' << v.active << ';';
      QVariantMap m;
      m["name"] = QString::fromStdString(v.name);
      m["frequency"] = v.frequency / 1000.0;
      m["period"] = v.period;
      m["offset"] = v.offset;
      m["reach"] = v.reach;
      m["active"] = v.active;
      m["colour"] = Hex(FrequencyColor(v.frequency));
      plist.append(m);
    }
    if (pkey.str() != d.pingersKey)
    {
      d.pingersKey = pkey.str();
      d.pingers = plist;
      emit this->PingersChanged();
    }

    std::ostringstream rkey;
    QVariantList rlist;
    for (const auto &r : receivers)
    {
      rkey << r.name << ' ' << r.topic << ' ' << r.frequency << ' '
           << r.bandwidth << ';';
      QVariantMap m;
      m["name"] = QString::fromStdString(ShortName(r.name, d.world));
      m["topic"] = QString::fromStdString(r.topic);
      m["frequency"] = r.frequency / 1000.0;
      m["bandwidth"] = r.bandwidth / 1000.0;
      m["colour"] = Hex(FrequencyColor(r.frequency));
      rlist.append(m);
    }
    if (rkey.str() != d.receiversKey)
    {
      d.receiversKey = rkey.str();
      d.receivers = rlist;
      emit this->ReceiversChanged();
    }

    std::ostringstream lkey;
    QVariantMap rmap;
    for (const auto &r : receivers)
    {
      const auto it = last.find(r.topic);
      std::string text = "nothing heard";
      if (it != last.end())
      {
        const auto &h = it->second;
        const double age = std::chrono::duration<double>(now - h.stamp)
            .count();
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%+.0f° %.1f kHz %.0f dB%s, %.0f s ago",
            GZ_RTOD(h.bearing), h.frequency / 1000.0, h.amplitude,
            h.valid ? "" : " invalid", std::max(age, 0.0));
        text = buf;
      }
      lkey << r.topic << '=' << text << ';';
      rmap[QString::fromStdString(r.topic)] = QString::fromStdString(text);
    }
    if (lkey.str() != d.readingsKey)
    {
      d.readingsKey = lkey.str();
      d.readings = rmap;
      emit this->ReadingsChanged();
    }

    QString status;
    if (pingers.empty() && !d.listener.Heard())
      status = "No acoustics in this world.";
    else
      status = QString("%1 pingers, %2 receivers.")
          .arg(pingers.size()).arg(receivers.size());
    if (status != d.status)
    {
      d.status = status;
      emit this->StatusChanged();
    }
  }

  // The receivers' poses: for the plane that follows one, and for the
  // rays, which keep a reading's pose even while they are hidden.
  const auto poses = d.ReceiverPoses(_ecm, receivers);

  // The plane the reach and the pings are drawn on; a receiver's moves with
  // it.
  std::optional<double> receiverZ;
  std::string followed;
  if (d.plane == Plane::kReceiver)
  {
    for (const auto &r : receivers)
    {
      if (!d.planeReceiver.empty() && r.topic != d.planeReceiver)
        continue;
      const auto pose = poses.find(r.topic);
      if (pose != poses.end())
      {
        receiverZ = pose->second.Pos().Z();
        followed = ShortName(r.name, d.world);
      }
      break;
    }
  }
  d.planeZ = PlaneZ(d.plane, d.waterLevel, d.planeDepth, receiverZ);
  {
    char buf[128];
    if (d.plane == Plane::kWater)
      std::snprintf(buf, sizeof(buf), "on the water");
    else if (d.plane == Plane::k3D)
      std::snprintf(buf, sizeof(buf), "in 3D, under the water");
    else if (d.plane == Plane::kDepth)
      std::snprintf(buf, sizeof(buf), "%.1f m under the water", d.planeDepth);
    else if (receiverZ)
      std::snprintf(buf, sizeof(buf), "at %s's depth, %.1f m",
                    followed.c_str(), d.waterLevel - *receiverZ);
    else
      std::snprintf(buf, sizeof(buf), "on the water: no receiver found yet");
    const QString text(buf);
    if (receiverZ && followed != d.loggedPlane)
    {
      gzdbg << kLog << ": drawing at the depth of " << followed << "\n";
      d.loggedPlane = followed;
    }
    if (text != d.planeText)
    {
      d.planeText = text;
      emit this->StatusChanged();
    }
  }

  // The reach and the rays are asked for at every update and sent only
  // when they changed: a pinger switched off or a buoy dragged while the
  // world is paused is redrawn at once, and a circle that stays is not
  // sent again.
  if (d.showReach)
  {
    // The reach, and inside it where pings start to drop for the
    // receivers that have a dropout; in 3D, that ring on the water.
    std::vector<double> starts;
    for (const auto &r : receivers)
      starts.push_back(r.dropoutStart);
    const bool sphere = d.plane == Plane::k3D;
    auto markers = sphere ?
        ReachSpheres(pingers, d.waterLevel, d.reachLayer->Ns()) :
        ReachMarkers(pingers, d.planeZ, d.reachLayer->Ns());
    const double ringZ = sphere ?
        PlaneZ(Plane::kWater, d.waterLevel, 0.0, std::nullopt) : d.planeZ;
    for (auto &m : DropoutMarkers(pingers, starts, ringZ,
                                  d.reachLayer->Ns()))
    {
      markers.push_back(std::move(m));
    }
    d.reachLayer->DrawIfChanged(markers);
  }
  else if (d.reachLayer->Drawn())
  {
    d.reachLayer->Clear();
  }

  // A reading keeps the pose its receiver had when it came, hidden or
  // not, so a ray shown again later still points the way it was heard.
  KeepRays(d.rays, last, poses);
  if (d.showReadings)
  {
    d.rayLayer->DrawIfChanged(RayMarkers(
        LiveRays(d.rays, now, d.rayLifetime), d.rayLength, d.rayLayer->Ns()));
  }
  else if (d.rayLayer->Drawn())
  {
    d.rayLayer->Clear();
  }

  // The rings move with simulation time: a frame every so often of it,
  // and at once when a pinger or a setting changed. The rings follow their
  // pinger, but its position is in the key only while the world is paused:
  // running, the frames every so often follow it, and a pinger on anything
  // that moves would otherwise send a frame at every update.
  auto tracked = pingers;
  d.changes.Track(tracked, now, d.waterLevel);
  std::ostringstream key;
  for (const auto &p : tracked)
  {
    key << p.data.name << ' ' << p.data.active << ' ' << p.data.frequency
        << ' ' << p.data.period << ' ' << p.data.offset << ' '
        << p.data.reach;
    if (_info.paused)
      key << ' ' << p.position;
    key << ';';
  }
  // The plane the rings lie on, which a receiver's depth moves.
  key << static_cast<int>(d.plane) << ' ' << d.planeZ;
  const bool changed = d.dirty || key.str() != d.ringsKey;
  const bool ringsDue = changed || !d.lastRings || now < *d.lastRings ||
      (now - *d.lastRings >= kRingPeriod &&
       wall - d.lastRingsWall >= kRingPeriod);
  if (ringsDue)
  {
    if (d.showPings && d.plane == Plane::k3D)
    {
      d.pingLayer->DrawIfChanged(PingSpheres(tracked, d.waterLevel, now,
          d.speedOfSound / d.slowDown, d.maxRingRadius, d.pingLayer->Ns()));
    }
    else if (d.showPings)
    {
      d.pingLayer->DrawIfChanged(PingMarkers(tracked, d.planeZ, d.waterLevel,
          now, d.speedOfSound / d.slowDown, d.maxRingRadius,
          d.pingLayer->Ns()));
    }
    else if (d.pingLayer->Drawn())
    {
      d.pingLayer->Clear();
    }
    d.lastRings = now;
    d.lastRingsWall = wall;
    d.ringsKey = key.str();
  }
  d.dirty = false;
}

//////////////////////////////////////////////////
void AcousticsPanel::SetActive(const QString &_name, bool _active)
{
  auto &d = *this->dataPtr;
  if (d.world.empty())
    return;
  msgs::Param msg;
  auto &any = (*msg.mutable_params())["active"];
  any.set_type(msgs::Any::BOOLEAN);
  any.set_bool_value(_active);
  d.Send("/world/" + d.world + "/acoustics/" + _name.toStdString() + "/set",
         msg);
}

//////////////////////////////////////////////////
bool AcousticsPanel::SetReceiverFrequency(const QString &_topic,
    const QString &_kHz)
{
  double kHz{0.0};
  if (!ParseNumber(_kHz.toStdString(), kHz) || kHz < 0.0)
    return false;
  msgs::Param msg;
  auto &any = (*msg.mutable_params())["frequency"];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(kHz * 1000.0);
  this->dataPtr->Send(_topic.toStdString() + "/set", msg);
  return true;
}

//////////////////////////////////////////////////
bool AcousticsPanel::SetSlowDown(const QString &_text)
{
  double value{0.0};
  if (!ParseNumber(_text.toStdString(), value) || value < 1.0)
    return false;
  this->dataPtr->slowDown = value;
  this->dataPtr->dirty = true;
  emit this->SlowDownChanged();
  return true;
}

//////////////////////////////////////////////////
void AcousticsPanel::SetPlane(const QString &_plane)
{
  Plane plane{Plane::kWater};
  if (!ParsePlane(_plane.toStdString(), plane))
    return;
  this->dataPtr->plane = plane;
  this->dataPtr->dirty = true;
  emit this->PlaneChanged();
}

//////////////////////////////////////////////////
bool AcousticsPanel::SetPlaneDepth(const QString &_text)
{
  double value{0.0};
  if (!ParseNumber(_text.toStdString(), value) || value < 0.0)
    return false;
  this->dataPtr->planeDepth = value;
  this->dataPtr->dirty = true;
  emit this->PlaneDepthChanged();
  return true;
}

//////////////////////////////////////////////////
void AcousticsPanel::SetPlaneReceiver(const QString &_topic)
{
  this->dataPtr->planeReceiver = _topic.toStdString();
  this->dataPtr->dirty = true;
  emit this->PlaneReceiverChanged();
}

//////////////////////////////////////////////////
QString AcousticsPanel::CurrentPlane() const
{
  return QString::fromStdString(PlaneName(this->dataPtr->plane));
}

//////////////////////////////////////////////////
double AcousticsPanel::PlaneDepth() const
{
  return this->dataPtr->planeDepth;
}

//////////////////////////////////////////////////
QString AcousticsPanel::PlaneReceiver() const
{
  return QString::fromStdString(this->dataPtr->planeReceiver);
}

//////////////////////////////////////////////////
QString AcousticsPanel::PlaneText() const
{
  return this->dataPtr->planeText;
}

//////////////////////////////////////////////////
QVariantList AcousticsPanel::Pingers() const
{
  return this->dataPtr->pingers;
}

//////////////////////////////////////////////////
QVariantList AcousticsPanel::Receivers() const
{
  return this->dataPtr->receivers;
}

//////////////////////////////////////////////////
QVariantMap AcousticsPanel::Readings() const
{
  return this->dataPtr->readings;
}

//////////////////////////////////////////////////
bool AcousticsPanel::ShowReach() const
{
  return this->dataPtr->showReach;
}

//////////////////////////////////////////////////
void AcousticsPanel::SetShowReach(bool _show)
{
  if (_show == this->dataPtr->showReach)
    return;
  this->dataPtr->showReach = _show;
  this->dataPtr->dirty = true;
  emit this->ShowReachChanged();
}

//////////////////////////////////////////////////
bool AcousticsPanel::ShowPings() const
{
  return this->dataPtr->showPings;
}

//////////////////////////////////////////////////
void AcousticsPanel::SetShowPings(bool _show)
{
  if (_show == this->dataPtr->showPings)
    return;
  this->dataPtr->showPings = _show;
  this->dataPtr->dirty = true;
  emit this->ShowPingsChanged();
}

//////////////////////////////////////////////////
bool AcousticsPanel::ShowReadings() const
{
  return this->dataPtr->showReadings;
}

//////////////////////////////////////////////////
void AcousticsPanel::SetShowReadings(bool _show)
{
  if (_show == this->dataPtr->showReadings)
    return;
  this->dataPtr->showReadings = _show;
  this->dataPtr->dirty = true;
  emit this->ShowReadingsChanged();
}

//////////////////////////////////////////////////
double AcousticsPanel::SlowDown() const
{
  return this->dataPtr->slowDown;
}

//////////////////////////////////////////////////
QString AcousticsPanel::Status() const
{
  return this->dataPtr->status;
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::maritime_gui::AcousticsPanel, gz::gui::Plugin)
