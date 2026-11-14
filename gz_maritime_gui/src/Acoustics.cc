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
#include "gz/sim/maritime_gui/Acoustics.hh"

#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>
#include <utility>

#include <gz/math/Helpers.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/acoustics/Hearing.hh"
#include "gz/sim/components/AcousticPinger.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/maritime_gui/Flow.hh"

namespace gz::sim::maritime_gui
{
namespace
{
  /// \brief The id of a drawing's first marker. Not 0: MarkerManager gives
  /// a marker with id 0 a new random id, so every redraw would add one.
  constexpr std::uint64_t kFirstId{1};

  /// \brief Shades a ping fades through as it spreads.
  constexpr int kShades{3};

  /// \brief The colour of a pinger that is off, and of an invalid reading.
  const math::Color kOff(0.55f, 0.55f, 0.55f, 1.0f);
  const math::Color kInvalid(0.9f, 0.1f, 0.1f, 1.0f);

  /// \brief A colour from a hue, full saturation and value.
  /// \param[in] _hue Degrees.
  /// \return The colour.
  math::Color FromHue(double _hue)
  {
    const double h = std::fmod(std::fmod(_hue, 360.0) + 360.0, 360.0) / 60.0;
    const double x = 1.0 - std::abs(std::fmod(h, 2.0) - 1.0);
    double r{0}, g{0}, b{0};
    switch (static_cast<int>(h))
    {
      case 0: r = 1; g = x; break;
      case 1: r = x; g = 1; break;
      case 2: g = 1; b = x; break;
      case 3: g = x; b = 1; break;
      case 4: r = x; b = 1; break;
      default: r = 1; b = x; break;
    }
    return math::Color(static_cast<float>(r), static_cast<float>(g),
                       static_cast<float>(b), 1.0f);
  }

  /// \brief A colour blended towards white.
  /// \param[in] _c The colour.
  /// \param[in] _t 0 keeps it, 1 is white.
  /// \return The blend.
  math::Color Lighter(const math::Color &_c, double _t)
  {
    const float t = static_cast<float>(std::clamp(_t, 0.0, 1.0));
    return math::Color(_c.R() + (1.0f - _c.R()) * t,
                       _c.G() + (1.0f - _c.G()) * t,
                       _c.B() + (1.0f - _c.B()) * t, 1.0f);
  }

  /// \brief A colour as a key, so equal colours share a marker.
  /// \param[in] _c The colour.
  /// \return The key.
  std::uint32_t ColorKey(const math::Color &_c)
  {
    return _c.AsRGBA();
  }
}

//////////////////////////////////////////////////
std::vector<PingerView> ReadPingers(const EntityComponentManager &_ecm)
{
  std::vector<PingerView> out;
  _ecm.Each<components::AcousticPinger>(
      [&](const Entity &_link, const components::AcousticPinger *_comp) -> bool
      {
        PingerView v;
        v.data = _comp->Data();
        v.position = (worldPose(_link, _ecm) * v.data.pose).Pos();
        out.push_back(std::move(v));
        return true;
      });
  std::sort(out.begin(), out.end(),
            [](const PingerView &_a, const PingerView &_b)
            { return _a.data.name < _b.data.name; });
  return out;
}

//////////////////////////////////////////////////
math::Color FrequencyColor(double _hz)
{
  const double t = std::clamp((_hz - 25000.0) / 15000.0, 0.0, 1.0);
  return FromHue(30.0 + 250.0 * t);
}

//////////////////////////////////////////////////
double CutRadius(double _radius, double _depth)
{
  const double r2 = _radius * _radius - _depth * _depth;
  return r2 > 0.0 ? std::sqrt(r2) : 0.0;
}

//////////////////////////////////////////////////
std::vector<math::Vector3d> CircleSegments(const math::Vector3d &_centre,
    double _radius, bool _dashed, int _segments)
{
  std::vector<math::Vector3d> out;
  if (!(_radius > 0.0))
    return out;
  const int n = std::max(_segments, 8);
  for (int i = 0; i < n; ++i)
  {
    if (_dashed && i % 2 == 1)
      continue;
    const double a0 = 2.0 * GZ_PI * i / n;
    const double a1 = 2.0 * GZ_PI * (i + 1) / n;
    out.push_back(_centre + math::Vector3d(std::cos(a0), std::sin(a0), 0.0) *
                  _radius);
    out.push_back(_centre + math::Vector3d(std::cos(a1), std::sin(a1), 0.0) *
                  _radius);
  }
  return out;
}

//////////////////////////////////////////////////
std::vector<std::chrono::steady_clock::duration> RecentPings(
    const acoustics::PingerData &_pinger,
    const std::chrono::steady_clock::duration &_now,
    const std::chrono::steady_clock::duration &_window)
{
  const auto after = std::max(_now - _window,
                              -std::chrono::steady_clock::duration(1));
  return acoustics::EmissionTimes(_pinger, after, _now);
}

//////////////////////////////////////////////////
bool ParsePlane(const std::string &_name, Plane &_plane)
{
  if (_name == "water")
    _plane = Plane::kWater;
  else if (_name == "depth")
    _plane = Plane::kDepth;
  else if (_name == "receiver")
    _plane = Plane::kReceiver;
  else if (_name == "3d")
    _plane = Plane::k3D;
  else
    return false;
  return true;
}

//////////////////////////////////////////////////
std::string PlaneName(Plane _plane)
{
  switch (_plane)
  {
    case Plane::kDepth: return "depth";
    case Plane::kReceiver: return "receiver";
    case Plane::k3D: return "3d";
    default: return "water";
  }
}

//////////////////////////////////////////////////
double PlaneZ(Plane _plane, double _waterLevel, double _depth,
              const std::optional<double> &_receiverZ)
{
  // Just above the water, so the drawn sea does not hide what lies on it.
  constexpr double kAboveWater{0.08};
  switch (_plane)
  {
    case Plane::kDepth:
      return _waterLevel - std::max(_depth, 0.0);
    case Plane::kReceiver:
      if (_receiverZ)
        return *_receiverZ;
      break;
    default:
      break;
  }
  return _waterLevel + kAboveWater;
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> ReachMarkers(const std::vector<PingerView> &_pingers,
    double _planeZ, const std::string &_ns)
{
  std::vector<msgs::Marker> out;
  std::uint64_t id = kFirstId;
  for (const auto &p : _pingers)
  {
    const double r = CutRadius(p.data.reach, _planeZ - p.position.Z());
    if (!(r > 0.0))
      continue;
    const math::Vector3d centre(p.position.X(), p.position.Y(), _planeZ);
    out.push_back(SolidMarker(_ns, id++, msgs::Marker::LINE_LIST,
        p.data.active ? FrequencyColor(p.data.frequency) : kOff,
        CircleSegments(centre, r, !p.data.active)));
  }
  return out;
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> PingMarkers(const std::vector<PingerView> &_pingers,
    double _planeZ, const std::chrono::steady_clock::duration &_now,
    double _speed, double _maxRadius, const std::string &_ns)
{
  std::vector<msgs::Marker> out;
  if (!(_speed > 0.0))
    return out;
  std::uint64_t id = kFirstId;
  for (const auto &p : _pingers)
  {
    if (!p.data.active)
      continue;
    const double depth = _planeZ - p.position.Z();
    // Out to the reach, or the cap.
    double limit = _maxRadius;
    if (p.data.reach > 0.0)
      limit = std::min(limit, p.data.reach);
    if (!(limit > 0.0))
      continue;
    const auto window = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(limit / _speed));
    const math::Vector3d centre(p.position.X(), p.position.Y(), _planeZ);
    const math::Color colour = FrequencyColor(p.data.frequency);

    std::vector<std::vector<math::Vector3d>> shades(kShades);
    for (const auto &t : RecentPings(p.data, _now, window))
    {
      const double sphere =
          _speed * std::chrono::duration<double>(_now - t).count();
      const double r = CutRadius(sphere, depth);
      if (!(r > 0.0) || sphere > limit)
        continue;
      const int shade = std::min(kShades - 1,
          static_cast<int>(kShades * sphere / limit));
      auto ring = CircleSegments(centre, r, false);
      auto &dst = shades[static_cast<std::size_t>(shade)];
      dst.insert(dst.end(), ring.begin(), ring.end());
    }
    // A marker per shade, always, so the ids stay with the pinger; an
    // empty one is deleted by the layer.
    for (int s = 0; s < kShades; ++s)
    {
      out.push_back(SolidMarker(_ns, id++, msgs::Marker::LINE_LIST,
          Lighter(colour, 0.25 * s), shades[static_cast<std::size_t>(s)]));
    }
  }
  return out;
}

//////////////////////////////////////////////////
std::vector<math::Vector3d> SphereSegments(const math::Vector3d &_centre,
    double _radius, double _ceiling, int _meridians, int _parallels,
    int _segments)
{
  std::vector<math::Vector3d> out;
  if (!(_radius > 0.0) || _centre.Z() - _radius >= _ceiling)
    return out;
  const int n = std::max(_segments, 8);

  // A segment, cut at the ceiling: kept below it, dropped above it. The cut
  // end is put on the circle where the sphere meets the ceiling, not on the
  // chord, so a meridian ends on that circle.
  const double cutRadius = CutRadius(_radius, _ceiling - _centre.Z());
  auto add = [&out, &_centre, _ceiling, cutRadius](math::Vector3d _a,
                                                   math::Vector3d _b)
  {
    const bool aIn = _a.Z() <= _ceiling;
    const bool bIn = _b.Z() <= _ceiling;
    if (!aIn && !bIn)
      return;
    if (aIn != bIn)
    {
      const double t = (_ceiling - _a.Z()) / (_b.Z() - _a.Z());
      math::Vector3d cut = _a + (_b - _a) * t;
      math::Vector3d h(cut.X() - _centre.X(), cut.Y() - _centre.Y(), 0.0);
      if (h.Length() > 1e-12)
      {
        h = h.Normalized() * cutRadius;
        cut.Set(_centre.X() + h.X(), _centre.Y() + h.Y(), _ceiling);
      }
      if (aIn)
        _b = cut;
      else
        _a = cut;
    }
    out.push_back(_a);
    out.push_back(_b);
  };

  // Meridians: half circles from pole to pole, around the vertical.
  const int meridians = std::max(_meridians, 2);
  for (int m = 0; m < meridians; ++m)
  {
    const double lon = 2.0 * GZ_PI * m / meridians;
    for (int i = 0; i < n / 2; ++i)
    {
      const double l0 = -0.5 * GZ_PI + GZ_PI * i / (n / 2);
      const double l1 = -0.5 * GZ_PI + GZ_PI * (i + 1) / (n / 2);
      auto at = [&](double _lat)
      {
        return _centre + math::Vector3d(std::cos(_lat) * std::cos(lon),
            std::cos(_lat) * std::sin(lon), std::sin(_lat)) * _radius;
      };
      add(at(l0), at(l1));
    }
  }

  // Parallels, evenly between the poles.
  const int parallels = std::max(_parallels, 1);
  for (int p = 1; p <= parallels; ++p)
  {
    const double lat = -0.5 * GZ_PI + GZ_PI * p / (parallels + 1);
    const double z = _centre.Z() + _radius * std::sin(lat);
    if (z > _ceiling)
      continue;
    const auto ring = CircleSegments(math::Vector3d(_centre.X(), _centre.Y(),
        z), _radius * std::cos(lat), false, n);
    out.insert(out.end(), ring.begin(), ring.end());
  }

  // Where it meets the ceiling.
  const auto cut = CircleSegments(math::Vector3d(_centre.X(), _centre.Y(),
      _ceiling), cutRadius, false, n);
  out.insert(out.end(), cut.begin(), cut.end());
  return out;
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> ReachSpheres(const std::vector<PingerView> &_pingers,
    double _waterLevel, const std::string &_ns)
{
  std::vector<msgs::Marker> out;
  std::uint64_t id = kFirstId;
  for (const auto &p : _pingers)
  {
    if (!(p.data.reach > 0.0))
      continue;
    out.push_back(SolidMarker(_ns, id++, msgs::Marker::LINE_LIST,
        p.data.active ? FrequencyColor(p.data.frequency) : kOff,
        SphereSegments(p.position, p.data.reach, _waterLevel, 12, 5)));
  }
  return out;
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> PingSpheres(const std::vector<PingerView> &_pingers,
    double _waterLevel, const std::chrono::steady_clock::duration &_now,
    double _speed, double _maxRadius, const std::string &_ns)
{
  std::vector<msgs::Marker> out;
  if (!(_speed > 0.0))
    return out;
  std::uint64_t id = kFirstId;
  for (const auto &p : _pingers)
  {
    if (!p.data.active)
      continue;
    double limit = _maxRadius;
    if (p.data.reach > 0.0)
      limit = std::min(limit, p.data.reach);
    if (!(limit > 0.0))
      continue;
    const auto window = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(limit / _speed));
    const math::Color colour = FrequencyColor(p.data.frequency);

    std::vector<std::vector<math::Vector3d>> shades(kShades);
    for (const auto &t : RecentPings(p.data, _now, window))
    {
      const double r =
          _speed * std::chrono::duration<double>(_now - t).count();
      if (!(r > 0.0) || r > limit)
        continue;
      const int shade = std::min(kShades - 1,
          static_cast<int>(kShades * r / limit));
      // Sparser than the reach: a few shells drawn at once stay readable.
      auto shell = SphereSegments(p.position, r, _waterLevel, 6, 3, 32);
      auto &dst = shades[static_cast<std::size_t>(shade)];
      dst.insert(dst.end(), shell.begin(), shell.end());
    }
    for (int s = 0; s < kShades; ++s)
    {
      out.push_back(SolidMarker(_ns, id++, msgs::Marker::LINE_LIST,
          Lighter(colour, 0.25 * s), shades[static_cast<std::size_t>(s)]));
    }
  }
  return out;
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> DropoutMarkers(
    const std::vector<PingerView> &_pingers, std::vector<double> _starts,
    double _planeZ, const std::string &_ns, std::uint64_t _firstId)
{
  std::sort(_starts.begin(), _starts.end());
  _starts.erase(std::unique(_starts.begin(), _starts.end()), _starts.end());
  std::vector<msgs::Marker> out;
  std::uint64_t id = _firstId;
  for (const auto &p : _pingers)
  {
    if (!(p.data.reach > 0.0))
      continue;
    const math::Color colour = Lighter(
        p.data.active ? FrequencyColor(p.data.frequency) : kOff, 0.5);
    for (const double start : _starts)
    {
      if (!(start > 0.0 && start < 1.0))
        continue;
      const double r = CutRadius(p.data.reach * start, _planeZ - p.position.Z());
      if (!(r > 0.0))
        continue;
      out.push_back(SolidMarker(_ns, id++, msgs::Marker::LINE_LIST, colour,
          CircleSegments(math::Vector3d(p.position.X(), p.position.Y(),
                                        _planeZ), r, true)));
    }
  }
  return out;
}

//////////////////////////////////////////////////
std::vector<msgs::Marker> RayMarkers(const std::vector<BearingRay> &_rays,
    double _length, const std::string &_ns)
{
  // One marker per colour: Gazebo's renderer ignores colours given point by
  // point on a line list.
  std::map<std::uint32_t, std::pair<math::Color,
      std::vector<math::Vector3d>>> byColour;
  for (const auto &ray : _rays)
  {
    const auto &h = ray.ping;
    const math::Vector3d local(
        std::cos(h.elevation) * std::cos(h.bearing),
        std::cos(h.elevation) * std::sin(h.bearing),
        std::sin(h.elevation));
    const math::Vector3d start = ray.pose.Pos();
    const math::Vector3d end =
        start + ray.pose.Rot().RotateVector(local) * _length;
    // Three steps of fading, so a few markers carry every ray.
    const int step = std::clamp(static_cast<int>(3.0 * ray.fade), 0, 2);
    const math::Color colour = Lighter(
        h.valid ? FrequencyColor(h.frequency) : kInvalid, 0.3 * step);
    auto &entry = byColour[ColorKey(colour)];
    entry.first = colour;
    entry.second.push_back(start);
    entry.second.push_back(end);
  }
  std::vector<msgs::Marker> out;
  std::uint64_t id = kFirstId;
  for (const auto &[key, entry] : byColour)
  {
    out.push_back(SolidMarker(_ns, id++, msgs::Marker::LINE_LIST,
                              entry.first, entry.second));
  }
  return out;
}

//////////////////////////////////////////////////
struct MarkerLayer::Impl
{
  /// \brief The namespace.
  std::string ns;

  /// \brief MarkerManager's service.
  std::string service;

  /// \brief The ids on the scene.
  std::set<std::uint64_t> drawn;

  /// \brief Transport node. Last, so it is destroyed first.
  transport::Node node;

  /// \brief Send markers without waiting.
  /// \param[in] _markers The markers.
  void Send(const msgs::Marker_V &_markers)
  {
    std::function<void(const msgs::Boolean &, const bool)> cb =
        [](const msgs::Boolean &, const bool) {};
    this->node.Request(this->service + "_array", _markers, cb);
  }
};

//////////////////////////////////////////////////
MarkerLayer::MarkerLayer(const std::string &_ns, const std::string &_service)
  : impl(std::make_unique<Impl>())
{
  this->impl->ns = _ns;
  this->impl->service = _service;
}

//////////////////////////////////////////////////
MarkerLayer::~MarkerLayer()
{
  this->Clear();
}

//////////////////////////////////////////////////
void MarkerLayer::Draw(const std::vector<msgs::Marker> &_markers)
{
  auto &d = *this->impl;
  msgs::Marker_V out;
  std::set<std::uint64_t> now;
  for (const auto &m : _markers)
  {
    // A marker sent with no points keeps its old ones, so an empty one is
    // deleted instead.
    if (m.point_size() == 0)
      continue;
    auto *added = out.add_marker();
    *added = m;
    added->set_ns(d.ns);
    now.insert(m.id());
  }
  for (const auto id : d.drawn)
  {
    if (now.count(id) == 0u)
      *out.add_marker() = DeleteMarker(d.ns, id);
  }
  d.drawn = std::move(now);
  if (out.marker_size() > 0)
    d.Send(out);
}

//////////////////////////////////////////////////
void MarkerLayer::Clear()
{
  this->Draw({});
}

//////////////////////////////////////////////////
bool MarkerLayer::Drawn() const
{
  return !this->impl->drawn.empty();
}

//////////////////////////////////////////////////
const std::string &MarkerLayer::Ns() const
{
  return this->impl->ns;
}

//////////////////////////////////////////////////
std::optional<HeardPing> ParseReading(const msgs::Param &_msg)
{
  const auto &p = _msg.params();
  auto number = [&p](const std::string &_key, double &_out)
  {
    const auto it = p.find(_key);
    if (it == p.end() || it->second.type() != msgs::Any::DOUBLE)
      return false;
    _out = it->second.double_value();
    return true;
  };
  HeardPing h;
  if (!number("bearing", h.bearing))
    return std::nullopt;
  number("elevation", h.elevation);
  number("frequency", h.frequency);
  number("amplitude", h.amplitude);
  const auto valid = p.find("valid");
  h.valid = valid == p.end() || valid->second.bool_value();
  h.stamp = msgs::Convert(_msg.header().stamp());
  return h;
}

//////////////////////////////////////////////////
std::vector<ReceiverInfo> ParseReceivers(const msgs::Param &_msg)
{
  std::vector<ReceiverInfo> out;
  for (const auto &child : _msg.children())
  {
    const auto &p = child.params();
    auto text = [&p](const std::string &_key)
    {
      const auto it = p.find(_key);
      return it == p.end() ? std::string() : it->second.string_value();
    };
    auto number = [&p](const std::string &_key)
    {
      const auto it = p.find(_key);
      return it == p.end() ? 0.0 : it->second.double_value();
    };
    ReceiverInfo r;
    r.name = text("name");
    r.topic = text("topic");
    r.frameId = text("frame_id");
    r.frequency = number("frequency");
    r.bandwidth = number("bandwidth");
    if (p.count("dropout_start") > 0u)
      r.dropoutStart = number("dropout_start");
    if (!r.topic.empty())
      out.push_back(std::move(r));
  }
  std::sort(out.begin(), out.end(),
            [](const ReceiverInfo &_a, const ReceiverInfo &_b)
            { return _a.name < _b.name; });
  return out;
}

//////////////////////////////////////////////////
struct AcousticListener::Impl
{
  /// \brief Guards everything below across the transport threads.
  mutable std::mutex mutex;

  /// \brief The world listened to.
  std::string world;

  /// \brief The receivers as last listed.
  std::vector<ReceiverInfo> receivers;

  /// \brief Whether a list came.
  bool heard{false};

  /// \brief The last reading per topic.
  std::map<std::string, HeardPing> last;

  /// \brief The topics subscribed to.
  std::set<std::string> subscribed;

  /// \brief Transport node. Last, so it is destroyed first and no callback
  /// runs on the rest after it.
  transport::Node node;

  /// \brief A list of receivers.
  /// \param[in] _msg The message.
  void OnReceivers(const msgs::Param &_msg)
  {
    auto list = ParseReceivers(_msg);
    std::vector<std::string> fresh;
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      this->receivers = list;
      this->heard = true;
      for (const auto &r : list)
      {
        if (this->subscribed.insert(r.topic).second)
          fresh.push_back(r.topic);
      }
    }
    for (const auto &topic : fresh)
    {
      std::function<void(const msgs::Param &)> cb =
          [this, topic](const msgs::Param &_reading)
          {
            const auto h = ParseReading(_reading);
            if (!h)
              return;
            const std::lock_guard<std::mutex> lock(this->mutex);
            this->last[topic] = *h;
          };
      this->node.Subscribe(topic, cb);
    }
  }
};

//////////////////////////////////////////////////
AcousticListener::AcousticListener()
  : impl(std::make_unique<Impl>())
{
}

//////////////////////////////////////////////////
AcousticListener::~AcousticListener() = default;

//////////////////////////////////////////////////
void AcousticListener::Listen(const std::string &_world)
{
  auto &d = *this->impl;
  {
    const std::lock_guard<std::mutex> lock(d.mutex);
    if (_world.empty() || _world == d.world)
      return;
    d.world = _world;
  }
  d.node.Subscribe("/world/" + _world + "/acoustics/receivers",
                   &Impl::OnReceivers, &d);
}

//////////////////////////////////////////////////
std::vector<ReceiverInfo> AcousticListener::Receivers() const
{
  const std::lock_guard<std::mutex> lock(this->impl->mutex);
  return this->impl->receivers;
}

//////////////////////////////////////////////////
std::map<std::string, HeardPing> AcousticListener::Last() const
{
  const std::lock_guard<std::mutex> lock(this->impl->mutex);
  return this->impl->last;
}

//////////////////////////////////////////////////
bool AcousticListener::Heard() const
{
  const std::lock_guard<std::mutex> lock(this->impl->mutex);
  return this->impl->heard;
}
}  // namespace gz::sim::maritime_gui
