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
#include <sstream>
#include <utility>

#include <gz/math/Helpers.hh>
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

  /// \brief Segments around a ring: fewer than a reach circle, which is
  /// sent once, since the rings are sent again as they spread.
  constexpr int kRingSegments{64};

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
std::vector<PingerView> ParsePingers(const msgs::Param &_msg)
{
  std::vector<PingerView> out;
  for (const auto &child : _msg.children())
  {
    const auto &p = child.params();
    auto number = [&p](const std::string &_key, double _default)
    {
      const auto it = p.find(_key);
      return it == p.end() ? _default : it->second.double_value();
    };
    const auto name = p.find("name");
    if (name == p.end() || name->second.string_value().empty())
      continue;
    PingerView v;
    v.data.name = name->second.string_value();
    const auto active = p.find("active");
    v.data.active = active == p.end() || active->second.bool_value();
#define GZ_ACOUSTICS_GUI_READ(m, key) v.data.m = number(key, v.data.m);
    GZ_ACOUSTICS_PINGER_TABLE(GZ_ACOUSTICS_GUI_READ)
#undef GZ_ACOUSTICS_GUI_READ
    v.position.Set(number("x", 0.0), number("y", 0.0), number("z", 0.0));
    out.push_back(std::move(v));
  }
  std::sort(out.begin(), out.end(),
            [](const PingerView &_a, const PingerView &_b)
            { return _a.data.name < _b.data.name; });
  return out;
}

//////////////////////////////////////////////////
std::vector<PingerView> MergePingers(std::vector<PingerView> _listed,
    const std::vector<PingerView> &_state)
{
  for (const auto &fresh : _state)
  {
    const auto it = std::find_if(_listed.begin(), _listed.end(),
        [&fresh](const PingerView &_p)
        { return _p.data.name == fresh.data.name; });
    if (it != _listed.end())
      *it = fresh;
    else
      _listed.push_back(fresh);
  }
  std::sort(_listed.begin(), _listed.end(),
            [](const PingerView &_a, const PingerView &_b)
            { return _a.data.name < _b.data.name; });
  return _listed;
}

//////////////////////////////////////////////////
void PingerChanges::Track(std::vector<PingerView> &_pingers,
    const std::chrono::steady_clock::duration &_now, double _waterLevel)
{
  // Simulation time going back is a world reset: nothing is known of the
  // new run yet.
  if (_now < this->last)
    this->seen.clear();
  this->last = _now;

  std::map<std::string, Seen> now;
  for (auto &p : _pingers)
  {
    std::ostringstream schedule;
    schedule << p.data.active << ' ' << p.data.period << ' ' << p.data.offset
             << ' ' << (p.position.Z() < _waterLevel);
    Seen entry;
    entry.schedule = schedule.str();
    const auto before = this->seen.find(p.data.name);
    if (before != this->seen.end())
    {
      entry.since = before->second.schedule == entry.schedule ?
          before->second.since : _now;
    }
    p.since = entry.since;
    now[p.data.name] = std::move(entry);
  }
  this->seen = std::move(now);
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
    double _planeZ, double _waterLevel,
    const std::chrono::steady_clock::duration &_now,
    double _speed, double _maxRadius, const std::string &_ns)
{
  std::vector<msgs::Marker> out;
  if (!(_speed > 0.0))
    return out;
  std::uint64_t id = kFirstId;
  for (const auto &p : _pingers)
  {
    // A pinger that is off, or out of the water, sends nothing.
    if (!p.data.active || !(p.position.Z() < _waterLevel))
      continue;
    const double depth = _planeZ - p.position.Z();
    // Out to the reach; the cap is for a pinger with none.
    const double limit = p.data.reach > 0.0 ? p.data.reach : _maxRadius;
    if (!(limit > 0.0) || !(p.data.period > 0.0))
      continue;
    const auto window = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(limit / _speed));
    const math::Vector3d centre(p.position.X(), p.position.Y(), _planeZ);
    const math::Color colour = FrequencyColor(p.data.frequency);

    // With more pings on their way than rings to draw, every stride-th
    // ping of the schedule, so the same pings stay drawn as they spread.
    const auto period = acoustics::FromSeconds(p.data.period);
    const auto offset = acoustics::FromSeconds(p.data.offset);
    const std::int64_t stride = std::max<std::int64_t>(1,
        static_cast<std::int64_t>(std::ceil(
            limit / _speed / p.data.period / kMaxRings)));

    std::vector<std::vector<math::Vector3d>> shades(kShades);
    for (const auto &t : RecentPings(p.data, _now, window))
    {
      if (p.since && t < *p.since)
        continue;
      if (period.count() > 0 && ((t - offset) / period) % stride != 0)
        continue;
      const double sphere =
          _speed * std::chrono::duration<double>(_now - t).count();
      const double r = CutRadius(sphere, depth);
      if (!(r > 0.0) || sphere > limit)
        continue;
      const int shade = std::min(kShades - 1,
          static_cast<int>(kShades * sphere / limit));
      auto ring = CircleSegments(centre, r, false, kRingSegments);
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
    const math::Color colour = h.valid ? FrequencyColor(h.frequency) :
        kInvalid;
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
void KeepRays(std::map<std::string, BearingRay> &_kept,
    const std::map<std::string, HeardPing> &_last,
    const std::map<std::string, math::Pose3d> &_poses)
{
  for (auto it = _kept.begin(); it != _kept.end();)
  {
    if (_last.count(it->first) == 0u)
      it = _kept.erase(it);
    else
      ++it;
  }
  for (const auto &[topic, heard] : _last)
  {
    const auto kept = _kept.find(topic);
    if (kept != _kept.end() && kept->second.ping.stamp == heard.stamp &&
        kept->second.ping.bearing == heard.bearing)
    {
      continue;
    }
    const auto pose = _poses.find(topic);
    if (pose != _poses.end())
      _kept[topic] = {pose->second, heard};
  }
}

//////////////////////////////////////////////////
std::vector<BearingRay> LiveRays(
    const std::map<std::string, BearingRay> &_kept,
    const std::chrono::steady_clock::duration &_now,
    const std::chrono::steady_clock::duration &_lifetime)
{
  std::vector<BearingRay> out;
  for (const auto &[topic, ray] : _kept)
  {
    if (_now >= ray.ping.stamp && _now - ray.ping.stamp <= _lifetime)
      out.push_back(ray);
  }
  return out;
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

  /// \brief The pingers as the ground truth last listed them.
  std::vector<PingerView> pingers;

  /// \brief The last reading per topic.
  std::map<std::string, HeardPing> last;

  /// \brief The topics subscribed to.
  std::set<std::string> subscribed;

  /// \brief Transport node. Last, so it is destroyed first and no callback
  /// runs on the rest after it.
  transport::Node node;

  /// \brief The pingers' ground truth.
  /// \param[in] _msg The message.
  void OnPingers(const msgs::Param &_msg)
  {
    auto list = ParsePingers(_msg);
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->pingers = std::move(list);
  }

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
  d.node.Subscribe("/world/" + _world + "/acoustics/pingers",
                   &Impl::OnPingers, &d);
}

//////////////////////////////////////////////////
std::vector<ReceiverInfo> AcousticListener::Receivers() const
{
  const std::lock_guard<std::mutex> lock(this->impl->mutex);
  return this->impl->receivers;
}

//////////////////////////////////////////////////
std::vector<PingerView> AcousticListener::Pingers() const
{
  const std::lock_guard<std::mutex> lock(this->impl->mutex);
  return this->impl->pingers;
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
