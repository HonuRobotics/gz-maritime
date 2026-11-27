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
#include "gz/sim/maritime_gui/MarkerLayer.hh"

#include <cstdint>
#include <functional>
#include <set>
#include <utility>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/transport/Node.hh>

#include "gz/sim/maritime_gui/Fields.hh"

namespace gz::sim::maritime_gui
{
const char kMarkerService[] = "/marker";

//////////////////////////////////////////////////
struct MarkerLayer::Impl
{
  /// \brief The namespace.
  std::string ns;

  /// \brief MarkerManager's service.
  std::string service;

  /// \brief The ids on the scene.
  std::set<std::uint64_t> drawn;

  /// \brief The markers on the scene, serialized, to tell a change.
  std::string last;

  /// \brief Transport node. Last, so it is destroyed first.
  transport::Node node;

  /// \brief The markers with points, in this layer's namespace.
  /// \param[in] _markers The markers.
  /// \return A request that adds or replaces them.
  msgs::Marker_V WithPoints(const std::vector<msgs::Marker> &_markers) const
  {
    msgs::Marker_V out;
    for (const auto &m : _markers)
    {
      if (m.point_size() == 0)
        continue;
      auto *added = out.add_marker();
      *added = m;
      added->set_ns(this->ns);
    }
    return out;
  }

  /// \brief Send a drawing, and delete what it no longer draws.
  /// \param[in] _markers The markers with points.
  void Send(msgs::Marker_V _markers)
  {
    std::set<std::uint64_t> now;
    for (const auto &m : _markers.marker())
      now.insert(m.id());
    for (const auto id : this->drawn)
    {
      if (now.count(id) == 0u)
        *_markers.add_marker() = DeleteMarker(this->ns, id);
    }
    this->drawn = std::move(now);
    if (_markers.marker_size() == 0)
      return;
    // Without waiting for the answer.
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
  auto out = d.WithPoints(_markers);
  d.last.clear();
  out.SerializeToString(&d.last);
  d.Send(std::move(out));
}

//////////////////////////////////////////////////
bool MarkerLayer::DrawIfChanged(const std::vector<msgs::Marker> &_markers)
{
  auto &d = *this->impl;
  auto out = d.WithPoints(_markers);
  std::string now;
  out.SerializeToString(&now);
  if (now == d.last)
    return false;
  d.last = std::move(now);
  d.Send(std::move(out));
  return true;
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
}  // namespace gz::sim::maritime_gui
