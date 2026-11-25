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
// Stands for MarkerManager in the offscreen GUI test, where the 3D view,
// and so the real one, cannot run: it serves a marker array service and
// prints one line per marker it is sent, with the box its points fill, so
// the test can check what a panel draws and not only that it runs.
//
//   marker_recorder [service]     default: the panels' own, /marker
//
//   marker <ns> <id> add <points> <min x> <max x> <min y> <max y>
//   marker <ns> <id> delete

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

#include <gz/transport/Node.hh>

#include "gz/sim/maritime_gui/MarkerLayer.hh"

using namespace gz;

//////////////////////////////////////////////////
/// \brief Print the markers of a request.
/// \param[in] _req The markers.
/// \param[out] _rep Taken.
/// \return True.
bool OnArray(const msgs::Marker_V &_req, msgs::Boolean &_rep)
{
  for (const auto &m : _req.marker())
  {
    std::cout << "marker " << m.ns() << ' ' << m.id();
    if (m.action() != msgs::Marker::ADD_MODIFY)
    {
      std::cout << " delete" << std::endl;
      continue;
    }
    double minX = std::numeric_limits<double>::max();
    double minY = minX;
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = maxX;
    for (const auto &p : m.point())
    {
      minX = std::min(minX, p.x());
      maxX = std::max(maxX, p.x());
      minY = std::min(minY, p.y());
      maxY = std::max(maxY, p.y());
    }
    std::cout << " add " << m.point_size() << ' ' << minX << ' ' << maxX
              << ' ' << minY << ' ' << maxY << std::endl;
  }
  _rep.set_data(true);
  return true;
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  const std::string service =
      _argc > 1 ? _argv[1] : sim::maritime_gui::kMarkerService;
  transport::Node node;
  if (!node.Advertise(service + "_array", &OnArray))
  {
    std::cerr << "cannot serve " << service << "_array" << std::endl;
    return 1;
  }
  std::cout << "serving " << service << "_array" << std::endl;
  transport::waitForShutdown();
  return 0;
}
