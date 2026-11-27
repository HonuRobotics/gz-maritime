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
#ifndef GZ_SIM_MARITIME_GUI_MARKERLAYER_HH_
#define GZ_SIM_MARITIME_GUI_MARKERLAYER_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/marker.pb.h>

namespace gz::sim::maritime_gui
{
/// \brief MarkerManager's service in the GUI, where every panel draws by
/// default. Never the sensors' own, `/sensors/marker`: what is drawn there
/// shows in a vehicle's camera, whatever the marker's visibility says.
extern const char kMarkerService[];

/// \brief One drawing on the scene, through MarkerManager's array service,
/// and the one place that knows what is drawn: the markers drawn last are
/// replaced, and those no longer drawn, or empty now, deleted, since a
/// marker sent with no points keeps its old ones.
class MarkerLayer
{
  /// \brief Constructor.
  /// \param[in] _ns The drawing's namespace.
  /// \param[in] _service MarkerManager's service.
  public: MarkerLayer(const std::string &_ns, const std::string &_service);

  /// \brief Destructor: takes the drawing off the scene.
  public: ~MarkerLayer();

  /// \brief Draw: the markers with points added or replaced, those drawn
  /// before and empty or missing now deleted. Sent even when nothing
  /// changed.
  /// \param[in] _markers The markers; they take this layer's namespace.
  public: void Draw(const std::vector<msgs::Marker> &_markers);

  /// \brief Draw, unless these are the markers already on the scene: a
  /// drawing that is asked for at every update, and sent only when what it
  /// shows changed.
  /// \param[in] _markers The markers; they take this layer's namespace.
  /// \return True if anything was sent.
  public: bool DrawIfChanged(const std::vector<msgs::Marker> &_markers);

  /// \brief Take the drawing off the scene.
  public: void Clear();

  /// \brief Whether anything is drawn.
  /// \return True if it is.
  public: bool Drawn() const;

  /// \brief The namespace.
  /// \return It.
  public: const std::string &Ns() const;

  /// \brief The private data.
  private: struct Impl;

  /// \brief Private data pointer.
  private: std::unique_ptr<Impl> impl;
};
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_MARKERLAYER_HH_
