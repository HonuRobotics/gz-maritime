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
#ifndef GZ_SIM_MARKED_SHAPES_MARKEDSHAPES_HH_
#define GZ_SIM_MARKED_SHAPES_MARKEDSHAPES_HH_

#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>

namespace gz::sim::marked_shapes
{
/// \brief One marked shape of a link, resolved once when the link is found.
struct Shape
{
  /// \brief Pose of the shape in the link frame.
  math::Pose3d pose;

  /// \brief Half extents of the shape's bounding box in the shape frame.
  math::Vector3d half;

  /// \brief Projected area of the whole shape along each shape axis, m^2.
  math::Vector3d area;

  /// \brief Drag coefficient.
  double cd{1.0};
};

/// \brief Which side of the water level a fluid acts on.
enum class Side
{
  /// \brief The part above the water: the air.
  kAbove,

  /// \brief The part below the water: the water.
  kBelow
};

/// \brief The part of a shape on one side of the water level.
struct Part
{
  /// \brief Whether any of the shape is on that side.
  bool any{false};

  /// \brief The share of the shape's height on that side, in [0, 1].
  double fraction{0.0};

  /// \brief Centre of that part, world frame, m.
  math::Vector3d centre;
};

/// \brief Cut a shape's bounding box at the water level.
/// \param[in] _worldPose Pose of the shape in the world.
/// \param[in] _half Half extents of its bounding box, shape frame.
/// \param[in] _waterLevel World z of the water.
/// \param[in] _side The side to keep.
/// \return The part on that side.
Part Cut(const math::Pose3d &_worldPose, const math::Vector3d &_half,
         double _waterLevel, Side _side);

/// \brief Resolve one collision into a shape, when it carries the mark.
/// Boxes, cylinders, spheres, capsules and ellipsoids keep their shape; a
/// mesh counts as its bounding box.
/// \param[in] _ecm The entity component manager.
/// \param[in] _collision The collision entity.
/// \param[in] _mark The attribute that marks it, read as a bool.
/// \param[in] _cdMark The optional attribute with its drag coefficient.
/// \param[in] _defaultCd Drag coefficient without one.
/// \param[out] _shape The shape, when the collision is marked.
/// \return True if the collision is a marked shape.
bool Resolve(const EntityComponentManager &_ecm, const Entity _collision,
             const std::string &_mark, const std::string &_cdMark,
             double _defaultCd, Shape &_shape);

/// \brief The velocity of a fluid at a point, world frame.
using FluidVelocity = std::function<math::Vector3d(
    const math::Vector3d &_point,
    const std::chrono::steady_clock::duration &_time)>;

/// \brief The links that carry collisions marked for one fluid, found on
/// every model, whenever it shows up, and the drag that fluid applies to
/// them.
///
/// A system calls Find at the start of PreUpdate, ApplyDrag when it pushes,
/// NoteNew in PostUpdate and Reset in Reset. EachNew only sees an entity in
/// the step it was made in; NoteNew notes it after every system's
/// PreUpdate, whichever system made it, and Find resolves it at the next
/// step.
class MarkedLinks
{
  /// \brief Constructor.
  /// \param[in] _mark The attribute a collision is marked with.
  /// \param[in] _cdMark The optional per shape drag coefficient attribute.
  public: MarkedLinks(std::string _mark, std::string _cdMark);

  /// \brief Set the drag coefficient of shapes that do not set their own.
  /// Takes effect for links found after the call.
  /// \param[in] _cd Drag coefficient.
  public: void SetDefaultCd(double _cd);

  /// \brief Find the marked shapes of every link not seen yet, or of every
  /// link after construction or a Reset.
  /// \param[in] _ecm The entity component manager.
  public: void Find(EntityComponentManager &_ecm);

  /// \brief Note the links created this step.
  /// \param[in] _ecm The entity component manager.
  public: void NoteNew(const EntityComponentManager &_ecm);

  /// \brief Forget every link; the next Find scans them all again.
  public: void Reset();

  /// \brief Push on the part of every marked shape on one side of the
  /// water: quadratic drag, 0.5 * rho * Cd * A * |v| * v per shape axis, on
  /// the fluid's velocity relative to the centre of that part, at that
  /// centre. The water level cuts the areas a horizontal flow sees; the plan
  /// area is left whole.
  /// \param[in] _info Update info.
  /// \param[in] _ecm The entity component manager.
  /// \param[in] _side The side the fluid is on.
  /// \param[in] _waterLevel World z of the water.
  /// \param[in] _density Density of the fluid, kg/m^3.
  /// \param[in] _fluid The fluid's velocity at a point.
  public: void ApplyDrag(const UpdateInfo &_info,
                         EntityComponentManager &_ecm, Side _side,
                         double _waterLevel, double _density,
                         const FluidVelocity &_fluid);

  /// \brief The marked shapes, by link.
  /// \return The shapes.
  public: const std::unordered_map<Entity, std::vector<Shape>> &Links()
      const;

  /// \brief The attribute a collision is marked with.
  private: std::string mark;

  /// \brief The per shape drag coefficient attribute.
  private: std::string cdMark;

  /// \brief Drag coefficient of shapes that do not set their own.
  private: double defaultCd{1.0};

  /// \brief Marked shapes per link.
  private: std::unordered_map<Entity, std::vector<Shape>> links;

  /// \brief Scan every link on the next Find.
  private: bool rescan{true};

  /// \brief Links created since the last Find.
  private: std::vector<Entity> newLinks;
};
}  // namespace gz::sim::marked_shapes

#endif  // GZ_SIM_MARKED_SHAPES_MARKEDSHAPES_HH_
