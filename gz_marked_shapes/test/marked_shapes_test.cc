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
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <gz/common/Filesystem.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Collision.hh>
#include <sdf/Element.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include "gz/sim/marked_shapes/MarkedShapes.hh"

using namespace gz;
using namespace sim;
using marked_shapes::Cut;
using marked_shapes::CutAreas;
using marked_shapes::MarkedLinks;
using marked_shapes::Resolve;
using marked_shapes::Shape;
using marked_shapes::Side;

namespace
{
/// \brief Half extents of a 1 m cube.
const math::Vector3d kHalf(0.5, 0.5, 0.5);

/// \brief The mark the tests use.
const char kMark[] = "gz:test";

/// \brief The drag coefficient attribute the tests use.
const char kCdMark[] = "gz:test_cd";

/// \brief A link of one collision, the way the server makes it.
class OneCollision
{
  /// \brief Constructor.
  /// \param[in] _collision The <collision> element, attributes included.
  public: explicit OneCollision(const std::string &_collision)
  {
    const std::string sdf =
        "<?xml version='1.0'?>"
        "<sdf version='1.9' xmlns:gz='http://gazebosim.org/schema'>"
        "<model name='m'><link name='link'>" + _collision +
        "</link></model></sdf>";
    sdf::Errors errors = this->root.LoadSdfString(sdf);
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::Collision &coll =
        *this->root.Model()->LinkByIndex(0)->CollisionByIndex(0);

    this->link = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->link, components::Link());
    this->ecm.CreateComponent(this->link, components::Pose());
    this->ecm.CreateComponent(this->link,
        components::Inertial(math::Inertiald()));

    this->collision = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->collision, components::Collision());
    this->ecm.CreateComponent(this->collision,
        components::CollisionElement(coll));
    this->ecm.CreateComponent(this->collision,
        components::Pose(coll.RawPose()));
    this->ecm.CreateComponent(this->collision,
        components::ParentEntity(this->link));
  }

  /// \brief Resolve the collision.
  /// \param[out] _shape The shape.
  /// \return True if it is a marked shape.
  public: bool Resolve(Shape &_shape) const
  {
    return marked_shapes::Resolve(this->ecm, this->collision, kMark, kCdMark,
                                  1.0, _shape);
  }

  /// \brief The loaded SDF, which the collision element points into.
  public: sdf::Root root;

  /// \brief The entity component manager.
  public: EntityComponentManager ecm;

  /// \brief The link.
  public: Entity link{kNullEntity};

  /// \brief The collision.
  public: Entity collision{kNullEntity};
};

/// \brief A marked collision of one geometry.
/// \param[in] _geometry The <geometry> content.
/// \param[in] _pose The <pose>.
/// \return The <collision> element.
std::string Marked(const std::string &_geometry,
                   const std::string &_pose = "0 0 0 0 0 0")
{
  return "<collision name='c' gz:test='true'><pose>" + _pose + "</pose>"
         "<geometry>" + _geometry + "</geometry></collision>";
}

/// \brief Expect two vectors to be equal to within rounding.
/// \param[in] _a One.
/// \param[in] _b The other.
void ExpectNear(const math::Vector3d &_a, const math::Vector3d &_b)
{
  EXPECT_NEAR(_a.X(), _b.X(), 1e-9) << _a << " vs " << _b;
  EXPECT_NEAR(_a.Y(), _b.Y(), 1e-9) << _a << " vs " << _b;
  EXPECT_NEAR(_a.Z(), _b.Z(), 1e-9) << _a << " vs " << _b;
}
}  // namespace

/////////////////////////////////////////////////
/// A cube straddling the water splits in two halves, each centred in its
/// own part.
TEST(Cut, HalfAboveHalfBelow)
{
  const math::Pose3d pose(3, 4, 0, 0, 0, 0);
  const auto above = Cut(pose, kHalf, 0.0, Side::kAbove);
  const auto below = Cut(pose, kHalf, 0.0, Side::kBelow);
  ASSERT_TRUE(above.any);
  ASSERT_TRUE(below.any);
  EXPECT_DOUBLE_EQ(0.5, above.fraction);
  EXPECT_DOUBLE_EQ(0.5, below.fraction);
  EXPECT_EQ(math::Vector3d(3, 4, 0.25), above.centre);
  EXPECT_EQ(math::Vector3d(3, 4, -0.25), below.centre);
}

/////////////////////////////////////////////////
/// A cube clear of the water is all on one side and none on the other.
TEST(Cut, AllOnOneSide)
{
  const math::Pose3d deep(0, 0, -2, 0, 0, 0);
  EXPECT_FALSE(Cut(deep, kHalf, 0.0, Side::kAbove).any);
  const auto wet = Cut(deep, kHalf, 0.0, Side::kBelow);
  ASSERT_TRUE(wet.any);
  EXPECT_DOUBLE_EQ(1.0, wet.fraction);
  EXPECT_EQ(math::Vector3d(0, 0, -2), wet.centre);

  const math::Pose3d high(0, 0, 2, 0, 0, 0);
  EXPECT_FALSE(Cut(high, kHalf, 0.0, Side::kBelow).any);
  EXPECT_DOUBLE_EQ(1.0, Cut(high, kHalf, 0.0, Side::kAbove).fraction);
}

/////////////////////////////////////////////////
/// The water level moves the cut.
TEST(Cut, WaterLevel)
{
  const math::Pose3d pose(0, 0, 1.25, 0, 0, 0);
  const auto below = Cut(pose, kHalf, 1.5, Side::kBelow);
  ASSERT_TRUE(below.any);
  EXPECT_DOUBLE_EQ(0.75, below.fraction);
  EXPECT_DOUBLE_EQ(1.125, below.centre.Z());
}

/////////////////////////////////////////////////
/// A tilted shape is cut by its bounding box in the world: a cube turned
/// 45 degrees about x reaches sqrt(2) / 2 above and below its centre.
TEST(Cut, TiltedShape)
{
  const math::Pose3d pose(0, 0, 0.5, GZ_PI / 4, 0, 0);
  const auto below = Cut(pose, kHalf, 0.0, Side::kBelow);
  ASSERT_TRUE(below.any);
  const double hz = std::sqrt(2.0) / 2.0;
  EXPECT_NEAR((hz - 0.5) / (2 * hz), below.fraction, 1e-12);
}

/////////////////////////////////////////////////
/// Upright, the side faces are cut and the plan area is whole.
TEST(CutAreas, Upright)
{
  ExpectNear(math::Vector3d(1, 2, 6),
             CutAreas(math::Vector3d(2, 4, 6), math::Quaterniond::Identity,
                      0.5));
}

/////////////////////////////////////////////////
/// On its side the shape's z face is cut and the axis now vertical keeps
/// its face whole, whichever way it was turned.
TEST(CutAreas, OnItsSide)
{
  const math::Vector3d area(2, 4, 6);
  // Pitched 90 degrees: shape x is vertical.
  ExpectNear(math::Vector3d(2, 2, 3),
             CutAreas(area, math::Quaterniond(0, GZ_PI / 2, 0), 0.5));
  // Rolled 90 degrees: shape y is vertical.
  ExpectNear(math::Vector3d(1, 4, 3),
             CutAreas(area, math::Quaterniond(GZ_PI / 2, 0, 0), 0.5));
}

/////////////////////////////////////////////////
/// Between upright and on its side, each face is weighted by how vertical
/// its axis is; all of it kept is all of every face.
TEST(CutAreas, Tilted)
{
  const math::Vector3d area(1, 1, 1);
  const double c = std::cos(GZ_PI / 3);
  const double s = std::sin(GZ_PI / 3);
  ExpectNear(math::Vector3d(0.5 * (1 + s), 0.5, 0.5 * (1 + c)),
             CutAreas(area, math::Quaterniond(0, GZ_PI / 3, 0), 0.5));
  ExpectNear(area, CutAreas(area, math::Quaterniond(0.3, 0.7, 1.1), 1.0));
}

/////////////////////////////////////////////////
/// Each geometry resolves to its bounding box and projected areas.
TEST(Resolve, EveryGeometry)
{
  struct Case
  {
    public: std::string geometry;
    public: math::Vector3d half;
    public: math::Vector3d area;
  };
  const double pi = GZ_PI;
  const Case cases[] = {
    {"<box><size>1 2 3</size></box>", {0.5, 1, 1.5}, {6, 3, 2}},
    {"<cylinder><radius>0.5</radius><length>2</length></cylinder>",
     {0.5, 0.5, 1}, {2, 2, pi / 4}},
    {"<sphere><radius>2</radius></sphere>", {2, 2, 2},
     {4 * pi, 4 * pi, 4 * pi}},
    {"<capsule><radius>0.5</radius><length>2</length></capsule>",
     {0.5, 0.5, 1.5}, {2 + pi / 4, 2 + pi / 4, pi / 4}},
    {"<ellipsoid><radii>1 2 3</radii></ellipsoid>", {1, 2, 3},
     {6 * pi, 3 * pi, 2 * pi}},
  };
  for (const auto &c : cases)
  {
    SCOPED_TRACE(c.geometry);
    OneCollision one(Marked(c.geometry, "1 2 3 0 0 0"));
    Shape shape;
    ASSERT_TRUE(one.Resolve(shape));
    ExpectNear(c.half, shape.half);
    ExpectNear(c.area, shape.area);
    EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), shape.pose);
    EXPECT_DOUBLE_EQ(1.0, shape.cd);
  }
}

/////////////////////////////////////////////////
/// A mesh is its scaled bounding box, centred where the box is, not at the
/// mesh origin.
TEST(Resolve, MeshIsItsBoundingBox)
{
  const std::string uri =
      common::joinPaths(TEST_MEDIA_DIR, "offset_box.obj");
  // Yawed 90 degrees, so the box's centre offset turns with the shape.
  OneCollision one(Marked("<mesh><uri>" + uri + "</uri>"
                          "<scale>1 1 2</scale></mesh>",
                          "1 0 0 0 0 1.5707963267948966"));
  Shape shape;
  ASSERT_TRUE(one.Resolve(shape));
  ExpectNear(math::Vector3d(1, 0.5, 1), shape.half);
  ExpectNear(math::Vector3d(2, 4, 2), shape.area);
  ExpectNear(math::Vector3d(1 - 0.5, 1, 1), shape.pose.Pos());
}

/////////////////////////////////////////////////
/// The mark is read as a bool; without it, or false, there is no shape.
TEST(Resolve, OnlyMarkedCollisions)
{
  Shape shape;
  EXPECT_TRUE(OneCollision("<collision name='c' gz:test='1'><geometry>"
      "<sphere><radius>1</radius></sphere></geometry></collision>")
      .Resolve(shape));
  EXPECT_FALSE(OneCollision("<collision name='c' gz:test='false'><geometry>"
      "<sphere><radius>1</radius></sphere></geometry></collision>")
      .Resolve(shape));
  EXPECT_FALSE(OneCollision("<collision name='c'><geometry>"
      "<sphere><radius>1</radius></sphere></geometry></collision>")
      .Resolve(shape));
}

/////////////////////////////////////////////////
/// The drag coefficient attribute sets the shape's; an invalid one keeps
/// the default.
TEST(Resolve, DragCoefficient)
{
  const std::string box = "<geometry><box><size>1 1 1</size></box>"
                          "</geometry></collision>";
  Shape shape;
  ASSERT_TRUE(OneCollision("<collision name='c' gz:test='true' "
      "gz:test_cd='2.5'>" + box).Resolve(shape));
  EXPECT_DOUBLE_EQ(2.5, shape.cd);
  ASSERT_TRUE(OneCollision("<collision name='c' gz:test='true' "
      "gz:test_cd='-1'>" + box).Resolve(shape));
  EXPECT_DOUBLE_EQ(1.0, shape.cd);
  ASSERT_TRUE(OneCollision("<collision name='c' gz:test='true' "
      "gz:test_cd='lots'>" + box).Resolve(shape));
  EXPECT_DOUBLE_EQ(1.0, shape.cd);
}

/////////////////////////////////////////////////
/// The density and default drag coefficient come from the system's SDF;
/// an invalid value keeps the default.
TEST(MarkedLinks, Configure)
{
  // A system gets its <plugin> element, with whatever it holds.
  auto sdfOf = [](const std::string &_body)
  {
    sdf::Root root;
    EXPECT_TRUE(root.LoadSdfString(
        "<?xml version='1.0'?><sdf version='1.9'><model name='m'>"
        "<link name='l'/><plugin name='p' filename='f'>" + _body +
        "</plugin></model></sdf>").empty());
    return std::const_pointer_cast<const sdf::Element>(
        root.Model()->Element()->GetElement("plugin")->Clone());
  };
  const std::string marked = Marked("<box><size>1 1 1</size></box>");

  MarkedLinks links(kMark, kCdMark);
  links.Configure(sdfOf("<rho>2</rho><default_drag_coefficient>3"
                        "</default_drag_coefficient>"), "Test", "rho",
                  7.0);
  EXPECT_DOUBLE_EQ(2.0, links.Density());
  OneCollision one(marked);
  links.Find(one.ecm);
  ASSERT_EQ(1u, links.Links().count(one.link));
  EXPECT_DOUBLE_EQ(3.0, links.Links().at(one.link).front().cd);

  MarkedLinks invalid(kMark, kCdMark);
  invalid.Configure(sdfOf("<rho>-2</rho><default_drag_coefficient>-3"
                          "</default_drag_coefficient>"), "Test", "rho",
                    7.0);
  EXPECT_DOUBLE_EQ(7.0, invalid.Density());
  OneCollision other(marked);
  invalid.Find(other.ecm);
  ASSERT_EQ(1u, invalid.Links().count(other.link));
  EXPECT_DOUBLE_EQ(1.0, invalid.Links().at(other.link).front().cd);
}

/////////////////////////////////////////////////
/// A marked link removed from the world is forgotten.
TEST(MarkedLinks, RemovedLinkIsForgotten)
{
  OneCollision one(Marked("<box><size>1 1 1</size></box>"));
  MarkedLinks links(kMark, kCdMark);
  links.Find(one.ecm);
  ASSERT_EQ(1u, links.Links().size());

  one.ecm.RequestRemoveEntity(one.link);
  one.ecm.ProcessRemoveEntityRequests();
  bool asked{false};
  links.ApplyDrag(UpdateInfo(), one.ecm, Side::kBelow, 0.0,
      [&asked](const math::Vector3d &,
               const std::chrono::steady_clock::duration &)
      {
        asked = true;
        return math::Vector3d::Zero;
      });
  EXPECT_TRUE(links.Links().empty());
  EXPECT_FALSE(asked);
}
