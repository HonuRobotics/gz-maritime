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
// The maritime GUI's core, without a GUI: the recipes read back, the
// messages it sends, the arrows it draws, and a refused change told from an
// applied one.

#include <gtest/gtest.h>

#include <chrono>
#include <clocale>
#include <map>
#include <string>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/OceanCurrentfield.hh>
#include <gz/sim/components/Windfield.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/maritime_gui/FieldLink.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"
#include "gz/sim/wind/WindSampler.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;

namespace
{
/// \brief An ECM with a world entity, and the recipes asked for.
/// \param[in] _ecm The ECM to fill.
/// \param[in] _current Add an ocean current recipe.
/// \param[in] _wind Add a wind recipe.
/// \param[in] _extra Give the current a parameter of its model's own,
/// which the standard model refuses.
void World(EntityComponentManager &_ecm, bool _current, bool _wind,
           bool _extra = false)
{
  const Entity world = _ecm.CreateEntity();
  _ecm.CreateComponent(world, components::World());
  _ecm.CreateComponent(world, components::Name("default"));
  if (_current)
  {
    ocean_current::OceanCurrentfieldData d;
    d.generation = 3;
    d.params.speed = 0.5;
    d.params.direction = 90.0;
    d.params.water_level = -0.25;
    if (_extra)
      d.params.extra["source"] = "grid.nc";
    _ecm.CreateComponent(world, components::OceanCurrentfield(d));
  }
  if (_wind)
  {
    wind::WindfieldData d;
    d.generation = 7;
    d.params.speed = 10.0;
    d.params.direction = 270.0;
    d.params.speed_gust = 1.5;
    d.params.reference_height = 10.0;
    _ecm.CreateComponent(world, components::Windfield(d));
  }
}
}  // namespace

/////////////////////////////////////////////////
/// The panel shows the recipes' values, and nothing for a world without a
/// field.
TEST(Fields, ReadTheRecipes)
{
  EntityComponentManager ecm;
  World(ecm, true, true, true);
  const auto current = ReadCurrent(ecm);
  ASSERT_TRUE(current.has_value());
  EXPECT_EQ(3u, current->generation);
  EXPECT_EQ("standard", current->model);
  EXPECT_DOUBLE_EQ(0.5, current->speed);
  EXPECT_DOUBLE_EQ(90.0, current->direction);
  EXPECT_DOUBLE_EQ(-0.25, current->waterLevel);
  EXPECT_EQ("grid.nc", current->extra.at("source"));

  const auto wind = ReadWind(ecm);
  ASSERT_TRUE(wind.has_value());
  EXPECT_EQ(7u, wind->generation);
  EXPECT_DOUBLE_EQ(10.0, wind->speed);
  EXPECT_DOUBLE_EQ(270.0, wind->direction);
  EXPECT_DOUBLE_EQ(1.5, wind->speedGust);
  EXPECT_DOUBLE_EQ(10.0, wind->referenceHeight);

  EntityComponentManager slack;
  World(slack, false, false);
  EXPECT_FALSE(ReadCurrent(slack).has_value());
  EXPECT_FALSE(ReadWind(slack).has_value());
  EXPECT_FALSE(ReadCurrent(EntityComponentManager()).has_value())
      << "no world entity at all";
}

/////////////////////////////////////////////////
/// A message carries only the keys the user changed, as doubles.
TEST(Fields, OnlyChangedKeysAreSent)
{
  CurrentValues current;
  current.speed = 0.5;
  current.direction = 90.0;
  const auto recipe = Editable(current);
  EXPECT_EQ(3u, recipe.size());

  const auto none = ChangedKeys(recipe, recipe);
  EXPECT_TRUE(none.params().empty()) << "nothing changed, nothing sent";

  auto edits = recipe;
  edits["direction"] = 0.0;
  const auto msg = ChangedKeys(edits, recipe);
  ASSERT_EQ(1, msg.params().size());
  const auto &any = msg.params().at("direction");
  EXPECT_EQ(msgs::Any::DOUBLE, any.type());
  EXPECT_DOUBLE_EQ(0.0, any.double_value());

  WindValues wind;
  EXPECT_EQ(6u, Editable(wind).size());
  EXPECT_EQ(1u, Editable(wind).count("direction_gust_time"));
}

/////////////////////////////////////////////////
/// The compass draws the flow: a wind from 270 and a current setting 090
/// both go east.
TEST(Fields, FlowHeadingKeepsEachConvention)
{
  EXPECT_DOUBLE_EQ(90.0, FlowHeading(90.0, false));
  EXPECT_DOUBLE_EQ(90.0, FlowHeading(270.0, true));
  EXPECT_DOUBLE_EQ(225.0, FlowHeading(45.0, true));
  EXPECT_DOUBLE_EQ(180.0, FlowHeading(0.0, true));
  EXPECT_DOUBLE_EQ(350.0, FlowHeading(-10.0, false));
  EXPECT_DOUBLE_EQ(0.0, FlowHeading(360.0, false));
}

/////////////////////////////////////////////////
/// The arrows are the fields' own samplers: a wind from 270 and a current
/// setting 090 both draw arrows towards +x.
TEST(Fields, ArrowsFollowTheSamplers)
{
  EntityComponentManager ecm;
  World(ecm, true, true);

  ocean_current::OceanCurrentSampler current;
  current.Sync(ecm);
  ASSERT_TRUE(current.Valid());
  wind::WindSampler wind;
  wind.Sync(ecm);
  ASSERT_TRUE(wind.Valid());

  const std::chrono::steady_clock::duration t{0};
  const std::vector<math::Vector3d> point{{0.0, 0.0, 10.0}};
  for (const auto &velocity :
       {std::function<math::Vector3d(const math::Vector3d &)>(
            [&](const math::Vector3d &_p) { return current.At(_p, t); }),
        std::function<math::Vector3d(const math::Vector3d &)>(
            [&](const math::Vector3d &_p) { return wind.At(_p, t); })})
  {
    const auto segments = ArrowSegments(point, velocity, 1.0);
    ASSERT_EQ(6u, segments.size());
    const auto shaft = segments[1] - segments[0];
    EXPECT_GT(shaft.X(), 0.0);
    EXPECT_NEAR(0.0, shaft.Y(), 1e-6);
  }
}

/////////////////////////////////////////////////
/// An arrow is a shaft scaled by the speed and a head lying flat behind its
/// tip; a point with no velocity draws nothing.
TEST(Fields, ArrowShape)
{
  const std::vector<math::Vector3d> points{{1.0, 2.0, 0.1}, {5.0, 5.0, 0.1}};
  const auto segments = ArrowSegments(points,
      [](const math::Vector3d &_p)
      {
        return _p.X() < 2.0 ? math::Vector3d(0.0, 0.5, 0.0) :
            math::Vector3d::Zero;
      }, 4.0);
  ASSERT_EQ(6u, segments.size()) << "the still point draws nothing";
  EXPECT_EQ(math::Vector3d(1.0, 2.0, 0.1), segments[0]);
  EXPECT_EQ(math::Vector3d(1.0, 4.0, 0.1), segments[1]) << "2 m: 0.5 m/s x 4";
  for (std::size_t i : {3u, 5u})
  {
    EXPECT_LT(segments[i].Y(), segments[1].Y()) << "the head is behind";
    EXPECT_NEAR(0.1, segments[i].Z(), 1e-9) << "and flat on the water";
  }
  EXPECT_NEAR(segments[3].X() - 1.0, 1.0 - segments[5].X(), 1e-9)
      << "symmetric";

  const auto up = ArrowSegments({math::Vector3d::Zero},
      [](const math::Vector3d &) { return math::Vector3d(0, 0, 1); }, 1.0);
  ASSERT_EQ(6u, up.size()) << "a vertical arrow has a head too";
  EXPECT_NE(up[1], up[3]);
}

/////////////////////////////////////////////////
/// A grid is centred, spaced, and capped.
TEST(Fields, GridPoints)
{
  Grid grid;
  grid.center.Set(10.0, -10.0);
  grid.extent = 10.0;
  grid.spacing = 5.0;
  const auto points = GridPoints(grid, 2.0);
  ASSERT_EQ(9u, points.size());
  EXPECT_EQ(math::Vector3d(5.0, -15.0, 2.0), points.front());
  EXPECT_EQ(math::Vector3d(15.0, -5.0, 2.0), points.back());
  EXPECT_EQ(math::Vector3d(10.0, -10.0, 2.0), points[4]);

  grid.extent = 1000.0;
  grid.spacing = 0.1;
  EXPECT_EQ(static_cast<std::size_t>(kMaxArrowsPerSide * kMaxArrowsPerSide),
            GridPoints(grid, 0.0).size());

  grid.spacing = 0.0;
  EXPECT_TRUE(GridPoints(grid, 0.0).empty());
  grid.extent = 0.0;
  grid.spacing = 5.0;
  EXPECT_EQ(1u, GridPoints(grid, 0.0).size()) << "one arrow at the centre";
}

/////////////////////////////////////////////////
/// The arrows are one LINE_LIST marker, and a delete removes it.
TEST(Fields, Markers)
{
  const std::vector<math::Vector3d> segments{{0, 0, 0}, {1, 0, 0}};
  const auto marker = ArrowMarker("ns", 4, math::Color::Blue, segments);
  EXPECT_EQ("ns", marker.ns());
  EXPECT_EQ(4u, marker.id());
  EXPECT_EQ(msgs::Marker::ADD_MODIFY, marker.action());
  EXPECT_EQ(msgs::Marker::LINE_LIST, marker.type());
  EXPECT_EQ(2, marker.point_size());
  EXPECT_FLOAT_EQ(1.0f, marker.material().diffuse().b());

  const auto del = DeleteMarker("ns", 4);
  EXPECT_EQ(msgs::Marker::DELETE_MARKER, del.action());
  EXPECT_EQ("ns", del.ns());
  EXPECT_EQ(4u, del.id());
}

/////////////////////////////////////////////////
/// A change shows as a new generation; none within the timeout is a
/// refusal.
TEST(Fields, ChangeTracker)
{
  using Clock = ChangeTracker::Clock;
  const auto t0 = Clock::now();
  ChangeTracker tracker(std::chrono::seconds(2));
  EXPECT_EQ(ChangeTracker::State::kIdle, tracker.Update(5, t0));

  tracker.Sent(5, t0);
  EXPECT_EQ(ChangeTracker::State::kPending,
            tracker.Update(5, t0 + std::chrono::seconds(1)));
  EXPECT_EQ(ChangeTracker::State::kApplied,
            tracker.Update(6, t0 + std::chrono::seconds(1)));
  EXPECT_EQ(ChangeTracker::State::kApplied,
            tracker.Update(6, t0 + std::chrono::seconds(9)))
      << "it stays applied";

  tracker.Sent(6, t0);
  EXPECT_EQ(ChangeTracker::State::kRefused,
            tracker.Update(6, t0 + std::chrono::seconds(2)));
  EXPECT_EQ(ChangeTracker::State::kRefused,
            tracker.Update(7, t0 + std::chrono::seconds(3)))
      << "a later change by someone else does not undo the refusal";

  tracker.NotSent();
  EXPECT_EQ(ChangeTracker::State::kUnheard,
            tracker.Update(8, t0 + std::chrono::seconds(4)))
      << "a message nobody heard stays unheard";
}

/////////////////////////////////////////////////
/// A number takes a decimal point or a decimal comma, whatever the locale,
/// and nothing else.
TEST(Fields, ParseNumber)
{
  double v{-7.0};
  EXPECT_TRUE(ParseNumber("1.5", v));
  EXPECT_DOUBLE_EQ(1.5, v);
  EXPECT_TRUE(ParseNumber("1,5", v));
  EXPECT_DOUBLE_EQ(1.5, v);
  EXPECT_TRUE(ParseNumber(" -0.25 ", v));
  EXPECT_DOUBLE_EQ(-0.25, v);
  EXPECT_TRUE(ParseNumber(".5", v));
  EXPECT_DOUBLE_EQ(0.5, v);
  EXPECT_TRUE(ParseNumber("270", v));
  EXPECT_DOUBLE_EQ(270.0, v);

  v = -7.0;
  for (const char *text : {"", " ", "-", "1.5.2", "1,5,2", "abc", "1.5m",
                           "nan", "inf"})
  {
    EXPECT_FALSE(ParseNumber(text, v)) << "[" << text << "]";
  }
  EXPECT_DOUBLE_EQ(-7.0, v) << "a refused text leaves the value alone";

  // Qt sets the process's locale from the environment; a locale with a
  // decimal comma must not make the point unreadable.
  const char *old = std::setlocale(LC_NUMERIC, nullptr);
  const std::string saved = old ? old : "C";
  if (std::setlocale(LC_NUMERIC, "es_ES.UTF-8") ||
      std::setlocale(LC_NUMERIC, "de_DE.UTF-8"))
  {
    EXPECT_TRUE(ParseNumber("2.75", v));
    EXPECT_DOUBLE_EQ(2.75, v);
    EXPECT_TRUE(ParseNumber("2,75", v));
    EXPECT_DOUBLE_EQ(2.75, v);
  }
  std::setlocale(LC_NUMERIC, saved.c_str());
}

/////////////////////////////////////////////////
/// The status line says what became of a change.
TEST(Fields, StatusText)
{
  EXPECT_EQ("", StatusText(ChangeTracker::State::kIdle));
  EXPECT_EQ("Applied.", StatusText(ChangeTracker::State::kApplied));
  EXPECT_EQ(0u, StatusText(ChangeTracker::State::kRefused).find("Not applied"));
  EXPECT_EQ(0u, StatusText(ChangeTracker::State::kUnheard).find("Not sent"));
}
