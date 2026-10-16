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

#include <tinyxml2.h>

#include <chrono>
#include <clocale>
#include <map>
#include <optional>
#include <string>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/OceanCurrentfield.hh>
#include <gz/sim/components/Windfield.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/maritime_gui/FieldControl.hh"
#include "gz/sim/maritime_gui/FieldSpec.hh"
#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/maritime_gui/Flow.hh"
#include "gz/sim/maritime_gui/Presets.hh"
#include "gz/sim/maritime_gui/Vessels.hh"
#include "gz/sim/maritime_gui/WindSock.hh"
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

/////////////////////////////////////////////////
/// A colour map blends its stops from still to its maximum, and holds the
/// last one beyond.
TEST(Flow, Colormap)
{
  const Colormap map({math::Color::Black, math::Color::White}, 2.0);
  EXPECT_EQ(math::Color::Black, map.At(0.0));
  EXPECT_EQ(math::Color::Black, map.At(-1.0));
  EXPECT_EQ(math::Color::White, map.At(2.0));
  EXPECT_EQ(math::Color::White, map.At(9.0));
  EXPECT_NEAR(0.5, map.At(1.0).R(), 1e-6);

  auto water = Colormap::Water(1.0);
  EXPECT_LT(water.At(0.0).R() + water.At(0.0).G(),
            water.At(1.0).R() + water.At(1.0).G()) << "faster is brighter";
  water.SetMaxSpeed(-1.0);
  EXPECT_DOUBLE_EQ(1.0, water.MaxSpeed()) << "a bad maximum is ignored";
  EXPECT_EQ(5u, Colormap::Air(20.0).Stops().size());
}

/////////////////////////////////////////////////
/// Segments go to one bin per colour, one unlit LINE_LIST each.
TEST(Flow, ColorBins)
{
  ColorBins bins(Colormap({math::Color::Black, math::Color::White}, 1.0),
                 2, 2);
  EXPECT_EQ(4, bins.Count());
  bins.Add({0, 0, 0}, {1, 0, 0}, 0.0, 1.0);   // slow, bright
  bins.Add({0, 0, 0}, {0, 1, 0}, 1.0, 1.0);   // fast, bright
  bins.Add({0, 0, 0}, {0, 0, 1}, 1.0, 0.0);   // fast, dim
  const auto markers = bins.Markers("ns", 5);
  ASSERT_EQ(4u, markers.size());
  EXPECT_EQ(5u, markers[0].id());
  EXPECT_EQ(8u, markers[3].id());
  EXPECT_EQ(0, markers[0].point_size()) << "slow and dim: empty";
  EXPECT_EQ(2, markers[1].point_size());
  EXPECT_EQ(2, markers[2].point_size());
  EXPECT_EQ(2, markers[3].point_size());
  for (const auto &m : markers)
  {
    EXPECT_EQ(msgs::Marker::LINE_LIST, m.type());
    EXPECT_FALSE(m.material().lighting());
  }
  EXPECT_GT(bins.BinColor(3).R(), bins.BinColor(2).R()) << "brighter bin";
  EXPECT_GT(bins.BinColor(3).R(), bins.BinColor(1).R()) << "faster bin";
}

/////////////////////////////////////////////////
/// Arrows coloured by speed land in the bin of their speed.
TEST(Flow, ColoredArrows)
{
  const auto bins = ColoredArrows({{0, 0, 0}},
      [](const math::Vector3d &) { return math::Vector3d(1, 0, 0); }, 1.0,
      Colormap({math::Color::Black, math::Color::White}, 1.0));
  const auto markers = bins.Markers("ns", 1);
  int points{0};
  for (const auto &m : markers)
    points += m.point_size();
  EXPECT_EQ(6, points) << "a shaft and two head strokes";
  EXPECT_EQ(6, markers.back().point_size()) << "full speed, last bin";
}

/////////////////////////////////////////////////
/// Particles drift with the field at its velocity times the speed-up, stay
/// on their plane, and start again inside the region when they leave it.
TEST(Flow, ParticlesDriftWithTheField)
{
  ParticleField field(7u);
  ParticleField::Settings s;
  s.region.extent = 20.0;
  s.z = 3.0;
  s.count = 50;
  s.lifetime = 100.0;
  s.speedup = 2.0;
  field.Configure(s);
  const auto before = field.Positions();
  ASSERT_EQ(50u, before.size());
  for (const auto &p : before)
  {
    EXPECT_LE(std::abs(p.X()), 10.0);
    EXPECT_LE(std::abs(p.Y()), 10.0);
    EXPECT_DOUBLE_EQ(3.0, p.Z());
  }

  const auto east = [](const math::Vector3d &)
      { return math::Vector3d(0.5, 0.0, 0.3); };
  field.Step(0.1, east);
  const auto after = field.Positions();
  int moved{0};
  for (std::size_t i = 0; i < after.size(); ++i)
  {
    // A particle that left the region started again elsewhere.
    if (std::abs(after[i].X() - before[i].X() - 0.1) < 1e-9)
    {
      ++moved;
      EXPECT_DOUBLE_EQ(before[i].Y(), after[i].Y());
      EXPECT_DOUBLE_EQ(3.0, after[i].Z()) << "horizontal only";
    }
  }
  EXPECT_GT(moved, 40) << "0.5 m/s x 2 x 0.1 s = 0.1 m east";

  for (int i = 0; i < 400; ++i)
    field.Step(0.1, east);
  for (const auto &p : field.Positions())
  {
    EXPECT_LE(std::abs(p.X()), 10.0) << "still in the region";
    EXPECT_LE(std::abs(p.Y()), 10.0);
  }

  // Nothing moves without time.
  const auto frozen = field.Positions();
  field.Step(0.0, east);
  EXPECT_EQ(frozen, field.Positions());
}

/////////////////////////////////////////////////
/// The trails fade out towards the edge of the region's disc: none drawn
/// outside it.
TEST(Flow, TrailsFadeOutAtTheEdge)
{
  ParticleField field(3u);
  ParticleField::Settings s;
  s.region.extent = 20.0;
  s.count = 300;
  s.lifetime = 100.0;
  s.trailStep = 0.05;
  field.Configure(s);
  const auto east = [](const math::Vector3d &)
      { return math::Vector3d(1.0, 0.0, 0.0); };
  for (int i = 0; i < 20; ++i)
    field.Step(0.05, east);
  const auto markers = field.Trails(Colormap::Water(1.0)).Markers("ns", 1);
  int points{0};
  for (const auto &m : markers)
  {
    for (const auto &p : m.point())
    {
      ++points;
      EXPECT_LE(std::hypot(p.x(), p.y()), 10.0 + 1.5)
          << "within the disc, give or take a segment";
    }
  }
  EXPECT_GT(points, 100);
}

/////////////////////////////////////////////////
/// Steering ramps from the world's values to the target no faster than its
/// rates, the short way round, one step per period, and stops there.
TEST(Flow, SteeringRamps)
{
  using Clock = Steering::Clock;
  Steering steer(1.0, 90.0, std::chrono::milliseconds(100));
  auto t = Clock::now();
  EXPECT_FALSE(steer.Step(0.0, 0.0, t).has_value()) << "no target";

  steer.SetTarget(0.5, 350.0);
  ASSERT_TRUE(steer.Active());
  auto msg = steer.Step(0.0, 10.0, t);
  ASSERT_TRUE(msg.has_value());
  EXPECT_NEAR(0.1, msg->params().at("speed").double_value(), 1e-9);
  EXPECT_NEAR(1.0, msg->params().at("direction").double_value(), 1e-9)
      << "10 to 350 turns 20 degrees west, through north";
  EXPECT_FALSE(steer.Step(0.0, 10.0, t + std::chrono::milliseconds(50))
               .has_value()) << "one step per period";

  int steps{1};
  while (steer.Active() && steps < 100)
  {
    t += std::chrono::milliseconds(100);
    msg = steer.Step(0.0, 10.0, t);
    if (msg)
      ++steps;
  }
  EXPECT_FALSE(steer.Active());
  EXPECT_TRUE(steer.Arrived());
  EXPECT_EQ(5, steps) << "0.5 m/s at 1 m/s per second, in 0.1 s steps";
  EXPECT_NEAR(0.5, msg->params().at("speed").double_value(), 1e-9);
  EXPECT_NEAR(350.0, msg->params().at("direction").double_value(), 1e-9);

  // A new target after arriving starts again from the world's values.
  steer.SetTarget(1.0, 350.0);
  msg = steer.Step(0.9, 350.0, t + std::chrono::milliseconds(100));
  ASSERT_TRUE(msg.has_value());
  EXPECT_NEAR(1.0, msg->params().at("speed").double_value(), 1e-9);
  EXPECT_TRUE(steer.Arrived());
}

/////////////////////////////////////////////////
/// Another controller moving the field after one of ours landed stops the
/// steering, and it stays stopped until the next target.
TEST(Flow, SteeringYieldsToAnotherController)
{
  using Clock = Steering::Clock;
  using std::chrono::milliseconds;
  Steering steer(1.0, 90.0, milliseconds(100));
  auto t = Clock::now();
  steer.SetTarget(1.0, 0.0);
  auto msg = steer.Step(0.0, 0.0, t);
  ASSERT_TRUE(msg.has_value());
  const double first = msg->params().at("speed").double_value();

  // The recipe still shows the world's value, then ours: carry on.
  t += milliseconds(100);
  ASSERT_TRUE(steer.Step(0.0, 0.0, t).has_value());
  t += milliseconds(100);
  ASSERT_TRUE(steer.Step(first, 0.0, t).has_value());
  EXPECT_FALSE(steer.Yielded());

  // Someone sets 0.7 m/s from 45 degrees: stop, and send nothing more.
  t += milliseconds(100);
  EXPECT_FALSE(steer.Step(0.7, 45.0, t).has_value());
  EXPECT_FALSE(steer.Active());
  EXPECT_TRUE(steer.Yielded());
  EXPECT_FALSE(steer.Arrived());
  t += milliseconds(100);
  EXPECT_FALSE(steer.Step(0.7, 45.0, t).has_value());

  // A new target steers again, from the world's values.
  steer.SetTarget(0.7, 50.0);
  EXPECT_FALSE(steer.Yielded());
  t += milliseconds(100);
  msg = steer.Step(0.7, 45.0, t);
  ASSERT_TRUE(msg.has_value());
  EXPECT_NEAR(50.0, msg->params().at("direction").double_value(), 1e-9);
}

/////////////////////////////////////////////////
/// Taking over a field another controller is still ramping: its values
/// show until ours land, and do not stop us; then it is the other one that
/// sees a value it did not send. The last to start wins.
TEST(Flow, LastControllerToStartWins)
{
  using Clock = Steering::Clock;
  using std::chrono::milliseconds;
  Steering older(1.0, 90.0, milliseconds(100));
  Steering newer(1.0, 90.0, milliseconds(100));
  auto t = Clock::now();

  // The recipe the world holds, as the last message to arrive set it.
  double speed{0.0};
  double direction{0.0};
  auto apply = [&](const std::optional<msgs::Param> &_msg)
  {
    if (!_msg)
      return;
    speed = _msg->params().at("speed").double_value();
    direction = _msg->params().at("direction").double_value();
  };

  older.SetTarget(1.0, 0.0);
  apply(older.Step(speed, direction, t));
  t += milliseconds(100);
  apply(older.Step(speed, direction, t));

  // The newer one starts while the older is half way, and both step until
  // one of them gives way.
  newer.SetTarget(0.0, 180.0);
  for (int i = 0; i < 40 && (older.Active() || newer.Active()); ++i)
  {
    t += milliseconds(100);
    const auto fromOlder = older.Step(speed, direction, t);
    const auto fromNewer = newer.Step(speed, direction, t);
    apply(fromOlder);
    apply(fromNewer);
  }
  EXPECT_TRUE(older.Yielded());
  EXPECT_FALSE(newer.Yielded());
  EXPECT_TRUE(newer.Arrived());
  EXPECT_NEAR(0.0, speed, 1e-9);
  EXPECT_NEAR(180.0, direction, 1e-9);
}

/////////////////////////////////////////////////
/// Each field's rates, colours and scales are defined once.
TEST(Flow, FieldSpecs)
{
  const auto current = OceanCurrentSpec();
  EXPECT_EQ("ocean_current", current.name);
  EXPECT_DOUBLE_EQ(0.5, current.speedRate);
  EXPECT_DOUBLE_EQ(1.0, current.colormap.MaxSpeed());
  EXPECT_DOUBLE_EQ(4.0, current.scale);
  EXPECT_DOUBLE_EQ(8.0, current.trails.speedup);
  const auto wind = WindSpec();
  EXPECT_EQ("wind", wind.name);
  EXPECT_DOUBLE_EQ(5.0, wind.speedRate);
  EXPECT_DOUBLE_EQ(20.0, wind.colormap.MaxSpeed());
  EXPECT_DOUBLE_EQ(0.3, wind.scale);
  EXPECT_DOUBLE_EQ(1.0, wind.trails.speedup);

  // The steering they make runs at their rates: 0.5 m/s a second is 0.05
  // m/s in the first 0.1 s step.
  auto steer = current.MakeSteering();
  steer.SetTarget(1.0, 0.0);
  const auto msg = steer.Step(0.0, 0.0, Steering::Clock::now());
  ASSERT_TRUE(msg.has_value());
  EXPECT_NEAR(0.05, msg->params().at("speed").double_value(), 1e-9);
}

namespace
{
/// \brief Add a model with a link on the world.
/// \param[in,out] _ecm The ECM.
/// \param[in] _world The world entity.
/// \param[in] _name The model's name.
/// \param[in] _pose Its pose.
/// \param[in] _static Whether it is static.
/// \return The model.
Entity AddModel(EntityComponentManager &_ecm, Entity _world,
                const std::string &_name, const math::Pose3d &_pose,
                bool _static)
{
  const Entity model = _ecm.CreateEntity();
  _ecm.CreateComponent(model, components::Model());
  _ecm.CreateComponent(model, components::Name(_name));
  _ecm.CreateComponent(model, components::ParentEntity(_world));
  _ecm.CreateComponent(model, components::Pose(_pose));
  _ecm.CreateComponent(model, components::Static(_static));
  const Entity link = _ecm.CreateEntity();
  _ecm.CreateComponent(link, components::Link());
  _ecm.CreateComponent(link, components::ParentEntity(model));
  _ecm.CreateComponent(link, components::Pose(math::Pose3d::Zero));
  return model;
}
}  // namespace

/////////////////////////////////////////////////
/// The vehicles are the models that move: their velocity comes from their
/// poses over simulation time, and they leave a track.
TEST(Vessels, TrackerFollowsMovingModels)
{
  EntityComponentManager ecm;
  const Entity world = ecm.CreateEntity();
  ecm.CreateComponent(world, components::World());
  ecm.CreateComponent(world, components::Name("default"));
  const Entity boat = AddModel(ecm, world, "boat", math::Pose3d::Zero, false);
  AddModel(ecm, world, "pad", math::Pose3d(8, -8, 1, 0, 0, 0), true);

  VesselTracker tracker(10.0, 0.5);
  using std::chrono::milliseconds;
  for (int i = 0; i <= 20; ++i)
  {
    // 1 m/s east, 0.5 m/s north.
    const double t = 0.1 * i;
    ecm.SetComponentData<components::Pose>(boat,
        math::Pose3d(t, 0.5 * t, 0, 0, 0, 0));
    tracker.Update(ecm, milliseconds(100 * i));
  }
  const auto vessels = tracker.Vessels();
  ASSERT_EQ(1u, vessels.size()) << "the static pad is not a vehicle";
  EXPECT_EQ("boat", vessels[0].name);
  EXPECT_NEAR(1.0, vessels[0].velocity.X(), 1e-6);
  EXPECT_NEAR(0.5, vessels[0].velocity.Y(), 1e-6);
  EXPECT_EQ(5u, vessels[0].track.size()) << "a point every 0.5 s over 2 s";
  EXPECT_NEAR(2.0, vessels[0].pose.Pos().X(), 1e-9);

  // A removed vehicle is forgotten; a reset clears the track.
  tracker.Update(ecm, milliseconds(0));
  EXPECT_EQ(1u, tracker.Vessels()[0].track.size());
  ecm.RequestRemoveEntity(boat);
  ecm.ProcessRemoveEntityRequests();
  tracker.Update(ecm, milliseconds(100));
  EXPECT_TRUE(tracker.Vessels().empty());
}

/////////////////////////////////////////////////
/// A solid arrow is flat and upright, crossed, from its start to its tip.
TEST(Vessels, FlatArrow)
{
  ColorBins bins(Colormap({math::Color::White}, 1.0), 1, 1,
                 msgs::Marker::TRIANGLE_LIST);
  FlatArrow(bins, {1, 2, 3}, {2, 0, 5}, 0.2, 1.0);
  const auto markers = bins.Markers("ns", 1);
  ASSERT_EQ(1u, markers.size());
  EXPECT_EQ(msgs::Marker::TRIANGLE_LIST, markers[0].type());
  // Two copies of three triangles, each seen from both sides.
  EXPECT_EQ(2 * 3 * 6, markers[0].point_size());
  double maxX{-1e9}, minZ{1e9}, maxZ{-1e9};
  for (const auto &p : markers[0].point())
  {
    maxX = std::max(maxX, p.x());
    minZ = std::min(minZ, p.z());
    maxZ = std::max(maxZ, p.z());
  }
  EXPECT_NEAR(3.0, maxX, 1e-9) << "the tip, 2 m east; z ignored";
  EXPECT_LT(minZ, 3.0) << "the upright copy";
  EXPECT_GT(maxZ, 3.0);

  ColorBins none(Colormap({math::Color::White}, 1.0), 1, 1,
                 msgs::Marker::TRIANGLE_LIST);
  FlatArrow(none, {0, 0, 0}, {0, 0, 1}, 0.2, 1.0);
  EXPECT_EQ(0, none.Markers("ns", 1)[0].point_size()) << "no horizontal";
}

/////////////////////////////////////////////////
/// The relative velocity of each vehicle becomes one arrow; the tracks and
/// headings are lines on a plane.
TEST(Vessels, ArrowsTracksAndHeadings)
{
  Vessel v;
  v.name = "boat";
  v.pose = math::Pose3d(0, 0, 0, 0, 0, GZ_PI / 2);
  v.velocity = {0.2, 0.0, 0.0};
  v.track = {{-2, 0, 0}, {-1, 0, 0}};
  const auto arrows = VesselArrows({v}, [](const Vessel &_v)
      { return math::Vector3d(0.5, 0, 0) - _v.velocity; }, 1.0, 4.0, 0.25,
      Colormap::Water(1.0));
  int points{0};
  for (const auto &m : arrows.Markers("ns", 1))
  {
    for (const auto &p : m.point())
    {
      ++points;
      EXPECT_LE(p.x(), 1.2 + 1e-9) << "0.3 m/s x 4 m per m/s";
    }
  }
  EXPECT_EQ(36, points);

  const auto lines = TracksAndHeadings({v}, 0.1, 3.0).Markers("ns", 1);
  bool heading{false};
  int segments{0};
  for (const auto &m : lines)
  {
    EXPECT_EQ(msgs::Marker::LINE_LIST, m.type());
    for (int i = 0; i + 1 < m.point_size(); i += 2)
    {
      ++segments;
      EXPECT_DOUBLE_EQ(0.1, m.point(i).z());
      if (std::abs(m.point(i + 1).y() - 3.0) < 1e-9)
        heading = true;
    }
  }
  EXPECT_EQ(3, segments) << "one track segment, the join, the heading";
  EXPECT_TRUE(heading) << "yaw 90 degrees points along +y";
}

/////////////////////////////////////////////////
/// A sock points where the wind goes, droops in light air, and flies out
/// straight at its full speed.
TEST(WindSock, Axis)
{
  const math::Vector3d full = SockAxis({10, 0, 0}, math::Vector3d::UnitY);
  EXPECT_NEAR(1.0, full.X(), 1e-9) << "straight out, downwind";
  EXPECT_NEAR(0.0, full.Z(), 1e-9);

  const math::Vector3d half = SockAxis({0, kSockFullSpeed / 2, 0},
                                       math::Vector3d::UnitX);
  EXPECT_GT(half.Y(), 0.0);
  EXPECT_LT(half.Z(), 0.0) << "drooping";
  EXPECT_NEAR(1.0, half.Length(), 1e-9);

  const math::Vector3d still = SockAxis(math::Vector3d::Zero, {-1, 0, 0});
  EXPECT_LT(still.X(), 0.0) << "the way it pointed last";
  EXPECT_NEAR(-std::sin(GZ_DTOR(kSockMaxDroop)), still.Z(), 1e-9);
}

/////////////////////////////////////////////////
/// A sock is a pole and two colours of bands at its top.
TEST(WindSock, Markers)
{
  const auto markers = WindSockMarkers("ns", 4, {1, 2, 3}, 3.0,
                                       {1, 0, 0}, 2.0);
  ASSERT_EQ(3u, markers.size());
  EXPECT_EQ(4u, markers[0].id());
  EXPECT_EQ(6u, markers[2].id());
  for (const auto &m : markers)
  {
    EXPECT_EQ(msgs::Marker::TRIANGLE_LIST, m.type());
    EXPECT_GT(m.point_size(), 0);
    EXPECT_EQ(0, m.point_size() % 3);
  }
  double maxX{-1e9};
  for (const auto &p : markers[1].point())
    maxX = std::max(maxX, p.x());
  EXPECT_LE(maxX, 1.0 + 2.0 + 1e-6) << "the cone reaches 2 m downwind";
  EXPECT_GT(markers[1].material().diffuse().r(),
            markers[1].material().diffuse().b()) << "orange";
}

/////////////////////////////////////////////////
/// Presets come built in, or from the configuration.
TEST(Presets, DefaultsAndConfiguration)
{
  const auto defaults = DefaultPresets();
  ASSERT_EQ(4u, defaults.size());
  EXPECT_EQ("Calm", defaults.front().name);
  EXPECT_EQ("Squall", defaults.back().name);
  for (const auto &p : defaults)
  {
    EXPECT_TRUE(p.current.has_value());
    EXPECT_TRUE(p.wind.has_value());
    EXPECT_GT(p.dwell, 0.0);
  }

  tinyxml2::XMLDocument doc;
  ASSERT_EQ(tinyxml2::XML_SUCCESS, doc.Parse(R"(
    <plugin>
      <preset name="Slack" dwell="5">
        <current speed="0" direction="0"/>
      </preset>
      <preset name="Gale">
        <wind speed="20" direction="270" speed_gust="4"/>
      </preset>
      <preset name="Nothing"/>
      <preset><wind speed="1" direction="0"/></preset>
    </plugin>)"));
  const auto parsed = ParsePresets(doc.RootElement());
  ASSERT_EQ(2u, parsed.size()) << "a preset needs a name and a field";
  EXPECT_EQ("Slack", parsed[0].name);
  EXPECT_DOUBLE_EQ(5.0, parsed[0].dwell);
  EXPECT_FALSE(parsed[0].wind.has_value());
  EXPECT_EQ("Gale", parsed[1].name);
  ASSERT_TRUE(parsed[1].wind.has_value());
  EXPECT_DOUBLE_EQ(270.0, parsed[1].wind->direction);
  ASSERT_TRUE(parsed[1].gusts.has_value());
  EXPECT_DOUBLE_EQ(4.0, parsed[1].gusts->speed);
  EXPECT_DOUBLE_EQ(0.0, parsed[1].gusts->direction);
  EXPECT_TRUE(ParsePresets(nullptr).empty());
}

/////////////////////////////////////////////////
/// The demo steps through the presets on their dwell, in simulation time,
/// and loops.
TEST(Presets, Demo)
{
  std::vector<Preset> presets(2);
  presets[0].name = "A";
  presets[0].dwell = 2.0;
  presets[1].name = "B";
  presets[1].dwell = 1.0;
  using std::chrono::seconds;
  using std::chrono::milliseconds;

  Demo demo;
  EXPECT_FALSE(demo.Update(presets, seconds(0)).has_value()) << "stopped";
  demo.Start();
  EXPECT_EQ(0u, demo.Update(presets, seconds(10)).value());
  EXPECT_FALSE(demo.Update(presets, milliseconds(11900)).has_value());
  EXPECT_EQ(1u, demo.Update(presets, seconds(12)).value());
  EXPECT_FALSE(demo.Update(presets, milliseconds(12500)).has_value());
  EXPECT_EQ(0u, demo.Update(presets, seconds(13)).value()) << "loops";
  EXPECT_EQ(0u, demo.Current().value());
  EXPECT_EQ(0u, demo.Update(presets, seconds(1)).value())
      << "a reset starts it again";
  demo.Stop();
  EXPECT_FALSE(demo.Running());
  EXPECT_FALSE(demo.Current().has_value());
}
