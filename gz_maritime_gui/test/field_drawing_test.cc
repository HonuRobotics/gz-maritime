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
// A panel's drawing of a field, without a GUI or a scene: a fake
// MarkerManager array service keeps what the drawing sends. The service is
// in this process, so gz-transport answers a request to it before Request
// returns, and what a call sent, or did not, is known right after it.

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker_v.pb.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/maritime_gui/FieldDrawing.hh"
#include "gz/sim/maritime_gui/FieldSpec.hh"

using namespace gz;
using namespace sim;
using namespace maritime_gui;
using std::chrono::milliseconds;

namespace
{
/// \brief Stands for MarkerManager: keeps what comes on its array service.
class FakeMarkerManager
{
  /// \brief Constructor.
  /// \param[in] _service The marker service; the array service is it with
  /// `_array` after it.
  public: explicit FakeMarkerManager(const std::string &_service)
  {
    this->node.Advertise(_service + "_array",
        &FakeMarkerManager::OnArray, this);
  }

  /// \brief A request came.
  /// \param[in] _req The markers.
  /// \param[out] _rep Whether they were taken.
  /// \return True.
  public: bool OnArray(const msgs::Marker_V &_req, msgs::Boolean &_rep)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->received.push_back(_req);
    _rep.set_data(true);
    return true;
  }

  /// \brief The requests so far, and forget them.
  /// \return The requests.
  public: std::vector<msgs::Marker_V> Take()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    auto out = std::move(this->received);
    this->received.clear();
    return out;
  }

  /// \brief The requests.
  private: std::vector<msgs::Marker_V> received;

  /// \brief Guards them.
  private: std::mutex mutex;

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief What the requests did to a namespace's markers.
struct Changes
{
  /// \brief Ids added or replaced, with points.
  std::set<std::uint64_t> added;

  /// \brief Ids deleted.
  std::set<std::uint64_t> deleted;

  /// \brief Markers in other namespaces.
  int others{0};
};

/// \brief Sort the markers of requests by what they did.
/// \param[in] _requests The requests.
/// \param[in] _ns The namespace.
/// \return The changes.
Changes Sort(const std::vector<msgs::Marker_V> &_requests,
             const std::string &_ns)
{
  Changes c;
  for (const auto &req : _requests)
  {
    for (const auto &m : req.marker())
    {
      if (m.ns() != _ns)
        ++c.others;
      else if (m.action() == msgs::Marker::DELETE_MARKER)
        c.deleted.insert(m.id());
      else
      {
        EXPECT_EQ(msgs::Marker::ADD_MODIFY, m.action());
        EXPECT_GT(m.point_size(), 0) << "an empty bin is not sent";
        EXPECT_NE(0u, m.id()) << "MarkerManager renumbers id 0";
        c.added.insert(m.id());
      }
    }
  }
  return c;
}

/// \brief A drawing whose markers go to a service of its own.
/// \param[in] _service The marker service.
/// \param[in] _xml More configuration.
/// \return The drawing.
std::unique_ptr<FieldDrawing> Drawing(const std::string &_service,
                                      const std::string &_xml = "")
{
  auto drawing = std::make_unique<FieldDrawing>(OceanCurrentSpec(), "test");
  tinyxml2::XMLDocument doc;
  const std::string xml = "<plugin><marker_service>" + _service +
      "</marker_service><extent>10</extent><spacing>5</spacing>" + _xml +
      "</plugin>";
  EXPECT_EQ(tinyxml2::XML_SUCCESS, doc.Parse(xml.c_str()));
  drawing->LoadConfig(doc.RootElement());
  return drawing;
}

/// \brief The configuration of a plugin element.
/// \param[in] _xml The element's children.
/// \return The drawing, configured.
std::unique_ptr<FieldDrawing> Configured(const std::string &_xml)
{
  auto drawing = std::make_unique<FieldDrawing>(OceanCurrentSpec(), "test");
  tinyxml2::XMLDocument doc;
  const std::string xml = "<plugin>" + _xml + "</plugin>";
  EXPECT_EQ(tinyxml2::XML_SUCCESS, doc.Parse(xml.c_str()));
  drawing->LoadConfig(doc.RootElement());
  return drawing;
}

/// \brief A field of that speed east, everywhere.
/// \param[in] _speed m/s.
/// \return The field.
std::function<math::Vector3d(const math::Vector3d &)> East(double _speed)
{
  return [_speed](const math::Vector3d &)
      { return math::Vector3d(_speed, 0.0, 0.0); };
}

/// \brief The arrows' and the trails' namespaces of the ocean current.
const std::string kArrows{"gz_maritime_gui/ocean_current/arrows"};
const std::string kTrails{"gz_maritime_gui/ocean_current/trails"};
}  // namespace

/////////////////////////////////////////////////
/// A display's name and back; anything else is not one.
TEST(FieldDrawing, DisplayNames)
{
  for (const auto display : {Display::kOff, Display::kArrows,
                             Display::kTrails})
  {
    Display parsed{display == Display::kOff ? Display::kArrows :
                   Display::kOff};
    EXPECT_TRUE(ParseDisplay(DisplayName(display), parsed));
    EXPECT_EQ(display, parsed);
  }
  Display kept{Display::kTrails};
  EXPECT_FALSE(ParseDisplay("Arrows", kept));
  EXPECT_FALSE(ParseDisplay("", kept));
  EXPECT_EQ(Display::kTrails, kept) << "untouched";
}

/////////////////////////////////////////////////
/// The configuration is read, and a value out of range falls back.
TEST(FieldDrawing, Configuration)
{
  const auto spec = OceanCurrentSpec();
  {
    FieldDrawing drawing(spec, "test");
    drawing.LoadConfig(nullptr);
    EXPECT_EQ(Display::kOff, drawing.CurrentDisplay());
    EXPECT_DOUBLE_EQ(spec.scale, drawing.Scale());
    EXPECT_DOUBLE_EQ(spec.colormap.MaxSpeed(), drawing.Colors().MaxSpeed());
  }

  auto drawing = Configured(
      "<display>trails</display><extent>60</extent><spacing>2.5</spacing>"
      "<center_x>3</center_x><center_y>-4</center_y><scale>2</scale>"
      "<max_speed>3</max_speed>");
  EXPECT_EQ(Display::kTrails, drawing->CurrentDisplay());
  EXPECT_DOUBLE_EQ(60.0, drawing->ArrowGrid().extent);
  EXPECT_DOUBLE_EQ(2.5, drawing->ArrowGrid().spacing);
  EXPECT_EQ(math::Vector2d(3.0, -4.0), drawing->ArrowGrid().center);
  EXPECT_DOUBLE_EQ(2.0, drawing->Scale());
  EXPECT_DOUBLE_EQ(3.0, drawing->Colors().MaxSpeed());

  drawing = Configured(
      "<display>sideways</display><extent>-1</extent><spacing>0</spacing>"
      "<max_speed>-2</max_speed>");
  EXPECT_EQ(Display::kOff, drawing->CurrentDisplay());
  EXPECT_DOUBLE_EQ(40.0, drawing->ArrowGrid().extent);
  EXPECT_DOUBLE_EQ(5.0, drawing->ArrowGrid().spacing);
  EXPECT_DOUBLE_EQ(spec.colormap.MaxSpeed(), drawing->Colors().MaxSpeed());

  drawing = Configured("<display/><spacing>-3</spacing>");
  EXPECT_EQ(Display::kOff, drawing->CurrentDisplay());
  EXPECT_DOUBLE_EQ(5.0, drawing->ArrowGrid().spacing);
}

/////////////////////////////////////////////////
/// A setter takes a value in range and says it changed, and leaves the
/// setting alone otherwise.
TEST(FieldDrawing, Setters)
{
  FieldDrawing drawing(OceanCurrentSpec(), "test");
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_TRUE(drawing.SetMaxSpeed(2.0));
  for (const double bad : {0.0, -1.0, inf, nan})
    EXPECT_FALSE(drawing.SetMaxSpeed(bad)) << bad;
  EXPECT_DOUBLE_EQ(2.0, drawing.Colors().MaxSpeed());

  EXPECT_TRUE(drawing.SetExtent(0.0)) << "a single arrow";
  EXPECT_TRUE(drawing.SetExtent(25.0));
  for (const double bad : {-1.0, inf, nan})
    EXPECT_FALSE(drawing.SetExtent(bad)) << bad;
  EXPECT_DOUBLE_EQ(25.0, drawing.ArrowGrid().extent);

  EXPECT_TRUE(drawing.SetSpacing(1.5));
  for (const double bad : {0.0, -1.0, inf, nan})
    EXPECT_FALSE(drawing.SetSpacing(bad)) << bad;
  EXPECT_DOUBLE_EQ(1.5, drawing.ArrowGrid().spacing);

  drawing.SetDisplay(Display::kArrows);
  EXPECT_EQ(Display::kArrows, drawing.CurrentDisplay());
}

/////////////////////////////////////////////////
/// Off draws nothing, and nothing is taken off a scene never drawn on.
TEST(FieldDrawing, OffDrawsNothing)
{
  FakeMarkerManager mm("/test_off");
  {
    auto drawing = Drawing("/test_off");
    drawing->Draw(true, 1, true, 0.0, East(0.5), milliseconds(0));
    drawing->Draw(true, 2, true, 0.0, East(0.5), milliseconds(500));
  }
  EXPECT_TRUE(mm.Take().empty());
}

/////////////////////////////////////////////////
/// The arrows are drawn when they are switched on, when the recipe
/// changes, when a setting changes, and a few times a second of simulation
/// time while the field changes on its own; not otherwise.
TEST(FieldDrawing, ArrowsRedrawWhenDue)
{
  FakeMarkerManager mm("/test_arrows");
  auto drawing = Drawing("/test_arrows", "<display>arrows</display>");

  drawing->Draw(true, 1, false, 0.0, East(0.5), milliseconds(0));
  auto requests = mm.Take();
  ASSERT_EQ(1u, requests.size());
  auto c = Sort(requests, kArrows);
  EXPECT_FALSE(c.added.empty());
  EXPECT_TRUE(c.deleted.empty());
  EXPECT_EQ(0, c.others);
  int points{0};
  for (const auto &m : requests[0].marker())
  {
    EXPECT_EQ(msgs::Marker::LINE_LIST, m.type());
    points += m.point_size();
  }
  // A 10 m grid every 5 m: 3 x 3 arrows, a shaft and two barbs each.
  EXPECT_EQ(3 * 3 * 3 * 2, points);

  // The same recipe, still: nothing.
  drawing->Draw(true, 1, false, 0.0, East(0.5), milliseconds(1000));
  EXPECT_TRUE(mm.Take().empty());

  // A new generation.
  drawing->Draw(true, 2, false, 0.0, East(0.5), milliseconds(1100));
  EXPECT_EQ(1u, mm.Take().size());

  // A setting.
  ASSERT_TRUE(drawing->SetSpacing(10.0));
  drawing->Draw(true, 2, false, 0.0, East(0.5), milliseconds(1200));
  requests = mm.Take();
  ASSERT_EQ(1u, requests.size());
  points = 0;
  for (const auto &m : requests[0].marker())
    points += m.point_size();
  EXPECT_EQ(2 * 2 * 3 * 2, points) << "2 x 2 arrows now";

  // The colour map's top speed, the grid's side.
  ASSERT_TRUE(drawing->SetMaxSpeed(2.0));
  drawing->Draw(true, 2, false, 0.0, East(0.5), milliseconds(1210));
  EXPECT_EQ(1u, mm.Take().size());
  ASSERT_TRUE(drawing->SetExtent(20.0));
  drawing->Draw(true, 2, false, 0.0, East(0.5), milliseconds(1220));
  EXPECT_EQ(1u, mm.Take().size());

  // Redraw.
  drawing->Redraw();
  drawing->Draw(true, 2, false, 0.0, East(0.5), milliseconds(1300));
  EXPECT_EQ(1u, mm.Take().size());

  // Changing on its own: every 200 ms of simulation time.
  drawing->Draw(true, 2, true, 0.0, East(0.5), milliseconds(1400));
  EXPECT_TRUE(mm.Take().empty()) << "100 ms after the last";
  drawing->Draw(true, 2, true, 0.0, East(0.5), milliseconds(1500));
  EXPECT_EQ(1u, mm.Take().size());
}

/////////////////////////////////////////////////
/// A bin that empties is deleted, since a marker sent with no points keeps
/// its old ones; a field that is gone, or a display switched off, takes
/// the arrows off.
TEST(FieldDrawing, ArrowsDeleteWhatEmpties)
{
  FakeMarkerManager mm("/test_empties");
  auto drawing = Drawing("/test_empties", "<display>arrows</display>");

  drawing->Draw(true, 1, false, 0.0, East(1.0), milliseconds(0));
  const auto fast = Sort(mm.Take(), kArrows);
  ASSERT_FALSE(fast.added.empty());

  // Slower: other bins.
  drawing->Draw(true, 2, false, 0.0, East(0.3), milliseconds(100));
  const auto slow = Sort(mm.Take(), kArrows);
  ASSERT_FALSE(slow.added.empty());
  EXPECT_NE(fast.added, slow.added);
  for (const auto id : slow.deleted)
  {
    EXPECT_EQ(1u, fast.added.count(id)) << id;
    EXPECT_EQ(0u, slow.added.count(id)) << id;
  }
  EXPECT_FALSE(slow.deleted.empty());

  // Still water: every bin empties. A field with no arrows to draw is
  // still drawn, so it is not redrawn at every update.
  drawing->Draw(true, 3, false, 0.0, East(0.0), milliseconds(200));
  const auto still = Sort(mm.Take(), kArrows);
  EXPECT_TRUE(still.added.empty());
  EXPECT_EQ(slow.added, still.deleted);
  drawing->Draw(true, 3, false, 0.0, East(0.0), milliseconds(300));
  EXPECT_TRUE(mm.Take().empty());

  // The field goes away: the arrows with it.
  drawing->Draw(true, 4, false, 0.0, East(0.5), milliseconds(400));
  const auto back = Sort(mm.Take(), kArrows);
  ASSERT_FALSE(back.added.empty());
  drawing->Draw(false, 4, false, 0.0, East(0.5), milliseconds(500));
  auto gone = Sort(mm.Take(), kArrows);
  EXPECT_TRUE(gone.added.empty());
  EXPECT_EQ(back.added, gone.deleted);

  // It comes back, and the display is switched off.
  drawing->Draw(true, 4, false, 0.0, East(0.5), milliseconds(600));
  ASSERT_EQ(back.added, Sort(mm.Take(), kArrows).added)
      << "drawn again, though the recipe is the same";
  drawing->SetDisplay(Display::kOff);
  drawing->Draw(true, 4, false, 0.0, East(0.5), milliseconds(700));
  EXPECT_EQ(back.added, Sort(mm.Take(), kArrows).deleted);
  drawing->Draw(true, 4, false, 0.0, East(0.5), milliseconds(800));
  EXPECT_TRUE(mm.Take().empty()) << "deleted once";
}

/////////////////////////////////////////////////
/// The trails move with simulation time, a frame every 40 ms of it, and
/// switching to the arrows takes them off.
TEST(FieldDrawing, Trails)
{
  FakeMarkerManager mm("/test_trails");
  auto drawing = Drawing("/test_trails",
      "<display>trails</display><particles>100</particles>"
      "<trail_length>4</trail_length><lifetime>100</lifetime>");

  // The first frame has no time to move in; trails grow as time goes.
  std::set<std::uint64_t> drawn;
  auto t = milliseconds(0);
  for (int i = 0; i < 25; ++i, t += milliseconds(40))
  {
    drawing->Draw(true, 1, false, 0.0, East(0.5), t);
    const auto requests = mm.Take();
    EXPECT_LE(requests.size(), 1u);
    const auto c = Sort(requests, kTrails);
    EXPECT_EQ(0, c.others);
    drawn.insert(c.added.begin(), c.added.end());
    for (const auto id : c.deleted)
      drawn.erase(id);
  }
  ASSERT_FALSE(drawn.empty()) << "trails after a second";

  // Less than a frame later: nothing.
  drawing->Draw(true, 1, false, 0.0, East(0.5), t - milliseconds(20));
  EXPECT_TRUE(mm.Take().empty());

  // A paused world: the next frame has the same time, so none.
  drawing->Draw(true, 1, false, 0.0, East(0.5), t - milliseconds(40));
  EXPECT_TRUE(mm.Take().empty());

  // Another plane configures them again, and the trails restart.
  drawing->Draw(true, 1, false, 2.0, East(0.5), t);
  for (const auto &req : mm.Take())
  {
    for (const auto &m : req.marker())
    {
      for (const auto &p : m.point())
        EXPECT_DOUBLE_EQ(2.0, p.z());
    }
  }

  // The arrows instead: the trails go, every one of them, and the arrows
  // come.
  drawing->SetDisplay(Display::kArrows);
  drawing->Draw(true, 1, false, 2.0, East(0.5), t + milliseconds(40));
  const auto requests = mm.Take();
  const auto trails = Sort(requests, kTrails);
  EXPECT_TRUE(trails.added.empty());
  EXPECT_FALSE(trails.deleted.empty());
  EXPECT_FALSE(Sort(requests, kArrows).added.empty());
}

/////////////////////////////////////////////////
/// A drawing besides the field has a namespace of its own: markers with
/// points are drawn, those drawn before and empty or missing now are
/// deleted, and ClearExtra takes all of them off.
TEST(FieldDrawing, Extras)
{
  const std::string ns{"gz_maritime_gui/ocean_current/vessels"};
  FakeMarkerManager mm("/test_extras");
  auto drawing = Drawing("/test_extras");

  const auto marker = [](std::uint64_t _id, int _points)
  {
    msgs::Marker m;
    m.set_id(_id);
    m.set_ns("overwritten");
    m.set_action(msgs::Marker::ADD_MODIFY);
    for (int i = 0; i < _points; ++i)
      msgs::Set(m.add_point(), math::Vector3d(i, 0, 0));
    return m;
  };

  drawing->DrawExtra("vessels", {marker(1, 2), marker(2, 2), marker(3, 0)});
  auto c = Sort(mm.Take(), ns);
  EXPECT_EQ((std::set<std::uint64_t>{1, 2}), c.added);
  EXPECT_TRUE(c.deleted.empty());
  EXPECT_EQ(0, c.others) << "the namespace is set";

  // One vehicle gone, another without an arrow now.
  drawing->DrawExtra("vessels", {marker(1, 2), marker(3, 0)});
  c = Sort(mm.Take(), ns);
  EXPECT_EQ((std::set<std::uint64_t>{1}), c.added);
  EXPECT_EQ((std::set<std::uint64_t>{2}), c.deleted);

  // Nothing drawn, nothing to delete: no request.
  drawing->DrawExtra("socks", {marker(1, 0)});
  EXPECT_TRUE(mm.Take().empty());
  drawing->ClearExtra("never_drawn");
  EXPECT_TRUE(mm.Take().empty());

  drawing->ClearExtra("vessels");
  c = Sort(mm.Take(), ns);
  EXPECT_TRUE(c.added.empty());
  EXPECT_EQ((std::set<std::uint64_t>{1}), c.deleted);
  drawing->ClearExtra("vessels");
  EXPECT_TRUE(mm.Take().empty()) << "cleared once";
}

/////////////////////////////////////////////////
/// Closing the panel takes everything it drew off the scene.
TEST(FieldDrawing, DestructorClears)
{
  FakeMarkerManager mm("/test_close");
  auto drawing = Drawing("/test_close", "<display>arrows</display>");
  drawing->Draw(true, 1, false, 0.0, East(0.5), milliseconds(0));
  msgs::Marker sock;
  sock.set_id(7);
  msgs::Set(sock.add_point(), math::Vector3d::Zero);
  drawing->DrawExtra("socks", {sock});
  const auto arrows = Sort(mm.Take(), kArrows).added;
  ASSERT_FALSE(arrows.empty());

  drawing.reset();
  const auto requests = mm.Take();
  EXPECT_EQ(arrows, Sort(requests, kArrows).deleted);
  EXPECT_EQ((std::set<std::uint64_t>{7}),
            Sort(requests, "gz_maritime_gui/ocean_current/socks").deleted);
}
