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

// The <gz:sonar> parser and its validation, on SDF snippets.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "gz/maritime/sonar/CameraSampler.hh"
#include "gz/maritime/sonar/Engine.hh"
#include "gz/maritime/sonar/Rays.hh"
#include "gz/maritime/sonar/Readout.hh"
#include "gz/maritime/sonar/Water.hh"

#include "SonarSdf.hh"

namespace
{
namespace sonar = gz::maritime::sonar;
using gz::sim::maritime::LoadSonarConfig;

//////////////////////////////////////////////////
/// \brief Parse one sensor of the given type with the given <gz:sonar> body.
/// \param[in] _type The sensor's gz:type.
/// \param[in] _body The <gz:sonar> body.
/// \param[out] _labels Whether the labels topic is on.
/// \param[out] _error Why it failed.
/// \return The configuration, or empty.
std::optional<sonar::SonarConfig> Load(const std::string &_type,
  const std::string &_body, bool &_labels, std::string &_error)
{
  const std::string xml =
    "<?xml version='1.0'?><sdf version='1.12' "
    "xmlns:gz='http://gazebosim.org/schema'><model name='m'><link name='l'>"
    "<sensor name='s' type='custom' gz:type='" + _type + "'><gz:sonar>" +
    _body + "</gz:sonar></sensor></link></model></sdf>";
  static sdf::Root root;
  root = sdf::Root();
  const auto errors = root.LoadSdfString(xml);
  EXPECT_TRUE(errors.empty()) << errors.front().Message();
  const auto *sensor = root.Model()->LinkByIndex(0)->SensorByIndex(0);
  return LoadSonarConfig(_type, sensor->Element()->FindElement("gz:sonar"),
    _labels, _error);
}

//////////////////////////////////////////////////
std::optional<sonar::SonarConfig> Load(const std::string &_type,
  const std::string &_body, std::string &_error)
{
  bool labels = false;
  return Load(_type, _body, labels, _error);
}
}  // namespace

//////////////////////////////////////////////////
TEST(SonarSdf, EmptyBlockGivesEachExemplar)
{
  std::string error;
  const auto imaging = Load("imaging_sonar", "", error);
  ASSERT_TRUE(imaging) << error;
  EXPECT_DOUBLE_EQ(imaging->frequency, 700000.0);
  EXPECT_EQ(imaging->beams, 256u);
  EXPECT_DOUBLE_EQ(imaging->sidelobeLevelDb, -25.0);
  EXPECT_DOUBLE_EQ(imaging->elevationSpan, 3.0);

  const auto sidescan = Load("sidescan_sonar", "", error);
  ASSERT_TRUE(sidescan) << error;
  EXPECT_DOUBLE_EQ(sidescan->frequency, 450000.0);
  EXPECT_EQ(sidescan->beams, 1u);

  const auto profiling = Load("profiling_sonar", "", error);
  ASSERT_TRUE(profiling) << error;
  EXPECT_DOUBLE_EQ(profiling->frequency, 240000.0);
  EXPECT_EQ(profiling->maxSoundings, 20u);

  EXPECT_FALSE(Load("radar", "", error));

  // A fixed gain by default, so the absolute level survives; auto asks for
  // the per-frame automatic gain.
  for (const auto &c : {imaging, sidescan, profiling})
    EXPECT_TRUE(c->gainDb);
  const auto automatic = Load("imaging_sonar", "<gain>auto</gain>", error);
  ASSERT_TRUE(automatic) << error;
  EXPECT_FALSE(automatic->gainDb);
}

//////////////////////////////////////////////////
TEST(SonarSdf, ElementsOverrideTheDefaults)
{
  std::string error;
  bool labels = false;
  const auto c = Load("imaging_sonar",
    "<range><min>0.5</min><max>12</max></range><samples>500</samples>"
    "<gain>40</gain><tvg>none</tvg><labels>true</labels>"
    "<elevation_span>2</elevation_span><sidelobe_level>-13</sidelobe_level>",
    labels, error);
  ASSERT_TRUE(c) << error;
  EXPECT_DOUBLE_EQ(c->rangeMin, 0.5);
  EXPECT_DOUBLE_EQ(c->rangeMax, 12.0);
  EXPECT_EQ(c->samples, 500u);
  ASSERT_TRUE(c->gainDb);
  EXPECT_DOUBLE_EQ(*c->gainDb, 40.0);
  EXPECT_EQ(c->tvg, sonar::Tvg::kNone);
  EXPECT_TRUE(labels);
  EXPECT_DOUBLE_EQ(c->elevationSpan, 2.0);
  EXPECT_DOUBLE_EQ(c->sidelobeLevelDb, -13.0);

  // The flat spellings a vehicle's parts config passes.
  const auto flat = Load("imaging_sonar", "<range_max>15</range_max>", error);
  ASSERT_TRUE(flat) << error;
  EXPECT_DOUBLE_EQ(flat->rangeMax, 15.0);

  // Unknown elements are warned about, not fatal.
  EXPECT_TRUE(Load("imaging_sonar", "<multipath>true</multipath>", error));
}

//////////////////////////////////////////////////
TEST(SonarSdf, BadValuesAreRefused)
{
  const char *bad[] = {
    "<range><min>5</min><max>2</max></range>",
    "<samples>0</samples>",
    "<gain>loud</gain>",
    "<tvg>sometimes</tvg>",
    "<detection>guess</detection>",
    "<pulse_length>0</pulse_length>",
    "<sound_speed>-1</sound_speed>",
    "<dynamic_range>0</dynamic_range>",
    "<sidelobe_level>3</sidelobe_level>",
    "<elevation_span>0</elevation_span>",
    "<vertical_fov>1.5</vertical_fov><elevation_span>3</elevation_span>",
    "<range_noise>-0.1</range_noise>",
  };
  for (const char *body : bad)
  {
    std::string error;
    EXPECT_FALSE(Load("profiling_sonar", body, error)) << body;
    EXPECT_FALSE(error.empty()) << body;
  }
}

//////////////////////////////////////////////////
TEST(SonarSdf, ZeroSpecularWidthIsIgnored)
{
  sonar::AcousticProperties p;
  p.specularWidth = 0.07;
  gz::sim::maritime::AcousticMarks m;
  m.specularWidth = 0.0;
  gz::sim::maritime::ApplyMarks(p, m);
  EXPECT_DOUBLE_EQ(p.specularWidth, 0.07);
  m.specularWidth = 0.1;
  gz::sim::maritime::ApplyMarks(p, m);
  EXPECT_DOUBLE_EQ(p.specularWidth, 0.1);
}

//////////////////////////////////////////////////
TEST(SonarSdf, MalformedValuesAreRefused)
{
  // Values that used to read as 0, as a prefix of themselves, or not at all.
  const char *bad[] = {
    "<range_max>inf</range_max>",
    "<range_max>nan</range_max>",
    "<gain>40dB</gain>",
    "<noise_floor>abc</noise_floor>",
    "<samples>1e3</samples>",
    "<samples>-5</samples>",
    "<labels>yes</labels>",
    "<speckle>maybe</speckle>",
    "<beams>100000</beams>",
    "<frequency>700kHz</frequency>",
  };
  for (const char *body : bad)
  {
    std::string error;
    EXPECT_FALSE(Load("imaging_sonar", body, error)) << body;
    EXPECT_FALSE(error.empty()) << body;
  }
  std::string error;
  bool labels = false;
  ASSERT_TRUE(Load("imaging_sonar", "<labels>1</labels><samples>500</samples>",
    labels, error)) << error;
  EXPECT_TRUE(labels);
}

namespace
{
//////////////////////////////////////////////////
/// \brief One <visual> with the given acoustic mark attributes.
/// \param[in] _attributes The attributes, as SDF text.
/// \return The marks read from it.
gz::sim::maritime::AcousticMarks Marks(const std::string &_attributes)
{
  const std::string xml =
    "<?xml version='1.0'?><sdf version='1.12' "
    "xmlns:gz='http://gazebosim.org/schema'><model name='m'><link name='l'>"
    "<visual name='v' " + _attributes + "><geometry><box><size>1 1 1</size>"
    "</box></geometry></visual></link></model></sdf>";
  static sdf::Root root;
  root = sdf::Root();
  const auto errors = root.LoadSdfString(xml);
  EXPECT_TRUE(errors.empty()) << errors.front().Message();
  return gz::sim::maritime::ReadAcousticMarks(
    root.Model()->LinkByIndex(0)->VisualByIndex(0)->Element());
}

//////////////////////////////////////////////////
/// \brief The sonar system's <plugin> element with the given children.
/// \param[in] _body The children, as SDF text.
/// \return The element.
sdf::ElementPtr Plugin(const std::string &_body)
{
  const std::string xml =
    "<?xml version='1.0'?><sdf version='1.12'><world name='w'>"
    "<plugin filename='gz-maritime-sonar-system' "
    "name='gz::sim::maritime::Sonar'>" + _body + "</plugin></world></sdf>";
  static sdf::Root root;
  root = sdf::Root();
  const auto errors = root.LoadSdfString(xml);
  EXPECT_TRUE(errors.empty()) << errors.front().Message();
  return root.WorldByIndex(0)->Element()->FindElement("plugin");
}
}  // namespace

//////////////////////////////////////////////////
TEST(SonarSdf, TransparentTakesItsValue)
{
  EXPECT_TRUE(Marks(R"(gz:acoustic_transparent="true")").transparent);
  EXPECT_TRUE(Marks(R"(gz:acoustic_transparent="1")").transparent);
  EXPECT_FALSE(Marks(R"(gz:acoustic_transparent="false")").transparent);
  EXPECT_FALSE(Marks("").transparent);
  // A malformed value is reported and left unset.
  EXPECT_FALSE(Marks(R"(gz:acoustic_lambert="loud")").lambertDb.has_value());
  const auto m = Marks(R"(gz:acoustic_lambert="-27" gz:acoustic_specular="-3" )"
    R"(gz:acoustic_specular_width="0.07")");
  EXPECT_DOUBLE_EQ(*m.lambertDb, -27.0);
  EXPECT_DOUBLE_EQ(*m.specularDb, -3.0);
  EXPECT_DOUBLE_EQ(*m.specularWidth, 0.07);
}

//////////////////////////////////////////////////
TEST(SonarSdf, WaterIsChecked)
{
  sonar::Water water;
  std::uint64_t seed = 0;
  std::string error;
  ASSERT_TRUE(gz::sim::maritime::ReadWater(Plugin(
    "<temperature>27</temperature><salinity>35</salinity>"
    "<sound_speed>1538</sound_speed><seed>42</seed>"), water, seed, error))
    << error;
  EXPECT_DOUBLE_EQ(water.temperature, 27.0);
  EXPECT_DOUBLE_EQ(water.soundSpeed, 1538.0);
  EXPECT_EQ(seed, 42u);

  for (const char *body : {"<temperature>80</temperature>",
                           "<salinity>-1</salinity>",
                           "<sound_speed>abc</sound_speed>",
                           "<sound_speed>inf</sound_speed>",
                           "<seed>-3</seed>"})
  {
    sonar::Water w;
    std::uint64_t s = 0;
    std::string e;
    EXPECT_FALSE(gz::sim::maritime::ReadWater(Plugin(body), w, s, e)) << body;
    EXPECT_FALSE(e.empty()) << body;
  }
}

//////////////////////////////////////////////////
// The three configurations from review that took a server down or built
// far too much: each now loads a small camera or is refused by name.
TEST(SonarSdf, LayoutIsCheckedBeforeAnythingIsBuilt)
{
  std::string error;
  // One elevation row: was a 45 by 4294967291 camera.
  for (const char *type : {"sidescan_sonar", "imaging_sonar",
                           "profiling_sonar"})
  {
    const auto c = Load(type, "<elevation_rays>1</elevation_rays>"
      "<elevation_span>2</elevation_span>", error);
    ASSERT_TRUE(c) << type << ": " << error;
    const auto layout = sonar::PlanCameraLayout(*c, error);
    ASSERT_TRUE(layout) << type << ": " << error;
    EXPECT_LE(layout->height, 8u) << type;
    EXPECT_LE(layout->width, 700u) << type;
  }
  {
    const auto c = Load("sidescan_sonar",
      "<elevation_rays>1</elevation_rays>", error);
    ASSERT_TRUE(c) << error;
    const auto layout = sonar::PlanCameraLayout(*c, error);
    ASSERT_TRUE(layout) << error;
    EXPECT_EQ(layout->yaws.size(), 1u);
    EXPECT_EQ(layout->width, 45u);
    EXPECT_LE(layout->height, 8u);
  }
  for (const char *type : {"sidescan_sonar", "imaging_sonar",
                           "profiling_sonar"})
  {
    // Hung the first update building 1e11 columns of rays.
    EXPECT_FALSE(Load(type, "<horizontal_fov>1e-9</horizontal_fov>", error));
    EXPECT_NE(error.find("<horizontal_fov>"), std::string::npos) << error;
    EXPECT_NE(error.find("1e-09"), std::string::npos) << error;
    // Degrees for radians: was 99 cameras.
    EXPECT_FALSE(Load(type, "<horizontal_fov>120</horizontal_fov>", error));
    EXPECT_NE(error.find("<horizontal_fov>"), std::string::npos) << error;
    EXPECT_NE(error.find("[120]"), std::string::npos) << error;
    EXPECT_NE(error.find("radians"), std::string::npos) << error;
    // The same mistake in the other angles.
    EXPECT_FALSE(Load(type, "<vertical_fov>20</vertical_fov>", error));
    EXPECT_NE(error.find("<vertical_fov>"), std::string::npos) << error;
    EXPECT_FALSE(Load(type, "<beam_width>4</beam_width>", error));
    EXPECT_NE(error.find("<beam_width>"), std::string::npos) << error;
    EXPECT_FALSE(Load(type, "<angle_noise>30</angle_noise>", error));
    EXPECT_NE(error.find("<angle_noise>"), std::string::npos) << error;
  }
  // Values that pass one by one and multiply into too much.
  EXPECT_FALSE(Load("imaging_sonar", "<beams>2048</beams>"
    "<rays_per_beam>32</rays_per_beam>", error));
  EXPECT_NE(error.find("azimuth columns"), std::string::npos) << error;
  EXPECT_FALSE(Load("sidescan_sonar", "<rays_per_beam>32</rays_per_beam>",
    error));
  EXPECT_NE(error.find("pixels"), std::string::npos) << error;
  EXPECT_FALSE(Load("imaging_sonar", "<horizontal_fov>0.001</horizontal_fov>",
    error));
  // And the exemplars' neighbors still load.
  EXPECT_TRUE(Load("imaging_sonar", "<horizontal_fov>2.2689</horizontal_fov>"
    "<beams>512</beams>", error)) << error;
  EXPECT_TRUE(Load("sidescan_sonar", "<horizontal_fov>0.0035</horizontal_fov>"
    "<beam_width>0.0035</beam_width><range_max>150</range_max>", error))
    << error;
  EXPECT_TRUE(Load("profiling_sonar", "<horizontal_fov>2.4</horizontal_fov>"
    "<beam_width>0.122</beam_width><detection>beamformed</detection>", error))
    << error;
}

namespace
{
//////////////////////////////////////////////////
/// \brief Build what a sensor builds from a configuration the reader took,
/// and ping it once on a wall: the layout is inside its limits, and the
/// levels that come back are numbers.
/// \param[in] _c The configuration.
/// \param[in] _what What is being tried, for the failure message.
/// \param[in] _ping Whether to build the sampler and engine and ping too.
void ExpectBuilds(const sonar::SonarConfig &_c, const std::string &_what,
  bool _ping)
{
  std::string error;
  const auto layout = sonar::PlanCameraLayout(_c, error);
  ASSERT_TRUE(layout) << _what << ": " << error;
  EXPECT_GE(layout->yaws.size(), 1u) << _what;
  EXPECT_LE(layout->yaws.size(), sonar::kMaxCameras) << _what;
  EXPECT_GE(layout->width, 4u) << _what;
  EXPECT_GE(layout->height, 4u) << _what;
  EXPECT_LE(layout->width, sonar::kMaxCameraSide) << _what;
  EXPECT_LE(layout->height, sonar::kMaxCameraSide) << _what;
  EXPECT_LE(static_cast<double>(layout->yaws.size()) * layout->width *
    layout->height, sonar::kMaxCameraPixels) << _what;
  EXPECT_GT(layout->farClip, layout->nearClip) << _what;
  const sonar::RayGrid g = sonar::RayFanGrid(_c);
  const double rays = g.columns * static_cast<double>(g.rows);
  EXPECT_LE(rays, sonar::kMaxFanRays) << _what;
  if (!_ping)
    return;

  const sonar::CameraSampler sampler(_c);
  ASSERT_EQ(static_cast<double>(sampler.Rays().size()), rays) << _what;
  // A wall square to each camera in the middle of the range window.
  const auto depthValue = static_cast<float>(0.5 * (_c.rangeMin + _c.rangeMax));
  const std::vector<float> image(layout->width * layout->height, depthValue);
  const std::vector<const float *> depth(layout->yaws.size(), image.data());
  const auto hits = sampler.Sample(depth, {}, 1, {});
  ASSERT_EQ(hits.size(), sampler.Rays().size()) << _what;
  const sonar::Engine engine(_c, sonar::Water());
  const auto out = engine.Ping(hits, 1);
  ASSERT_EQ(out.data.size(), _c.beams * _c.samples) << _what;
  for (const auto v : out.data)
    ASSERT_TRUE(std::isfinite(v)) << _what;
  // The read-out, where the gains, the noise and the detection are.
  const auto levels = sonar::ReadLevels(out, _c, engine.Absorption(), 1);
  ASSERT_EQ(levels.db.size(), out.data.size()) << _what;
  for (const auto v : levels.db)
    ASSERT_TRUE(std::isfinite(v)) << _what;
  EXPECT_EQ(sonar::ReadImaging(levels, _c).data.size(), levels.db.size())
    << _what;
  for (const auto &s : sonar::ReadProfiling(levels, _c, 1500.0, 1))
  {
    ASSERT_TRUE(std::isfinite(s.angle) && std::isfinite(s.range)) << _what;
  }
}
}  // namespace

//////////////////////////////////////////////////
// Every number in <gz:sonar>, at zero, negative, tiny and huge values and
// at values around its default, for each sonar type: the reader either
// refuses it with a reason, or what it returns builds a sane layout, and
// (for fans of ordinary size) a sampler and an engine that ping.
TEST(SonarSdf, EveryNumberIsRefusedOrBuilds)
{
  const char *types[] = {"imaging_sonar", "sidescan_sonar",
    "profiling_sonar"};
  const char *numbers[] = {"frequency", "horizontal_fov", "vertical_fov",
    "elevation_span", "beam_width", "sidelobe_level", "range_min",
    "range_max", "pulse_length", "sound_speed", "gain",
    "gain_time_constant", "tvg_spreading", "noise_floor", "dynamic_range",
    "detection_threshold", "range_noise", "angle_noise"};
  const char *numberValues[] = {"0", "-0", "-1", "-1e-9", "-1e9", "-1e300",
    "1e-300", "1e-12", "1e-9", "1e-6", "1e-4", "1e-3", "0.01", "0.1", "0.5",
    "1", "1.5", "2", "3", "6.3", "10", "45", "120", "360", "1e3", "1500",
    "1e4", "1e5", "1e6", "1e7", "1e9", "1e12", "1e300", "-13", "-150"};
  const char *counts[] = {"beams", "samples", "rays_per_beam",
    "elevation_rays", "max_soundings"};
  const char *countValues[] = {"0", "1", "2", "3", "7", "16", "32", "33",
    "100", "1000", "1024", "1025", "2048", "2049", "20000", "20001",
    "100000", "999999999", "4294967296", "-1", "1.5"};

  std::size_t accepted = 0, refused = 0, pinged = 0;
  auto check = [&](const std::string &_type, const std::string &_body,
      bool _mayPing) {
    std::string error;
    const auto c = Load(_type, _body, error);
    const std::string what = _type + " " + _body;
    if (!c)
    {
      ++refused;
      EXPECT_FALSE(error.empty()) << what;
      return;
    }
    ++accepted;
    // Ping the fans of ordinary size; the largest are only sized up, so
    // the test stays quick.
    const sonar::RayGrid g = sonar::RayFanGrid(*c);
    const bool ping = _mayPing && g.columns * static_cast<double>(g.rows) <= 150000.0 &&
      g.columns * static_cast<double>(c->samples) <= 3.0e6;
    pinged += ping ? 1 : 0;
    ExpectBuilds(*c, what, ping);
  };
  auto element = [](const std::string &_name, const std::string &_value) {
    return "<" + _name + ">" + _value + "</" + _name + ">";
  };
  for (const char *type : types)
  {
    for (const char *name : numbers)
      for (const char *value : numberValues)
        check(type, element(name, value), true);
    for (const char *name : counts)
      for (const char *value : countValues)
        check(type, element(name, value), true);
    // The nested spelling of the range window.
    for (const char *value : numberValues)
    {
      check(type, std::string("<range><max>") + value + "</max></range>",
        true);
      check(type, std::string("<range><min>") + value + "</min></range>",
        true);
    }
    // Pairs and triples, drawn from the same values with a fixed seed:
    // counts and angles that pass alone can multiply. Sized up, not built.
    std::uint32_t state = 12345u;
    auto next = [&](std::size_t _n) {
      state = state * 1664525u + 1013904223u;
      return static_cast<std::size_t>((state >> 8) % _n);
    };
    const std::size_t nn = sizeof(numbers) / sizeof(numbers[0]);
    const std::size_t nv = sizeof(numberValues) / sizeof(numberValues[0]);
    const std::size_t cn = sizeof(counts) / sizeof(counts[0]);
    const std::size_t cv = sizeof(countValues) / sizeof(countValues[0]);
    for (int n = 0; n < 1500; ++n)
    {
      const std::size_t a = next(nn), b = next(nn);
      std::string body = element(numbers[a], numberValues[next(nv)]) +
        element(counts[next(cn)], countValues[next(cv)]);
      if (b != a)
        body += element(numbers[b], numberValues[next(nv)]);
      check(type, body, false);
    }
  }
  EXPECT_GT(accepted, 500u);
  EXPECT_GT(refused, 500u);
  EXPECT_GT(pinged, 100u);
  std::printf("sweep: %zu accepted (%zu pinged), %zu refused\n", accepted,
    pinged, refused);
}
