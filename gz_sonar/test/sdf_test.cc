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
#include <string>

#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

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
