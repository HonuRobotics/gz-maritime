// Copyright 2026 Honu Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "kai_sonar_bridge/Conversions.hh"

using kai_sonar_bridge::GzDetections;
using kai_sonar_bridge::GzImage;
using kai_sonar_bridge::RosDetections;
using kai_sonar_bridge::RosImage;
using ImageData = marine_acoustic_msgs::msg::SonarImageData;

namespace
{
//////////////////////////////////////////////////
/// \brief A gz header with a stamp and a frame id.
/// \param[out] _h The header.
/// \param[in] _frame The frame id.
void SetHeader(gz::msgs::Header *_h, const std::string &_frame)
{
  _h->mutable_stamp()->set_sec(12);
  _h->mutable_stamp()->set_nsec(345);
  auto *d = _h->add_data();
  d->set_key("frame_id");
  d->add_value(_frame);
}

//////////////////////////////////////////////////
/// \brief A 5 beam by 4 sample uint8 image whose sample k of beam j is
/// 10 j + k, beam-major as the sensor writes it.
/// \return The image.
GzImage FiveByFour()
{
  GzImage m;
  SetHeader(m.mutable_header(), "rov::sonar");
  m.set_frequency(1.2e6);
  m.set_sound_speed(1500.0);
  m.set_beam_width_horizontal(0.01f);
  m.set_beam_width_vertical(0.2f);
  m.set_beam_count(5);
  m.set_sample_count(4);
  for (float a : {0.4f, 0.2f, 0.0f, -0.2f, -0.4f})
    m.add_azimuths(a);
  for (float r : {1.0f, 2.0f, 3.0f, 4.0f})
    m.add_ranges(r);
  m.set_data_type(GzImage::UINT8);
  std::string data;
  for (int j = 0; j < 5; ++j)
  {
    for (int k = 0; k < 4; ++k)
      data.push_back(static_cast<char>(10 * j + k));
  }
  m.set_data(data);
  return m;
}
}  // namespace

//////////////////////////////////////////////////
TEST(Conversions, TransposesFiveByFour)
{
  RosImage out;
  ASSERT_TRUE(kai_sonar_bridge::ToProjectedSonarImage(FiveByFour(), "", out));
  EXPECT_EQ(out.image.beam_count, 5u);
  EXPECT_EQ(out.ranges.size(), 4u);
  ASSERT_EQ(out.image.data.size(), 20u);
  // Range-major: sample k of beam j at k * 5 + j.
  for (int j = 0; j < 5; ++j)
  {
    for (int k = 0; k < 4; ++k)
      EXPECT_EQ(out.image.data[k * 5 + j], 10 * j + k) << j << " " << k;
  }
  // The first row is every beam at the first range.
  EXPECT_EQ(std::vector<std::uint8_t>(out.image.data.begin(),
    out.image.data.begin() + 5), (std::vector<std::uint8_t>{0, 10, 20, 30, 40}));
  EXPECT_EQ(out.header.stamp.sec, 12);
  EXPECT_EQ(out.header.stamp.nanosec, 345u);
  EXPECT_EQ(out.header.frame_id, "rov::sonar_acoustic");
  EXPECT_FLOAT_EQ(out.ping_info.sound_speed, 1500.0f);
  EXPECT_FLOAT_EQ(out.ping_info.rx_beamwidths.at(0), 0.01f);
  EXPECT_FLOAT_EQ(out.ping_info.tx_beamwidths.at(0), 0.2f);
}

//////////////////////////////////////////////////
TEST(Conversions, BeamDirectionsOnFiveByFour)
{
  RosImage out;
  ASSERT_TRUE(kai_sonar_bridge::ToProjectedSonarImage(FiveByFour(), "f", out));
  EXPECT_EQ(out.header.frame_id, "f");
  ASSERT_EQ(out.beam_directions.size(), 5u);
  const double az[] = {0.4, 0.2, 0.0, -0.2, -0.4};
  for (int j = 0; j < 5; ++j)
  {
    const auto &d = out.beam_directions[j];
    EXPECT_DOUBLE_EQ(d.x, 0.0);
    EXPECT_NEAR(d.y, -std::sin(az[j]), 1e-7);
    EXPECT_NEAR(d.z, std::cos(az[j]), 1e-7);
    EXPECT_NEAR(std::hypot(d.y, d.z), 1.0, 1e-12);
    EXPECT_NEAR(kai_sonar_bridge::BeamAzimuth(d), az[j], 1e-7);
  }
  // Beam 0 is to port (positive azimuth), which is the acoustic frame's -Y.
  EXPECT_LT(out.beam_directions[0].y, 0.0);
  EXPECT_DOUBLE_EQ(out.beam_directions[2].z, 1.0);
}

//////////////////////////////////////////////////
TEST(Conversions, DtypeMapping)
{
  std::uint32_t dtype{99};
  std::size_t bytes{0};
  ASSERT_TRUE(kai_sonar_bridge::ImageDtype(GzImage::UINT8, dtype, bytes));
  EXPECT_EQ(dtype, ImageData::DTYPE_UINT8);
  EXPECT_EQ(bytes, 1u);
  ASSERT_TRUE(kai_sonar_bridge::ImageDtype(GzImage::FLOAT32, dtype, bytes));
  EXPECT_EQ(dtype, ImageData::DTYPE_FLOAT32);
  EXPECT_EQ(bytes, 4u);
  EXPECT_FALSE(kai_sonar_bridge::ImageDtype(7, dtype, bytes));

  // A float32 image keeps each sample's four bytes together.
  GzImage m = FiveByFour();
  m.set_data_type(GzImage::FLOAT32);
  std::string data(5 * 4 * sizeof(float), '\0');
  for (int j = 0; j < 5; ++j)
  {
    for (int k = 0; k < 4; ++k)
    {
      const float v = -0.5f * (10 * j + k);
      std::memcpy(&data[(j * 4 + k) * sizeof(float)], &v, sizeof(float));
    }
  }
  m.set_data(data);
  RosImage out;
  ASSERT_TRUE(kai_sonar_bridge::ToProjectedSonarImage(m, "", out));
  EXPECT_EQ(out.image.dtype, ImageData::DTYPE_FLOAT32);
  EXPECT_FALSE(out.image.is_bigendian);
  for (int j = 0; j < 5; ++j)
  {
    for (int k = 0; k < 4; ++k)
    {
      float v{0.0f};
      std::memcpy(&v, &out.image.data[(k * 5 + j) * sizeof(float)],
        sizeof(float));
      EXPECT_FLOAT_EQ(v, -0.5f * (10 * j + k));
    }
  }

  // An unknown type, or data of the wrong size, is malformed.
  m.set_data_type(static_cast<GzImage::DataType>(7));
  EXPECT_FALSE(kai_sonar_bridge::ToProjectedSonarImage(m, "", out));
  m = FiveByFour();
  m.set_data(std::string(19, '\0'));
  EXPECT_FALSE(kai_sonar_bridge::ToProjectedSonarImage(m, "", out));
  m = FiveByFour();
  m.add_azimuths(0.6f);
  EXPECT_FALSE(kai_sonar_bridge::ToProjectedSonarImage(m, "", out));
}

//////////////////////////////////////////////////
TEST(Conversions, OneSoundingByHand)
{
  // 0.02 s two-way at 1500 m/s is 15 m; the sensor's 0.5 rad is toward +y,
  // which is port for a down-looking sonar, so rx_angle is -0.5 rad.
  GzDetections m;
  SetHeader(m.mutable_header(), "boat::profiler");
  m.set_sound_speed(1500.0);
  m.add_angles(0.5f);
  m.add_travel_times(0.02f);
  m.add_intensities(std::numeric_limits<float>::quiet_NaN());
  m.add_flags(0);
  // A flagged sounding, which the cloud leaves out.
  m.add_angles(-0.1f);
  m.add_travel_times(0.01f);
  m.add_intensities(3.0f);
  m.add_flags(1);

  RosDetections out;
  ASSERT_TRUE(kai_sonar_bridge::ToSonarDetections(m, "", out));
  EXPECT_EQ(out.header.frame_id, "boat::profiler_acoustic");
  ASSERT_EQ(out.rx_angles.size(), 2u);
  EXPECT_FLOAT_EQ(out.rx_angles[0], -0.5f);
  EXPECT_FLOAT_EQ(out.rx_angles[1], 0.1f);
  EXPECT_FLOAT_EQ(out.two_way_travel_times[0], 0.02f);
  EXPECT_FLOAT_EQ(out.intensities[0], 0.0f);
  EXPECT_FLOAT_EQ(out.intensities[1], 3.0f);
  EXPECT_EQ(out.flags[1].flag, 1);
  EXPECT_EQ(out.tx_angles.size(), 2u);
  EXPECT_EQ(out.tx_delays.size(), 2u);

  // By hand: 15 m along (0, sin -0.5, cos -0.5) = (0, -7.1914, 13.1637),
  // 7.19 m to port and 13.16 m down in the acoustic (north-east-down) frame.
  const auto p = kai_sonar_bridge::SoundingPoint(out.rx_angles[0],
    out.two_way_travel_times[0], out.ping_info.sound_speed);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, -7.1914, 1e-4);
  EXPECT_NEAR(p.z, 13.1637, 1e-4);

  // The same direction as an image beam at the sensor's azimuth 0.5 rad:
  // one frame convention for both.
  const auto d = kai_sonar_bridge::BeamDirection(0.5);
  EXPECT_NEAR(p.y, 15.0 * d.y, 1e-4);
  EXPECT_NEAR(p.z, 15.0 * d.z, 1e-4);

  const auto cloud = kai_sonar_bridge::SoundingsCloud(out);
  EXPECT_EQ(cloud.header.frame_id, out.header.frame_id);
  ASSERT_EQ(cloud.width * cloud.height, 1u);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  EXPECT_NEAR(*x, 0.0, 1e-6);
  EXPECT_NEAR(*y, -7.1914, 1e-3);
  EXPECT_NEAR(*z, 13.1637, 1e-3);

  // Arrays of different lengths are malformed.
  m.add_flags(0);
  EXPECT_FALSE(kai_sonar_bridge::ToSonarDetections(m, "", out));
}

//////////////////////////////////////////////////
TEST(Conversions, EmptyCloud)
{
  RosDetections det;
  const auto cloud = kai_sonar_bridge::SoundingsCloud(det);
  EXPECT_EQ(cloud.width * cloud.height, 0u);
  EXPECT_EQ(cloud.fields.size(), 4u);
}

//////////////////////////////////////////////////
TEST(Conversions, FanFitsItsBox)
{
  int w{0};
  int h{0};
  // A wide fan, 130 degrees: as wide as the side, half as tall.
  kai_sonar_bridge::FanSize(15.0, -1.1345, 1.1345, 600, w, h);
  EXPECT_EQ(w, 600);
  EXPECT_NEAR(h, 600 / (2.0 * std::sin(1.1345)) + 1, 1.0);
  // A narrow fan, 10 degrees: as tall as the side, not 3442 pixels tall as it was.
  kai_sonar_bridge::FanSize(15.0, -0.0873, 0.0873, 600, w, h);
  EXPECT_EQ(h, 601);
  EXPECT_NEAR(w, 600 * 2.0 * std::sin(0.0873), 1.0);
  // One beam, on axis: drawn as a degree either side.
  kai_sonar_bridge::FanSize(15.0, 0.0, 0.0, 600, w, h);
  EXPECT_EQ(h, 601);
  EXPECT_EQ(w, 21);
}

//////////////////////////////////////////////////
TEST(Conversions, DrawsFiveByFourFan)
{
  RosImage img;
  ASSERT_TRUE(kai_sonar_bridge::ToProjectedSonarImage(FiveByFour(), "", img));
  const auto layout = kai_sonar_bridge::LayOutFan(img, 100);
  // +-0.4 rad to 4 m: 2 sin 0.4 = 0.78 of the range, so taller than wide.
  EXPECT_EQ(layout.height, 101);
  EXPECT_EQ(layout.width, 78);
  const auto view = kai_sonar_bridge::DrawFan(img, layout);
  EXPECT_EQ(view.encoding, "mono8");
  ASSERT_EQ(view.data.size(), 78u * 101u);
  // The pixel straight ahead at 4 m (the top center) is beam 2, sample 3.
  EXPECT_EQ(view.data[39], 23);
  // At 2 m straight ahead, beam 2, sample 1.
  EXPECT_EQ(view.data[50 * 78 + 39], 21);
  // To the left is port, beam 0: at 3 m and 0.4 rad.
  const int u = static_cast<int>(39 - 3.0 * std::sin(0.39) * 25.0);
  const int v = static_cast<int>(100 - 3.0 * std::cos(0.39) * 25.0);
  EXPECT_EQ(view.data[v * 78 + u], 2);
  // Behind the sonar and outside the fan there is nothing.
  EXPECT_EQ(view.data[100 * 78 + 0], 0);
}

//////////////////////////////////////////////////
TEST(Conversions, WaterfallLine)
{
  RosImage img;
  img.ranges = {1.0f, 2.0f, 3.0f};
  img.image.beam_count = 1;
  img.image.dtype = ImageData::DTYPE_FLOAT32;
  const float db[] = {-60.0f, -30.0f, 5.0f};
  img.image.data.resize(sizeof(db));
  std::memcpy(img.image.data.data(), db, sizeof(db));
  EXPECT_EQ(kai_sonar_bridge::WaterfallLine(img, 60.0),
    (std::vector<std::uint8_t>{0, 128, 255}));
}
