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

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Helpers.hh>

#include "gz/sim/underwater_camera/UnderwaterImage.hh"

using namespace gz;
using namespace sim::underwater_camera;

namespace
{
/// \brief A 3C-like water, explicit, so the test does not depend on the
/// table.
Water CoastalAt(double _depth)
{
  Water w;
  w.attenuation = Rgb(1.16, 1.23, 1.65);
  w.backscatter = Rgb(1.16, 1.23, 1.65);
  w.diffuseAttenuation = Rgb(0.456, 0.187, 0.319);
  const Rgb veil(0.74, 0.94, 0.91);
  w.veil = Rgb(veil.X() * std::exp(-w.diffuseAttenuation.X() * _depth),
               veil.Y() * std::exp(-w.diffuseAttenuation.Y() * _depth),
               veil.Z() * std::exp(-w.diffuseAttenuation.Z() * _depth));
  w.cameraDepth = _depth;
  return w;
}

Intrinsics Camera(std::uint32_t _w, std::uint32_t _h, double _hfov)
{
  Intrinsics k;
  k.width = _w;
  k.height = _h;
  k.fx = k.fy = _w / (2.0 * std::tan(_hfov / 2.0));
  k.cx = _w / 2.0;
  k.cy = _h / 2.0;
  return k;
}

/// \brief A uniform image and a uniform depth.
void Uniform(const Intrinsics &_k, std::uint8_t _r, std::uint8_t _g,
             std::uint8_t _b, float _z, std::vector<std::uint8_t> &_rgb,
             std::vector<float> &_depth)
{
  const std::size_t n = static_cast<std::size_t>(_k.width) * _k.height;
  _rgb.resize(n * 3);
  _depth.assign(n, _z);
  for (std::size_t i = 0; i < n; ++i)
  {
    _rgb[3 * i] = _r;
    _rgb[3 * i + 1] = _g;
    _rgb[3 * i + 2] = _b;
  }
}

/// \brief The model, written out independently, for one pixel and channel.
double Expected(double _j, double _r, double _objDepth, double _a,
                double _bk, double _kd, double _veil)
{
  return _j * std::exp(-_kd * std::max(0.0, _objDepth)) *
             std::exp(-_a * _r) +
         _veil * (1.0 - std::exp(-_bk * _r));
}
}  // namespace

/////////////////////////////////////////////////
// The ray table: z-depth to range, and the rays, at the centre, corners and
// edges, against ‖((u−cx)/fx, (v−cy)/fy, 1)‖.
TEST(RayTable, MatchesFormula)
{
  const Intrinsics k = Camera(64, 48, 1.2);
  const RayTable rays(k);
  for (auto [u, v] : {std::pair<std::uint32_t, std::uint32_t>{0, 0},
                      {63, 0}, {0, 47}, {63, 47}, {32, 24}, {0, 24},
                      {32, 0}})
  {
    const double x = (u + 0.5 - k.cx) / k.fx;
    const double y = (v + 0.5 - k.cy) / k.fy;
    EXPECT_NEAR(rays.RangeFactor(u, v), std::sqrt(1 + x * x + y * y), 1e-12)
        << u << "," << v;
    const auto &r = rays.Ray(u, v);
    EXPECT_NEAR(r.Length(), 1.0, 1e-12);
    EXPECT_GT(r.X(), 0.0) << "the camera looks along +x";
    EXPECT_NEAR(r.Y(), -x * r.X(), 1e-12) << "image right is -y";
    EXPECT_NEAR(r.Z(), -y * r.X(), 1e-12) << "image down is -z";
  }
}

/////////////////////////////////////////////////
TEST(Srgb, RoundTripsEveryValue)
{
  for (int i = 0; i < 256; ++i)
  {
    const auto v = static_cast<std::uint8_t>(i);
    EXPECT_EQ(LinearToSrgb(SrgbToLinear(v)), v) << i;
  }
  EXPECT_DOUBLE_EQ(SrgbToLinear(0), 0.0);
  EXPECT_DOUBLE_EQ(SrgbToLinear(255), 1.0);
  EXPECT_NEAR(SrgbToLinear(188), 0.5029, 1e-3) << "sRGB 188 is about 50 %";
  EXPECT_EQ(LinearToSrgb(-1.0), 0);
  EXPECT_EQ(LinearToSrgb(2.0), 255);
  EXPECT_EQ(LinearToSrgb(std::nan("")), 0);
}

/////////////////////////////////////////////////
// The model on a synthetic frame: every pixel against the closed form,
// with its own range and its object's depth.
TEST(Model, ClosedFormPerPixel)
{
  const Intrinsics k = Camera(32, 24, 1.0);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 200, 150, 100, 3.0f, rgb, z);
  // A second depth on the right half.
  for (std::uint32_t v = 0; v < k.height; ++v)
    for (std::uint32_t u = k.width / 2; u < k.width; ++u)
      z[v * k.width + u] = 7.0f;

  const Water w = CoastalAt(4.0);
  std::mt19937 rng(1);
  std::vector<std::uint8_t> out;
  const auto stats = MakeUnderwater(rgb, z, rays, w, Response{}, rng, out);
  ASSERT_TRUE(stats.underwater);
  ASSERT_EQ(out.size(), rgb.size());

  for (std::uint32_t v = 0; v < k.height; ++v)
  {
    for (std::uint32_t u = 0; u < k.width; ++u)
    {
      const std::size_t i = v * k.width + u;
      const double r = z[i] * rays.RangeFactor(u, v);
      const double objDepth = w.cameraDepth - rays.Ray(u, v).Z() * r;
      for (int c = 0; c < 3; ++c)
      {
        const double e = Expected(SrgbToLinear(rgb[3 * i + c]), r, objDepth,
            w.attenuation[c], w.backscatter[c], w.diffuseAttenuation[c],
            w.veil[c]);
        EXPECT_EQ(out[3 * i + c], LinearToSrgb(e))
            << "pixel " << u << "," << v << " channel " << c;
      }
    }
  }
}

/////////////////////////////////////////////////
// β^D ≠ β^B: the revised model differs from the fog model (one
// coefficient) by what the closed form says, and only then.
TEST(Model, RevisedDiffersFromFog)
{
  const Intrinsics k = Camera(8, 8, 0.5);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 128, 128, 128, 4.0f, rgb, z);
  Water fog = CoastalAt(2.0);
  fog.diffuseAttenuation = Rgb::Zero;
  Water revised = fog;
  revised.attenuation = Rgb(0.6, 0.3, 0.4);
  std::mt19937 rng(1);
  std::vector<std::uint8_t> a, b;
  MakeUnderwater(rgb, z, rays, fog, Response{}, rng, a);
  MakeUnderwater(rgb, z, rays, revised, Response{}, rng, b);
  const double r = 4.0 * rays.RangeFactor(4, 4);
  const std::size_t i = 4 * k.width + 4;
  for (int c = 0; c < 3; ++c)
  {
    const double j = SrgbToLinear(128);
    EXPECT_EQ(a[3 * i + c], LinearToSrgb(Expected(j, r, 0, fog.attenuation[c],
        fog.backscatter[c], 0, fog.veil[c])));
    EXPECT_EQ(b[3 * i + c], LinearToSrgb(Expected(j, r, 0,
        revised.attenuation[c], revised.backscatter[c], 0,
        revised.veil[c])));
    EXPECT_GT(b[3 * i + c], a[3 * i + c])
        << "a smaller beta^D lets more of the object through";
  }
}

/////////////////////////////////////////////////
// A climbing ray leaves the water at the surface: only the water up to
// there counts.
TEST(Model, SurfaceClipsClimbingRays)
{
  const Intrinsics k = Camera(9, 9, 0.2);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 255, 255, 255, 50.0f, rgb, z);   // far above the surface
  Water w = CoastalAt(2.0);
  w.diffuseAttenuation = Rgb::Zero;
  // Pitched up 30 degrees (a Gazebo camera pitches up with negative pitch).
  w.cameraRotation = math::Quaterniond(0, -GZ_PI / 6, 0);
  std::mt19937 rng(1);
  std::vector<std::uint8_t> out;
  MakeUnderwater(rgb, z, rays, w, Response{}, rng, out);
  const std::size_t i = 4 * k.width + 4;
  const double climb = (w.cameraRotation * rays.Ray(4, 4)).Z();
  ASSERT_GT(climb, 0.4);
  const double water = 2.0 / climb;
  for (int c = 0; c < 3; ++c)
  {
    EXPECT_EQ(out[3 * i + c], LinearToSrgb(Expected(1.0, water, -100,
        w.attenuation[c], w.backscatter[c], 0, w.veil[c])))
        << "channel " << c;
  }
}

/////////////////////////////////////////////////
// Nothing hit, looking level or down: the open water, the veil.
TEST(Model, NothingHitIsTheVeil)
{
  const Intrinsics k = Camera(8, 8, 0.3);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 10, 20, 250, std::numeric_limits<float>::infinity(), rgb, z);
  const Water w = CoastalAt(5.0);
  std::mt19937 rng(1);
  std::vector<std::uint8_t> out;
  MakeUnderwater(rgb, z, rays, w, Response{}, rng, out);
  // The bottom row looks down: never meets the surface.
  const std::size_t i = 7 * k.width + 3;
  for (int c = 0; c < 3; ++c)
    EXPECT_EQ(out[3 * i + c], LinearToSrgb(w.veil[c]));
}

/////////////////////////////////////////////////
// In air, and in clear water, the image passes through unchanged.
TEST(Model, InAirAndClearWaterUnchanged)
{
  const Intrinsics k = Camera(16, 12, 1.0);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 0, 0, 0, 2.0f, rgb, z);
  for (std::size_t i = 0; i < rgb.size(); ++i)
    rgb[i] = static_cast<std::uint8_t>(i % 256);
  std::mt19937 rng(1);
  std::vector<std::uint8_t> out;

  Water air = CoastalAt(5.0);
  air.cameraDepth = -0.5;
  EXPECT_FALSE(MakeUnderwater(rgb, z, rays, air, Response{}, rng, out)
               .underwater);
  EXPECT_EQ(out, rgb) << "in air";

  Water clear;
  clear.cameraDepth = 3.0;
  EXPECT_TRUE(MakeUnderwater(rgb, z, rays, clear, Response{}, rng, out)
              .underwater);
  EXPECT_EQ(out, rgb) << "clear water";
}

/////////////////////////////////////////////////
// The camera's response applies above the water too: its gain, and the
// noise on the light it caught, which the gain amplifies.
TEST(Response, AppliesInAirAndAmplifiesTheNoise)
{
  const Intrinsics k = Camera(64, 48, 1.0);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 60, 60, 60, 2.0f, rgb, z);
  Water air;
  air.cameraDepth = -1.0;
  std::mt19937 rng(3);
  std::vector<std::uint8_t> out;

  Response bright;
  bright.gain = 2.0;
  MakeUnderwater(rgb, z, rays, air, bright, rng, out);
  EXPECT_EQ(out[0], LinearToSrgb(2.0 * SrgbToLinear(60)));

  // Noise of sigma 0.002 on the sensor: 4 times that after a gain of 4
  Response noisy;
  noisy.readNoise = 0.002;
  noisy.gain = 4.0;
  MakeUnderwater(rgb, z, rays, air, noisy, rng, out);
  double sum = 0, sum2 = 0;
  const std::size_t n = out.size();
  for (std::uint8_t v : out)
  {
    sum += SrgbToLinear(v);
    sum2 += SrgbToLinear(v) * SrgbToLinear(v);
  }
  const double sigma = std::sqrt(sum2 / n - (sum / n) * (sum / n));
  EXPECT_NEAR(sigma, 4.0 * 0.002, 0.25 * 4.0 * 0.002);
}

/////////////////////////////////////////////////
TEST(Response, FixedAndAutoExposure)
{
  const Intrinsics k = Camera(16, 16, 1.0);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 100, 100, 100, 1.0f, rgb, z);
  Water clear;
  clear.cameraDepth = 1.0;
  std::mt19937 rng(1);
  std::vector<std::uint8_t> out;

  Response fixed;
  fixed.gain = 2.0;
  MakeUnderwater(rgb, z, rays, clear, fixed, rng, out);
  EXPECT_EQ(out[0], LinearToSrgb(2.0 * SrgbToLinear(100)));

  Response automatic;
  automatic.exposure = Response::Exposure::kAuto;
  automatic.gain = 16.0;
  const auto stats = MakeUnderwater(rgb, z, rays, clear, automatic, rng, out);
  EXPECT_NEAR(stats.gain, 0.18 / SrgbToLinear(100), 1e-9);
  EXPECT_NEAR(SrgbToLinear(out[0]), 0.18, 0.005);

  // A black frame: the gain stops at its limit.
  Uniform(k, 1, 1, 1, 1.0f, rgb, z);
  EXPECT_DOUBLE_EQ(MakeUnderwater(rgb, z, rays, clear, automatic, rng, out)
                   .gain, 16.0);
}

/////////////////////////////////////////////////
// Grey world makes a neutral patch neutral, to 1 %.
TEST(Response, GreyWorldNeutralises)
{
  const Intrinsics k = Camera(16, 16, 1.0);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 128, 128, 128, 3.0f, rgb, z);
  const Water w = CoastalAt(3.0);
  Response r;
  r.whiteBalance = Response::WhiteBalance::kGreyWorld;
  std::mt19937 rng(1);
  std::vector<std::uint8_t> plain, balanced;
  MakeUnderwater(rgb, z, rays, w, Response{}, rng, plain);
  MakeUnderwater(rgb, z, rays, w, r, rng, balanced);
  const std::size_t i = 8 * k.width + 8;
  EXPECT_GT(std::abs(int(plain[3 * i]) - int(plain[3 * i + 1])), 10)
      << "the water tints the plain image";
  const double rr = SrgbToLinear(balanced[3 * i]);
  const double gg = SrgbToLinear(balanced[3 * i + 1]);
  const double bb = SrgbToLinear(balanced[3 * i + 2]);
  EXPECT_NEAR(rr / gg, 1.0, 0.02);
  EXPECT_NEAR(bb / gg, 1.0, 0.02);
}

/////////////////////////////////////////////////
// Noise: unbiased, with the variance asked for, to 5 %, over many pixels.
TEST(Response, NoiseStatistics)
{
  const Intrinsics k = Camera(200, 100, 1.0);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 188, 188, 188, 1.0f, rgb, z);
  Water clear;
  clear.cameraDepth = 1.0;
  Response r;
  r.shotNoise = 0.002;
  r.readNoise = 0.01;
  std::mt19937 rng(7);
  const double s = SrgbToLinear(188);
  const double sigma = std::sqrt(r.shotNoise * s + r.readNoise * r.readNoise);
  double sum = 0, sum2 = 0;
  std::size_t count = 0;
  std::vector<std::uint8_t> out;
  for (int f = 0; f < 50; ++f)
  {
    MakeUnderwater(rgb, z, rays, clear, r, rng, out);
    for (std::size_t i = 0; i < out.size(); i += 3)
    {
      const double v = SrgbToLinear(out[i]);
      sum += v;
      sum2 += v * v;
      ++count;
    }
  }
  const double mean = sum / count;
  const double var = sum2 / count - mean * mean;
  EXPECT_NEAR(mean, s, 0.002);
  // 8-bit quantisation adds about step^2 / 12 of variance.
  const double step = (SrgbToLinear(189) - SrgbToLinear(187)) / 2;
  EXPECT_NEAR(var, sigma * sigma + step * step / 12, 0.05 * sigma * sigma);
}

/////////////////////////////////////////////////
// Throughput: 640x480 within a 15 Hz frame on this machine.
TEST(Throughput, VgaAt15Hz)
{
  const Intrinsics k = Camera(640, 480, 1.2);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 120, 140, 160, 6.0f, rgb, z);
  const Water w = CoastalAt(5.0);
  std::mt19937 rng(1);
  std::vector<std::uint8_t> out;
  MakeUnderwater(rgb, z, rays, w, Response{}, rng, out);
  const int frames = 10;
  const auto t0 = std::chrono::steady_clock::now();
  for (int f = 0; f < frames; ++f)
    MakeUnderwater(rgb, z, rays, w, Response{}, rng, out);
  const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count() / frames;
  std::cout << "640x480: " << ms << " ms per frame\n";
  EXPECT_LT(ms, 1000.0 / 15.0);
}

/////////////////////////////////////////////////
// Every plugin parameter is in the reference docs and the README.
TEST(Docs, EveryParameterDocumented)
{
  const auto read = [](const std::string &_path)
  {
    std::ifstream in(_path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
  };
  const std::string reference = read(DOCS_REFERENCE);
  const std::string readme = read(PACKAGE_README);
  ASSERT_FALSE(reference.empty());
  ASSERT_FALSE(readme.empty());
  for (const std::string tag : {"output_topic", "exposure", "max_gain",
                                "white_balance", "shot_noise", "read_noise",
                                "seed"})
  {
    EXPECT_NE(reference.find("`<" + tag + ">`"), std::string::npos) << tag;
    EXPECT_NE(readme.find("`<" + tag + ">`"), std::string::npos) << tag;
  }
}
