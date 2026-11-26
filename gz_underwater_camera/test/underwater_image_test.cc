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
#include <thread>
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
// LinearToSrgb looks its answer up; the answer is the formula's, for a
// million values across [0, 1] and for the values right at each step.
TEST(Srgb, LookupIsTheFormula)
{
  const auto formula = [](double _v)
  {
    const double c = _v <= 0.0031308 ? 12.92 * _v
                                     : 1.055 * std::pow(_v, 1.0 / 2.4) - 0.055;
    return static_cast<std::uint8_t>(std::lround(c * 255.0));
  };
  int wrong = 0;
  for (int i = 1; i < 1000000; ++i)
  {
    // Denser near zero, where the steps are narrow.
    const double u = i / 1e6;
    for (const double v : {u, u * u * u})
      wrong += LinearToSrgb(v) != formula(v);
  }
  for (int k = 0; k < 256; ++k)
  {
    const double at = SrgbToLinear(static_cast<std::uint8_t>(k));
    double v = at;
    for (int n = 0; n < 4 && v < 1.0; ++n)
    {
      wrong += LinearToSrgb(v) != formula(v);
      v = std::nextafter(v, 2.0);
    }
    v = at;
    for (int n = 0; n < 4 && v > 0.0; ++n)
    {
      wrong += LinearToSrgb(v) != formula(v);
      v = std::nextafter(v, -1.0);
    }
  }
  EXPECT_EQ(wrong, 0);
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
// Throughput at 640x480, printed: a time depends on the machine and what
// else runs on it, so it is not a pass or fail.
TEST(Throughput, Vga)
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
  std::cout << "640x480: " << ms << " ms per frame (a 15 Hz frame is "
            << 1000.0 / 15.0 << " ms)\n";
  EXPECT_GT(ms, 0.0);
}

/////////////////////////////////////////////////
// The pipeline runs its rows on several cores. A frame large enough for
// that (1280x720, with noise, forward scatter and a flat port) is the same
// frame every time, and the same on one thread as on eight: the noise is
// drawn per row, from generators the frame's one draw seeds.
TEST(Throughput, ParallelFramesAreTheSame)
{
  const Intrinsics k = Camera(1280, 720, 1.2);
  const RayTable rays(k);
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 120, 140, 160, 6.0f, rgb, z);
  for (std::size_t i = 0; i < z.size(); ++i)
  {
    z[i] = 2.0f + 6.0f * static_cast<float>(i / k.width) / k.height;
    rgb[3 * i] = static_cast<std::uint8_t>((i * 7) & 255);
  }
  const Water w = CoastalAt(5.0);
  Response r;
  r.shotNoise = 1e-4;
  r.readNoise = 2e-3;
  Optics o;
  o.forwardScatter = 0.002;
  o.port = Optics::Port::kFlat;
  std::vector<std::uint8_t> first, again;
  std::mt19937 rng(11);
  MakeUnderwater(rgb, z, rays, w, r, rng, first, o);
  for (int n = 0; n < 3; ++n)
  {
    rng.seed(11);
    MakeUnderwater(rgb, z, rays, w, r, rng, again, o);
    EXPECT_EQ(first, again);
  }
  for (const unsigned int threads : {1u, 8u})
  {
    SetMaxThreads(threads);
    rng.seed(11);
    MakeUnderwater(rgb, z, rays, w, r, rng, again, o);
    EXPECT_EQ(first, again) << threads << " threads";
  }
  SetMaxThreads(0);

  // Two cameras at once, each on its own thread, share the pool: each gets
  // the frame it gets alone
  std::vector<std::uint8_t> outs[2];
  std::vector<std::thread> cameras;
  for (int c = 0; c < 2; ++c)
  {
    cameras.emplace_back([&, c]()
    {
      for (int n = 0; n < 5; ++n)
      {
        std::mt19937 own(11);
        MakeUnderwater(rgb, z, rays, w, r, own, outs[c], o);
        if (outs[c] != first)
          return;
      }
    });
  }
  for (auto &t : cameras)
    t.join();
  EXPECT_EQ(outs[0], first);
  EXPECT_EQ(outs[1], first);
}

/////////////////////////////////////////////////
// The borders blurred as if the edge went on: an image with detail up to
// its edges, blurred, against the same image padded far out with its edge
// values, blurred, and cropped back.
TEST(Optics, BlurBordersAsIfTheEdgeWentOn)
{
  const std::uint32_t w = 61, h = 47;
  const auto value = [](std::uint32_t _u, std::uint32_t _v)
  {
    return Rgb(0.5 + 0.4 * std::sin(0.37 * _u + 0.11 * _v),
               (_u * 7 + _v * 3) % 11 / 11.0, _u < 4 || _v > 42 ? 1.0 : 0.1);
  };
  for (const double sigma : {1.5, 4.0, 9.0})
  {
    std::vector<Rgb> img(w * h);
    for (std::uint32_t v = 0; v < h; ++v)
      for (std::uint32_t u = 0; u < w; ++u)
        img[v * w + u] = value(u, v);
    const std::uint32_t pad = static_cast<std::uint32_t>(12 * sigma);
    const std::uint32_t W = w + 2 * pad, H = h + 2 * pad;
    std::vector<Rgb> big(W * H);
    for (std::uint32_t v = 0; v < H; ++v)
    {
      for (std::uint32_t u = 0; u < W; ++u)
      {
        const auto cu = std::clamp<int>(int(u) - int(pad), 0, int(w) - 1);
        const auto cv = std::clamp<int>(int(v) - int(pad), 0, int(h) - 1);
        big[v * W + u] = value(cu, cv);
      }
    }
    GaussianBlur(img, w, h, sigma);
    GaussianBlur(big, W, H, sigma);
    double worst = 0.0;
    for (std::uint32_t v = 0; v < h; ++v)
    {
      for (std::uint32_t u = 0; u < w; ++u)
      {
        worst = std::max(worst, (img[v * w + u] -
                                 big[(v + pad) * W + u + pad]).Length());
      }
    }
    std::cout << "sigma " << sigma << ": worst border difference " << worst
              << "\n";
    EXPECT_LT(worst, 1e-3) << "sigma " << sigma;
  }
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
                                "seed", "forward_scatter", "port",
                                "water_index"})
  {
    EXPECT_NE(reference.find("`<" + tag + ">`"), std::string::npos) << tag;
    EXPECT_NE(readme.find("`<" + tag + ">`"), std::string::npos) << tag;
  }
}

/////////////////////////////////////////////////
// Forward scatter: a black-to-white edge in clear water, at 2, 4 and 8 m,
// blurs as a Gaussian of sigma fx * forward_scatter * range: its 10-90 %
// width is 2.563 sigma, within 10 %, after the 0.8 px a sharp edge between
// two pixels already measures (added in quadrature). With none, no blur.
TEST(Optics, ForwardScatterGrowsWithRange)
{
  const Intrinsics k = Camera(320, 60, 1.0);
  const RayTable rays(k);
  Water clear;
  clear.cameraDepth = 2.0;
  Optics optics;
  optics.forwardScatter = 0.003;
  for (const double r : {2.0, 4.0, 8.0})
  {
    std::vector<std::uint8_t> rgb;
    std::vector<float> z;
    Uniform(k, 0, 0, 0, static_cast<float>(r), rgb, z);
    for (std::uint32_t v = 0; v < k.height; ++v)
      for (std::uint32_t u = k.width / 2; u < k.width; ++u)
        for (int c = 0; c < 3; ++c)
          rgb[3 * (v * k.width + u) + c] = 255;
    std::mt19937 rng(1);
    std::vector<std::uint8_t> out;
    MakeUnderwater(rgb, z, rays, clear, Response{}, rng, out, optics);
    // The range at the edge, on the centre row.
    const std::uint32_t v = k.height / 2;
    const double range = r * rays.RangeFactor(k.width / 2, v);
    const double sigma = k.fx * optics.forwardScatter * range;
    double u10 = 0, u90 = 0;
    for (std::uint32_t u = 1; u < k.width; ++u)
    {
      const double a = SrgbToLinear(out[3 * (v * k.width + u - 1) + 1]);
      const double b = SrgbToLinear(out[3 * (v * k.width + u) + 1]);
      if (a < 0.1 && b >= 0.1)
        u10 = u - 1 + (0.1 - a) / (b - a);
      if (a < 0.9 && b >= 0.9)
        u90 = u - 1 + (0.9 - a) / (b - a);
    }
    const double expected = std::hypot(2.563 * sigma, 0.8);
    std::cout << "range " << range << ": sigma " << sigma << " px, 10-90 "
              << u90 - u10 << " px, expected " << expected << "\n";
    EXPECT_NEAR(u90 - u10, expected, 0.1 * expected) << r;
  }
}

/////////////////////////////////////////////////
// Open water beside an object stays the veil: pixels that hit nothing are
// not blurred, so the object does not bleed a halo into them.
TEST(Optics, NoHaloInOpenWater)
{
  const Intrinsics k = Camera(200, 40, 1.0);
  const RayTable rays(k);
  Water w = CoastalAt(5.0);
  w.cameraRotation = math::Quaterniond::Identity;
  std::vector<std::uint8_t> rgb;
  std::vector<float> z;
  Uniform(k, 255, 255, 255, 3.0f, rgb, z);
  // The right half hits nothing, looking level: the veil
  for (std::uint32_t v = 0; v < k.height; ++v)
    for (std::uint32_t u = k.width / 2; u < k.width; ++u)
      z[v * k.width + u] = std::numeric_limits<float>::infinity();
  Optics optics;
  optics.forwardScatter = 0.02;
  std::mt19937 rng(1);
  std::vector<std::uint8_t> blurred, sharp;
  MakeUnderwater(rgb, z, rays, w, Response{}, rng, blurred, optics);
  MakeUnderwater(rgb, z, rays, w, Response{}, rng, sharp);
  const std::uint32_t v = k.height / 2;
  for (std::uint32_t u = k.width / 2; u < k.width; ++u)
  {
    for (int c = 0; c < 3; ++c)
    {
      const std::size_t i = 3 * (v * k.width + u) + c;
      ASSERT_EQ(blurred[i], sharp[i]) << "u " << u;
    }
  }
}

/////////////////////////////////////////////////
// A flat port: a point at radius r_src in the rendered image appears at the
// radius the refraction model gives, within half a pixel; a dome port
// changes nothing.
TEST(Optics, FlatPortRefracts)
{
  const Intrinsics k = Camera(401, 401, 1.4);
  const RayTable rays(k);
  Water clear;
  clear.cameraDepth = 2.0;
  for (const double rOut : {40.0, 100.0, 160.0})
  {
    // Where the port takes the point from.
    const double rSrc = FlatPortSource(rOut, k.fx, 1.333);
    std::vector<std::uint8_t> rgb;
    std::vector<float> z;
    Uniform(k, 0, 0, 0, 3.0f, rgb, z);
    // A 3x3 white square centred on (cx + rSrc, cy).
    const int cu = static_cast<int>(std::lround(k.cx - 0.5 + rSrc));
    const int cv = static_cast<int>(std::lround(k.cy - 0.5));
    for (int dv = -1; dv <= 1; ++dv)
      for (int du = -1; du <= 1; ++du)
        for (int c = 0; c < 3; ++c)
          rgb[3 * ((cv + dv) * k.width + cu + du) + c] = 255;
    const double srcCentre = cu + 0.5 - k.cx;
    const double expected = [&]
    {
      // Invert the port's map at the square's actual centre.
      double lo = 0, hi = k.width;
      for (int i = 0; i < 60; ++i)
      {
        const double mid = 0.5 * (lo + hi);
        (FlatPortSource(mid, k.fx, 1.333) < srcCentre ? lo : hi) = mid;
      }
      return 0.5 * (lo + hi);
    }();
    Optics flat;
    flat.port = Optics::Port::kFlat;
    std::mt19937 rng(1);
    std::vector<std::uint8_t> out;
    MakeUnderwater(rgb, z, rays, clear, Response{}, rng, out, flat);
    double sum = 0, sumU = 0;
    for (std::uint32_t v = 0; v < k.height; ++v)
      for (std::uint32_t u = 0; u < k.width; ++u)
      {
        const double val = SrgbToLinear(out[3 * (v * k.width + u)]);
        sum += val;
        sumU += val * (u + 0.5);
      }
    const double measured = sumU / sum - k.cx;
    std::cout << "flat port: source radius " << srcCentre << " px appears at "
              << measured << ", model " << expected << "\n";
    EXPECT_NEAR(measured, expected, 0.5);
    EXPECT_GT(expected, srcCentre) << "a flat port magnifies";

    Optics dome;
    dome.port = Optics::Port::kDome;
    MakeUnderwater(rgb, z, rays, clear, Response{}, rng, out, dome);
    EXPECT_EQ(out, rgb) << "a dome centred on the lens does not refract";
  }
}
