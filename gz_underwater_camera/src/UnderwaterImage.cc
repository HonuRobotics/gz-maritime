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
#include "gz/sim/underwater_camera/UnderwaterImage.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace gz::sim::underwater_camera
{
namespace
{
/// \brief The 256 sRGB values as linear light, once.
const std::array<double, 256> &SrgbTable()
{
  static const std::array<double, 256> table = []
  {
    std::array<double, 256> t{};
    for (int i = 0; i < 256; ++i)
    {
      const double c = i / 255.0;
      t[i] = c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
    }
    return t;
  }();
  return table;
}

double Luminance(const Rgb &_c)
{
  return 0.2126 * _c.X() + 0.7152 * _c.Y() + 0.0722 * _c.Z();
}
}  // namespace

//////////////////////////////////////////////////
RayTable::RayTable(const Intrinsics &_k)
  : k(_k)
{
  const std::size_t n = static_cast<std::size_t>(_k.width) * _k.height;
  this->rays.resize(n);
  this->factors.resize(n);
  for (std::uint32_t v = 0; v < _k.height; ++v)
  {
    for (std::uint32_t u = 0; u < _k.width; ++u)
    {
      // Pixel centres; image right is the camera's -y, image down its -z.
      const double x = (u + 0.5 - _k.cx) / _k.fx;
      const double y = (v + 0.5 - _k.cy) / _k.fy;
      const math::Vector3d d(1.0, -x, -y);
      const double len = d.Length();
      this->rays[v * _k.width + u] = d / len;
      this->factors[v * _k.width + u] = len;
    }
  }
}

//////////////////////////////////////////////////
double SrgbToLinear(std::uint8_t _v)
{
  return SrgbTable()[_v];
}

//////////////////////////////////////////////////
std::uint8_t LinearToSrgb(double _v)
{
  if (!(_v > 0.0))
    return 0;
  if (_v >= 1.0)
    return 255;
  const double c = _v <= 0.0031308 ? 12.92 * _v
                                   : 1.055 * std::pow(_v, 1.0 / 2.4) - 0.055;
  return static_cast<std::uint8_t>(std::lround(c * 255.0));
}

//////////////////////////////////////////////////
FrameStats MakeUnderwater(const std::vector<std::uint8_t> &_rgb,
                          const std::vector<float> &_depth,
                          const RayTable &_rays, const Water &_water,
                          const Response &_response, std::mt19937 &_rng,
                          std::vector<std::uint8_t> &_out)
{
  FrameStats stats;
  const auto &k = _rays.Camera();
  const std::size_t n = static_cast<std::size_t>(k.width) * k.height;
  _out.resize(n * 3);
  if (_rgb.size() < n * 3 || _depth.size() < n)
  {
    // A frame that does not match the table: unchanged.
    std::copy(_rgb.begin(), _rgb.begin() + std::min(_rgb.size(), n * 3),
              _out.begin());
    return stats;
  }

  const auto &lin = SrgbTable();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<Rgb> img(n);
  // Two stages: the water, only under it; then the camera's own response,
  // always, so its gain and noise do not switch off above the water, or
  // when the renderer has drawn the water already.
  stats.underwater = _water.cameraDepth > 0;
  if (!stats.underwater)
  {
    for (std::size_t i = 0; i < n; ++i)
    {
      img[i].Set(lin[_rgb[3 * i]], lin[_rgb[3 * i + 1]],
                 lin[_rgb[3 * i + 2]]);
    }
  }
  for (std::uint32_t v = 0; stats.underwater && v < k.height; ++v)
  {
    for (std::uint32_t u = 0; u < k.width; ++u)
    {
      const std::size_t i = static_cast<std::size_t>(v) * k.width + u;
      const math::Vector3d ray = _water.cameraRotation * _rays.Ray(u, v);
      // Where a climbing ray leaves the water.
      const double toSurface = ray.Z() > 1e-9
          ? _water.cameraDepth / ray.Z() : inf;
      const double z = _depth[i];
      const bool hit = std::isfinite(z) && z > 0.0;
      const double range = hit ? z * _rays.RangeFactor(u, v) : inf;
      const double w = std::min(range, toSurface);

      Rgb lit(0, 0, 0);
      if (hit)
      {
        const Rgb j(lin[_rgb[3 * i]], lin[_rgb[3 * i + 1]],
                    lin[_rgb[3 * i + 2]]);
        // The daylight that lit it: faded to its depth, if under water.
        const double objDepth = _water.cameraDepth - ray.Z() * range;
        const double d = std::max(0.0, objDepth);
        lit.Set(j.X() * std::exp(-_water.diffuseAttenuation.X() * d),
                j.Y() * std::exp(-_water.diffuseAttenuation.Y() * d),
                j.Z() * std::exp(-_water.diffuseAttenuation.Z() * d));
      }
      else if (std::isfinite(toSurface))
      {
        // Nothing hit, looking up: the sky through the surface, as the
        // renderer drew it, seen through w of water.
        lit.Set(lin[_rgb[3 * i]], lin[_rgb[3 * i + 1]], lin[_rgb[3 * i + 2]]);
      }
      Rgb out;
      for (int c = 0; c < 3; ++c)
      {
        const double t = std::isfinite(w)
            ? std::exp(-_water.attenuation[c] * w) : 0.0;
        const double b = std::isfinite(w)
            ? 1.0 - std::exp(-_water.backscatter[c] * w) : 1.0;
        out[c] = lit[c] * t + _water.veil[c] * b;
      }
      img[i] = out;
    }
  }

  // Exposure.
  double gain = _response.gain;
  if (_response.exposure == Response::Exposure::kAuto)
  {
    double mean = 0.0;
    for (const auto &p : img)
      mean += Luminance(p);
    mean /= std::max<std::size_t>(1, n);
    const double limit = std::max(1.0, _response.gain);
    gain = mean > 0 ? std::clamp(_response.autoTarget / mean, 1.0 / limit,
                                 limit)
                    : limit;
  }
  stats.gain = gain;

  // White balance: grey world, on the exposed image.
  Rgb wb(1, 1, 1);
  if (_response.whiteBalance == Response::WhiteBalance::kGreyWorld)
  {
    Rgb mean(0, 0, 0);
    for (const auto &p : img)
      mean += p;
    mean /= static_cast<double>(std::max<std::size_t>(1, n));
    const double grey = Luminance(mean);
    for (int c = 0; c < 3; ++c)
      wb[c] = mean[c] > 0 ? grey / mean[c] : 1.0;
  }
  stats.whiteBalance = wb;

  std::normal_distribution<double> unit(0.0, 1.0);
  const bool noisy = _response.shotNoise > 0 || _response.readNoise > 0;
  for (std::size_t i = 0; i < n; ++i)
  {
    for (int c = 0; c < 3; ++c)
    {
      // The noise is the sensor's, on the light it caught: the gain and
      // the white balance amplify it, as in a real camera.
      double s = img[i][c];
      if (noisy)
      {
        const double var = _response.shotNoise * std::max(0.0, s) +
                           _response.readNoise * _response.readNoise;
        s += std::sqrt(var) * unit(_rng);
      }
      _out[3 * i + c] = LinearToSrgb(s * gain * wb[c]);
    }
  }
  return stats;
}
}  // namespace gz::sim::underwater_camera
