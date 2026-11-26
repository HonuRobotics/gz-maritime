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
#include <atomic>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <random>
#include <system_error>
#include <thread>
#include <vector>

#include <gz/math/Helpers.hh>

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

/// \brief Run _work(begin, end) over [0, _n) on the machine's cores, in
/// equal pieces. Small jobs run on the caller's thread. The pieces must not
/// depend on each other: every use here writes each output element from
/// inputs no other piece writes, so the result is the one of a plain loop.
/// \param[in] _n How many items.
/// \param[in] _grain Fewest items worth a thread.
/// \param[in] _work The work on a range of items.
/// \brief SetMaxThreads's value
std::atomic<unsigned int> gMaxThreads{0};

void ParallelFor(std::size_t _n, std::size_t _grain,
                 const std::function<void(std::size_t, std::size_t)> &_work)
{
  const unsigned int cap = gMaxThreads.load();
  const std::size_t cores = cap > 0 ? cap :
      std::max(1u, std::thread::hardware_concurrency());
  const std::size_t pieces = std::min<std::size_t>(
      std::min<std::size_t>(cores, 16), _n / std::max<std::size_t>(1, _grain));
  if (pieces < 2)
  {
    _work(0, _n);
    return;
  }
  // The first exception of any piece, rethrown once every thread is joined
  std::mutex mutex;
  std::exception_ptr error;
  const auto run = [&](std::size_t _begin, std::size_t _end)
  {
    try
    {
      _work(_begin, _end);
    }
    catch (...)
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (!error)
        error = std::current_exception();
    }
  };
  std::vector<std::thread> threads;
  threads.reserve(pieces - 1);
  for (std::size_t p = 1; p < pieces; ++p)
  {
    const std::size_t begin = _n * p / pieces, end = _n * (p + 1) / pieces;
    try
    {
      threads.emplace_back(run, begin, end);
    }
    catch (const std::system_error &)
    {
      // No thread to be had: this piece here
      run(begin, end);
    }
  }
  run(0, _n / pieces);
  for (auto &t : threads)
    t.join();
  if (error)
    std::rethrow_exception(error);
}

/// \brief Standard normal numbers from a 32-bit generator, by Box-Muller:
/// the same numbers with any standard library, where
/// std::normal_distribution's are each library's own.
class NormalSampler
{
  /// \brief The next number.
  /// \param[in] _rng The generator.
  /// \return A standard normal number.
  public: double operator()(std::mt19937 &_rng)
  {
    if (this->haveSpare)
    {
      this->haveSpare = false;
      return this->spare;
    }
    // In (0, 1): never 0, so the log is finite
    const double u1 = (_rng() + 0.5) / 4294967296.0;
    const double u2 = (_rng() + 0.5) / 4294967296.0;
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double a = 2.0 * GZ_PI * u2;
    this->spare = r * std::sin(a);
    this->haveSpare = true;
    return r * std::cos(a);
  }

  /// \brief The second number of the last pair.
  private: double spare{0.0};

  /// \brief Whether spare is still to be returned.
  private: bool haveSpare{false};
};

/// \brief For each of the 255 steps between sRGB values, the linear light
/// at which LinearToSrgb's rounding goes from one value to the next, found
/// with the formula itself, and for each of 4096 slices of [0, 1] the sRGB
/// value at the slice's start. Looking a value up in them gives what the
/// formula gives, without its pow.
struct SrgbInverse
{
  std::array<double, 256> edge{};
  std::array<std::uint8_t, 4097> first{};
};

std::uint8_t LinearToSrgbExact(double _v)
{
  const double c = _v <= 0.0031308 ? 12.92 * _v
                                   : 1.055 * std::pow(_v, 1.0 / 2.4) - 0.055;
  return static_cast<std::uint8_t>(std::lround(c * 255.0));
}

const SrgbInverse &SrgbInverseTable()
{
  static const SrgbInverse table = []
  {
    SrgbInverse t;
    // edge[k]: the smallest double that the formula sends to k or more.
    t.edge[0] = 0.0;
    for (int k = 1; k < 256; ++k)
    {
      double lo = 0.0, hi = 1.0;
      for (int i = 0; i < 80 && std::nextafter(lo, hi) < hi; ++i)
      {
        const double mid = 0.5 * (lo + hi);
        if (LinearToSrgbExact(mid) >= k)
          hi = mid;
        else
          lo = mid;
      }
      t.edge[k] = hi;
    }
    for (int i = 0; i <= 4096; ++i)
    {
      const double v = i / 4096.0;
      int k = 0;
      while (k < 255 && t.edge[k + 1] <= v)
        ++k;
      t.first[i] = static_cast<std::uint8_t>(k);
    }
    return t;
  }();
  return table;
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
  // The value at the start of _v's slice, then up to the last edge at or
  // under _v: the formula's own answer (see SrgbInverse).
  const auto &t = SrgbInverseTable();
  int k = t.first[static_cast<int>(_v * 4096.0)];
  while (k < 255 && t.edge[k + 1] <= _v)
    ++k;
  return static_cast<std::uint8_t>(k);
}

//////////////////////////////////////////////////
void SetMaxThreads(unsigned int _n)
{
  gMaxThreads = _n;
}

//////////////////////////////////////////////////
void GaussianBlur(std::vector<Rgb> &_img, std::uint32_t _w, std::uint32_t _h,
                  double _sigma)
{
  if (_sigma < 0.5 || _w < 2 || _h < 2)
    return;
  // Young and van Vliet (1995): a causal and an anti-causal third-order
  // pass per axis.
  const double q = _sigma >= 2.5 ? 0.98711 * _sigma - 0.96330
                                 : 3.97156 - 4.14554 * std::sqrt(
                                       1.0 - 0.26891 * _sigma);
  const double q2 = q * q, q3 = q2 * q;
  const double b0 = 1.57825 + 2.44413 * q + 1.4281 * q2 + 0.422205 * q3;
  const double b1 = (2.44413 * q + 2.85619 * q2 + 1.26661 * q3) / b0;
  const double b2 = (-1.4281 * q2 - 1.26661 * q3) / b0;
  const double b3 = 0.422205 * q3 / b0;
  const double B = 1.0 - (b1 + b2 + b3);
  // Each line padded with 3 sigma of its edge value on both sides, so both
  // passes start from their steady state for that value: the border is
  // blurred as if the edge went on, not pulled towards the filter's start.
  const std::size_t pad = static_cast<std::size_t>(std::ceil(3.0 * _sigma));
  const auto line = [&](std::size_t _start, std::size_t _stride,
                        std::size_t _n, std::vector<Rgb> &tmp)
  {
    const std::size_t m = _n + 2 * pad;
    tmp.resize(m);
    const Rgb first = _img[_start], last = _img[_start + (_n - 1) * _stride];
    Rgb w1 = first, w2 = w1, w3 = w1;
    for (std::size_t i = 0; i < m; ++i)
    {
      const Rgb x = i < pad ? first : i >= pad + _n ? last :
          _img[_start + (i - pad) * _stride];
      const Rgb y = x * B + w1 * b1 + w2 * b2 + w3 * b3;
      tmp[i] = y;
      w3 = w2; w2 = w1; w1 = y;
    }
    w1 = w2 = w3 = last;
    for (std::size_t i = m; i-- > 0;)
    {
      const Rgb y = tmp[i] * B + w1 * b1 + w2 * b2 + w3 * b3;
      if (i >= pad && i < pad + _n)
        _img[_start + (i - pad) * _stride] = y;
      w3 = w2; w2 = w1; w1 = y;
    }
  };
  // Rows, then columns: each line reads and writes only itself.
  ParallelFor(_h, 32, [&](std::size_t _begin, std::size_t _end)
  {
    std::vector<Rgb> tmp;
    for (std::size_t v = _begin; v < _end; ++v)
      line(v * _w, 1, _w, tmp);
  });
  ParallelFor(_w, 32, [&](std::size_t _begin, std::size_t _end)
  {
    std::vector<Rgb> tmp;
    for (std::size_t u = _begin; u < _end; ++u)
      line(u, _w, _h, tmp);
  });
}

//////////////////////////////////////////////////
double FlatPortSource(double _r, double _f, double _n)
{
  const double air = std::atan(_r / _f);
  const double water = std::asin(std::sin(air) / _n);
  return _f * std::tan(water);
}

namespace
{
/// \brief A blur whose sigma, in pixels, varies per pixel: a few levels,
/// each pixel interpolating between the two around its sigma.
void VariableBlur(std::vector<Rgb> &_img, std::uint32_t _w, std::uint32_t _h,
                  const std::vector<double> &_sigma)
{
  double maxSigma = 0.0;
  for (double s : _sigma)
    maxSigma = std::max(maxSigma, s);
  if (maxSigma < 0.5)
    return;
  std::vector<double> levels{0.0};
  for (double s = 0.5; s < maxSigma * 1.25; s *= 1.25)
    levels.push_back(s);
  // The level under each pixel's sigma: it reads that level and the next.
  std::vector<std::uint16_t> under(_sigma.size());
  std::vector<char> used(levels.size(), 0);
  for (std::size_t i = 0; i < _sigma.size(); ++i)
  {
    std::size_t k = 0;
    while (k + 1 < levels.size() && levels[k + 1] < _sigma[i])
      ++k;
    under[i] = static_cast<std::uint16_t>(k);
    used[k] = 1;
  }
  // Two levels at a time, lowest first, from the image as it came: the
  // pixels between them are done, then the upper level becomes the lower.
  const std::vector<Rgb> src = _img;
  const auto blur = [&](std::size_t _k)
  {
    std::vector<Rgb> b = src;
    if (_k > 0)
      GaussianBlur(b, _w, _h, levels[_k]);
    return b;
  };
  std::vector<Rgb> lo, hi;
  std::size_t loLevel = levels.size();
  for (std::size_t k = 0; k < levels.size(); ++k)
  {
    if (!used[k])
      continue;
    if (loLevel != k)
      lo = blur(k);
    const bool top = k + 1 >= levels.size();
    if (!top)
      hi = blur(k + 1);
    ParallelFor(_img.size(), 1 << 14,
        [&](std::size_t _begin, std::size_t _end)
    {
      for (std::size_t i = _begin; i < _end; ++i)
      {
        if (under[i] != k)
          continue;
        if (top)
        {
          _img[i] = lo[i];
          continue;
        }
        const double t = (_sigma[i] - levels[k]) / (levels[k + 1] - levels[k]);
        _img[i] = lo[i] * (1.0 - t) + hi[i] * t;
      }
    });
    if (!top)
    {
      lo.swap(hi);
      loLevel = k + 1;
    }
  }
}

/// \brief A flat port's refraction: each output pixel samples the image at
/// the radius the port maps it from, bilinearly.
void FlatPort(std::vector<Rgb> &_img, const Intrinsics &_k, double _n)
{
  const std::vector<Rgb> src = _img;
  const auto at = [&](int _u, int _v)
  {
    _u = std::clamp(_u, 0, static_cast<int>(_k.width) - 1);
    _v = std::clamp(_v, 0, static_cast<int>(_k.height) - 1);
    return src[static_cast<std::size_t>(_v) * _k.width + _u];
  };
  ParallelFor(_k.height, 16, [&](std::size_t _begin, std::size_t _end)
  {
  for (std::size_t v = _begin; v < _end; ++v)
  {
    for (std::uint32_t u = 0; u < _k.width; ++u)
    {
      const double dx = (u + 0.5 - _k.cx) / _k.fx;
      const double dy = (v + 0.5 - _k.cy) / _k.fy;
      const double r = std::hypot(dx, dy);
      const double scale = r > 1e-12 ? FlatPortSource(r, 1.0, _n) / r
                                     : 1.0 / _n;
      const double x = _k.cx + dx * scale * _k.fx - 0.5;
      const double y = _k.cy + dy * scale * _k.fy - 0.5;
      const int x0 = static_cast<int>(std::floor(x));
      const int y0 = static_cast<int>(std::floor(y));
      const double fx = x - x0, fy = y - y0;
      _img[static_cast<std::size_t>(v) * _k.width + u] =
          at(x0, y0) * ((1 - fx) * (1 - fy)) + at(x0 + 1, y0) * (fx * (1 - fy)) +
          at(x0, y0 + 1) * ((1 - fx) * fy) + at(x0 + 1, y0 + 1) * (fx * fy);
    }
  }
  });
}
}  // namespace

//////////////////////////////////////////////////
FrameStats MakeUnderwater(const std::vector<std::uint8_t> &_rgb,
                          const std::vector<float> &_depth,
                          const RayTable &_rays, const Water &_water,
                          const Response &_response, std::mt19937 &_rng,
                          std::vector<std::uint8_t> &_out,
                          const Optics &_optics)
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
  std::vector<double> ranges(n, 0.0);
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
  // Each pixel on its own: rows in parallel.
  if (stats.underwater)
  ParallelFor(k.height, 16, [&](std::size_t _begin, std::size_t _end)
  {
  for (std::size_t v = _begin; v < _end; ++v)
  {
    for (std::uint32_t u = 0; u < k.width; ++u)
    {
      const std::size_t i = v * k.width + u;
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
      // Nothing hit and nothing behind: the veil, the same everywhere, is
      // not blurred, so the objects around it do not bleed into it
      ranges[i] = std::isfinite(w) ? w : 0.0;
    }
  }
  });

  // Forward scatter: a blur growing with the range through the water.
  if (stats.underwater && _optics.forwardScatter > 0.0)
  {
    std::vector<double> sigma(n);
    // At most 40 pixels: wider costs more levels and changes little, the
    // image being all veil by then
    for (std::size_t i = 0; i < n; ++i)
      sigma[i] = std::min(40.0, k.fx * _optics.forwardScatter * ranges[i]);
    VariableBlur(img, k.width, k.height, sigma);
  }
  // The port: a flat one refracts; a dome centred on the lens does not.
  // Out of the water there is air on both sides: nothing to refract.
  if (stats.underwater && _optics.port == Optics::Port::kFlat)
    FlatPort(img, k, _optics.waterIndex);

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

  const bool noisy = _response.shotNoise > 0 || _response.readNoise > 0;
  // The noise: one draw from the camera's generator per frame seeds a
  // generator per row, so the image is the same however many threads run,
  // and the normal sampler is ours, so it is the same with any standard
  // library.
  std::uint32_t rowSeed = 0;
  if (noisy)
    rowSeed = static_cast<std::uint32_t>(_rng());
  ParallelFor(k.height, 16, [&](std::size_t _begin, std::size_t _end)
  {
    for (std::size_t v = _begin; v < _end; ++v)
    {
      std::seed_seq seq{rowSeed, static_cast<std::uint32_t>(v)};
      std::mt19937 rowRng(seq);
      NormalSampler unit;
      for (std::size_t i = v * k.width; i < (v + 1) * k.width; ++i)
      {
        for (int c = 0; c < 3; ++c)
        {
          // The noise is the sensor's, on the light it caught: the gain
          // and the white balance amplify it, as in a real camera.
          double s = img[i][c];
          if (noisy)
          {
            const double var = _response.shotNoise * std::max(0.0, s) +
                               _response.readNoise * _response.readNoise;
            s += std::sqrt(var) * unit(rowRng);
          }
          _out[3 * i + c] = LinearToSrgb(s * gain * wb[c]);
        }
      }
    }
  });
  return stats;
}
}  // namespace gz::sim::underwater_camera
