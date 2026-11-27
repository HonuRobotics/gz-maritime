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

#ifdef __linux__
#include <sched.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <random>
#include <system_error>
#include <thread>
#include <vector>

#include <gz/math/Helpers.hh>
#include <gz/math/Vector4.hh>

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

/// \brief The cores this process may run on: its CPU affinity, which a
/// container's cpuset limits, or the machine's.
unsigned int Cores()
{
#ifdef __linux__
  cpu_set_t set;
  if (sched_getaffinity(0, sizeof(set), &set) == 0)
    return std::max(1, CPU_COUNT(&set));
#endif
  return std::max(1u, std::thread::hardware_concurrency());
}

/// \brief Threads kept for ParallelFor, made on first use: one fewer than
/// the cores, at most 15, so a frame does not start threads of its own.
/// Several callers (several cameras) can use it at once. Never destroyed:
/// an object that makes a frame while the process exits, or a forked child
/// that exits, finds it there, rather than joining threads that are gone.
class Pool
{
  public: static Pool &Instance()
  {
    static Pool *pool = new Pool();
    return *pool;
  }

  /// \brief Queue a task.
  public: void Push(std::function<void()> _task)
  {
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      this->tasks.push_back(std::move(_task));
    }
    this->cv.notify_one();
  }

  /// \brief Run one queued task here, if there is one.
  /// \return Whether one ran.
  public: bool RunOne()
  {
    std::function<void()> task;
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      if (this->tasks.empty())
        return false;
      task = std::move(this->tasks.front());
      this->tasks.pop_front();
    }
    task();
    return true;
  }

  private: Pool()
  {
    const unsigned int cores = Cores();
    for (unsigned int i = 0; i + 1 < std::min(cores, 16u); ++i)
    {
      try
      {
        this->threads.emplace_back([this]() { this->Work(); });
      }
      catch (const std::system_error &)
      {
        // Fewer threads: the callers run the rest themselves
        break;
      }
    }
  }

  private: void Work()
  {
    for (;;)
    {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(this->mutex);
        this->cv.wait(lock, [this]() { return !this->tasks.empty(); });
        task = std::move(this->tasks.front());
        this->tasks.pop_front();
      }
      task();
    }
  }

  private: std::mutex mutex;
  private: std::condition_variable cv;
  private: std::deque<std::function<void()>> tasks;
  private: std::vector<std::thread> threads;
};

void ParallelFor(std::size_t _n, std::size_t _grain,
                 const std::function<void(std::size_t, std::size_t)> &_work)
{
  const unsigned int cap = gMaxThreads.load();
  const std::size_t cores = cap > 0 ? cap : Cores();
  const std::size_t pieces = std::min<std::size_t>(
      std::min<std::size_t>(cores, 16), _n / std::max<std::size_t>(1, _grain));
  if (pieces < 2)
  {
    _work(0, _n);
    return;
  }
  // This call's pieces still to finish, and the first exception of any,
  // rethrown once all are done
  struct Latch
  {
    std::mutex mutex;
    std::condition_variable cv;
    std::size_t left{0};
    std::exception_ptr error;
  } latch;
  latch.left = pieces;
  const auto run = [&latch, &_work](std::size_t _begin, std::size_t _end)
  {
    std::exception_ptr error;
    try
    {
      _work(_begin, _end);
    }
    catch (...)
    {
      error = std::current_exception();
    }
    const std::lock_guard<std::mutex> lock(latch.mutex);
    if (error && !latch.error)
      latch.error = error;
    if (--latch.left == 0)
      latch.cv.notify_all();
  };
  auto &pool = Pool::Instance();
  std::size_t queued = 1;
  try
  {
    for (; queued < pieces; ++queued)
    {
      const std::size_t begin = _n * queued / pieces;
      const std::size_t end = _n * (queued + 1) / pieces;
      pool.Push([&run, begin, end]() { run(begin, end); });
    }
  }
  catch (...)
  {
    // Out of memory for the queue: the pieces not queued run here, so every
    // piece has run, and nothing on this stack is still used, before this
    // returns or throws
  }
  for (std::size_t p = queued; p < pieces; ++p)
    run(_n * p / pieces, _n * (p + 1) / pieces);
  run(0, _n / pieces);
  // Help with what is queued, this call's or another's, rather than wait
  // idle: nothing waits on a pool with no thread free
  for (;;)
  {
    {
      std::unique_lock<std::mutex> lock(latch.mutex);
      if (latch.left == 0)
        break;
    }
    if (!pool.RunOne())
    {
      std::unique_lock<std::mutex> lock(latch.mutex);
      latch.cv.wait_for(lock, std::chrono::milliseconds(1),
                        [&latch]() { return latch.left == 0; });
    }
  }
  if (latch.error)
    std::rethrow_exception(latch.error);
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

namespace
{
/// \brief GaussianBlur for any pixel type with + and * by a number: an
/// Rgb, or an Rgb with a weight beside it.
template <typename T>
void RecursiveBlur(std::vector<T> &_img, std::uint32_t _w, std::uint32_t _h,
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
                        std::size_t _n, std::vector<T> &tmp)
  {
    const std::size_t m = _n + 2 * pad;
    tmp.resize(m);
    const T first = _img[_start], last = _img[_start + (_n - 1) * _stride];
    T w1 = first, w2 = w1, w3 = w1;
    for (std::size_t i = 0; i < m; ++i)
    {
      const T x = i < pad ? first : i >= pad + _n ? last :
          _img[_start + (i - pad) * _stride];
      const T y = x * B + w1 * b1 + w2 * b2 + w3 * b3;
      tmp[i] = y;
      w3 = w2; w2 = w1; w1 = y;
    }
    w1 = w2 = w3 = last;
    for (std::size_t i = m; i-- > 0;)
    {
      const T y = tmp[i] * B + w1 * b1 + w2 * b2 + w3 * b3;
      if (i >= pad && i < pad + _n)
        _img[_start + (i - pad) * _stride] = y;
      w3 = w2; w2 = w1; w1 = y;
    }
  };
  // Rows, then columns: each line reads and writes only itself.
  ParallelFor(_h, 32, [&](std::size_t _begin, std::size_t _end)
  {
    std::vector<T> tmp;
    for (std::size_t v = _begin; v < _end; ++v)
      line(v * _w, 1, _w, tmp);
  });
  ParallelFor(_w, 32, [&](std::size_t _begin, std::size_t _end)
  {
    std::vector<T> tmp;
    for (std::size_t u = _begin; u < _end; ++u)
      line(u, _w, _h, tmp);
  });
}
}  // namespace

//////////////////////////////////////////////////
void GaussianBlur(std::vector<Rgb> &_img, std::uint32_t _w, std::uint32_t _h,
                  double _sigma)
{
  RecursiveBlur(_img, _w, _h, _sigma);
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
/// \brief A blur whose sigma, in pixels, varies per pixel. Light goes from
/// one pixel to another blurred by the smaller of their two sigmas: forward
/// scatter reaches a pixel only from the water in front of what it sees, so
/// a near object neither spreads its light far onto open water or a far
/// wall beside it, nor takes in a far bright one's light from far. Each
/// pixel is divided by what the same blur gives an image of ones, so a
/// scene of one colour stays flat across a depth edge.
///
/// A few levels of sigma; a pixel's sigma falls between two, and its light
/// and its weight are shared between them. A pixel at level m takes the
/// light of narrower pixels each at its own level, and all the light at
/// level m or wider blurred at m.
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
  // The level under each pixel's sigma, and its share of the next
  const std::size_t n = _sigma.size();
  std::vector<std::uint16_t> under(n);
  std::vector<float> share(n, 0.0f);
  std::size_t top = 0;
  std::vector<char> used(levels.size(), 0);
  for (std::size_t i = 0; i < n; ++i)
  {
    std::size_t k = 0;
    while (k + 1 < levels.size() && levels[k + 1] < _sigma[i])
      ++k;
    under[i] = static_cast<std::uint16_t>(k);
    if (k + 1 < levels.size())
    {
      share[i] = static_cast<float>(
          std::clamp((_sigma[i] - levels[k]) / (levels[k + 1] - levels[k]),
                     0.0, 1.0));
    }
    top = std::max(top, k + (share[i] > 0.0f ? 1 : 0));
    used[k] = 1;
    if (share[i] > 0.0f)
      used[k + 1] = 1;
  }
  // A pixel's weight at level k, and the part of it at k or wider
  const auto at = [&](std::size_t _i, std::size_t _k)
  {
    return under[_i] == _k ? 1.0 - share[_i] :
        under[_i] + 1u == _k ? static_cast<double>(share[_i]) : 0.0;
  };
  const auto wide = [&](std::size_t _i, std::size_t _k)
  {
    return _k <= under[_i] ? 1.0 :
        under[_i] + 1u == _k ? static_cast<double>(share[_i]) : 0.0;
  };

  const std::vector<Rgb> src = _img;
  // Light (x, y, z) and its weight (w) from narrower levels so far; and the
  // sum of the weights taken
  std::vector<math::Vector4d> below(n, math::Vector4d::Zero);
  std::vector<double> weight(n, 0.0);
  std::fill(_img.begin(), _img.end(), Rgb(0, 0, 0));
  // This level's own light, and all the light at it or wider, each with
  // its weight
  std::vector<math::Vector4d> own(n), wider(n);
  const auto with = [](const Rgb &_c, double _w)
  {
    return math::Vector4d(_c.X() * _w, _c.Y() * _w, _c.Z() * _w, _w);
  };
  for (std::size_t k = 0; k <= top; ++k)
  {
    // A level no pixel is at adds no light and has no pixel to take any
    if (!used[k])
      continue;
    // Nothing is wider than the top level to take its own light
    const bool last = k == top;
    ParallelFor(n, 1 << 14, [&](std::size_t _begin, std::size_t _end)
    {
      for (std::size_t i = _begin; i < _end; ++i)
      {
        if (!last)
          own[i] = with(src[i], at(i, k));
        wider[i] = with(src[i], wide(i, k));
      }
    });
    if (k > 0)
    {
      if (!last)
        RecursiveBlur(own, _w, _h, levels[k]);
      RecursiveBlur(wider, _w, _h, levels[k]);
    }
    ParallelFor(n, 1 << 14, [&](std::size_t _begin, std::size_t _end)
    {
      for (std::size_t i = _begin; i < _end; ++i)
      {
        const double r = at(i, k);
        if (r > 0.0)
        {
          const math::Vector4d sum = below[i] + wider[i];
          _img[i] += Rgb(sum.X(), sum.Y(), sum.Z()) * r;
          weight[i] += sum.W() * r;
        }
        if (!last)
          below[i] += own[i];
      }
    });
  }
  ParallelFor(n, 1 << 14, [&](std::size_t _begin, std::size_t _end)
  {
    for (std::size_t i = _begin; i < _end; ++i)
      _img[i] = weight[i] > 1e-12 ? _img[i] / weight[i] : src[i];
  });
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
  // The veil of each pixel, kept apart where forward scatter spreads the
  // light from the objects: the veil is light scattered already, not
  // spread again
  const bool scatter = _water.cameraDepth > 0 && _optics.forwardScatter > 0.0;
  std::vector<Rgb> veils(scatter ? n : 0);
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
      Rgb out, veil;
      for (int c = 0; c < 3; ++c)
      {
        const double t = std::isfinite(w)
            ? std::exp(-_water.attenuation[c] * w) : 0.0;
        const double b = std::isfinite(w)
            ? 1.0 - std::exp(-_water.backscatter[c] * w) : 1.0;
        out[c] = lit[c] * t;
        veil[c] = _water.veil[c] * b;
      }
      if (scatter)
      {
        img[i] = out;
        veils[i] = veil;
      }
      else
      {
        img[i] = out + veil;
      }
      // Forward scatter spreads each pixel's direct light by its own path
      // through the water: an object's by its range, the sky's through the
      // surface by the way up. Open water below has no direct light, only
      // the veil, which is not spread; as the widest receiver it takes
      // each object's light by that object's range, not by the water's.
      ranges[i] = std::isfinite(w) ? w : std::numeric_limits<double>::infinity();
    }
  }
  });

  // Forward scatter: the light from the objects spread, each by its range
  // through the water.
  if (scatter)
  {
    std::vector<double> sigma(n);
    // At most 40 pixels: wider costs more levels and changes little, the
    // image being all veil by then
    for (std::size_t i = 0; i < n; ++i)
      sigma[i] = std::min(40.0, k.fx * _optics.forwardScatter * ranges[i]);
    VariableBlur(img, k.width, k.height, sigma);
    for (std::size_t i = 0; i < n; ++i)
      img[i] += veils[i];
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
