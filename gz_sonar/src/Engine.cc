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
#include "gz/maritime/sonar/Engine.hh"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>


namespace gz::maritime::sonar
{
namespace
{
/// \brief sinc^2(x) = 0.5 at this x.
constexpr double kHalfPowerX = 0.442946470689452;

/// \brief First sidelobe of sinc^2, dB.
constexpr double kSincSidelobeDb = -13.2614;

/// \brief Weights below this are dropped: -50 dB.
constexpr double kPatternFloor = 1e-5;

/// \brief Never divide by less than this cosine: incidence just short of
/// grazing.
constexpr double kMinCos = 1e-3;

/// \brief A footprint spreads over at most this many samples: more smears
/// a target's edge, where rays graze it, far in range.
constexpr double kMaxFootprintSamples = 8.0;

double Sinc2(double _x)
{
  if (std::abs(_x) < 1e-9)
    return 1.0;
  const double s = std::sin(M_PI * _x) / (M_PI * _x);
  return s * s;
}

/// \brief Worker threads kept for the life of an engine. For(n, fn) runs
/// fn(i) for every i in [0, n) on the workers and the calling thread, in
/// chunks taken in turn, and returns when all are done. fn must only touch
/// what belongs to i; then which thread runs which i does not change the
/// result.
class Pool
{
  /// \brief A pool that runs on _threads threads, the caller's included.
  public: explicit Pool(std::size_t _threads)
  {
    for (std::size_t w = 1; w < _threads; ++w)
      this->workers.emplace_back([this]() { this->Work(); });
  }

  public: ~Pool()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->stop = true;
    }
    this->wake.notify_all();
    for (auto &t : this->workers)
      t.join();
  }

  public: Pool(const Pool &) = delete;
  public: Pool &operator=(const Pool &) = delete;

  /// \brief Threads a job runs on, the caller's included.
  public: std::size_t Size() const
  {
    return this->workers.size() + 1;
  }

  public: template<typename Fn>
  void For(std::size_t _n, Fn &_fn)
  {
    if (this->workers.empty() || _n <= 1)
    {
      for (std::size_t i = 0; i < _n; ++i)
        _fn(i);
      return;
    }
    // Chunks several to a thread, so a thread that meets a busy part of the
    // fan does not hold the others up.
    this->job = [](void *_ctx, std::size_t _i) {
      (*static_cast<Fn *>(_ctx))(_i);
    };
    this->ctx = &_fn;
    this->count = _n;
    this->chunk = std::max<std::size_t>(1, _n / (8 * this->Size()));
    this->next.store(0, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->busy = this->workers.size();
      ++this->generation;
    }
    this->wake.notify_all();
    this->Drain();
    std::unique_lock<std::mutex> lock(this->mutex);
    this->done.wait(lock, [this]() { return this->busy == 0; });
  }

  /// \brief Take chunks of the current job until none is left.
  private: void Drain()
  {
    while (true)
    {
      const std::size_t first =
        this->next.fetch_add(this->chunk, std::memory_order_relaxed);
      if (first >= this->count)
        return;
      const std::size_t last = std::min(this->count, first + this->chunk);
      for (std::size_t i = first; i < last; ++i)
        this->job(this->ctx, i);
    }
  }

  /// \brief A worker: sleep until a job or the stop, help with the job.
  private: void Work()
  {
    std::uint64_t seen = 0;
    while (true)
    {
      {
        std::unique_lock<std::mutex> lock(this->mutex);
        this->wake.wait(lock, [&]() {
          return this->stop || this->generation != seen;
        });
        if (this->stop)
          return;
        seen = this->generation;
      }
      this->Drain();
      std::lock_guard<std::mutex> lock(this->mutex);
      if (--this->busy == 0)
        this->done.notify_one();
    }
  }

  private: std::vector<std::thread> workers;
  private: std::mutex mutex;
  private: std::condition_variable wake;
  private: std::condition_variable done;
  private: bool stop{false};
  private: std::uint64_t generation{0};
  private: std::size_t busy{0};

  // The current job, written before the generation changes and read only
  // while it runs.
  private: void (*job)(void *, std::size_t){nullptr};
  private: void *ctx{nullptr};
  private: std::size_t count{0};
  private: std::size_t chunk{1};
  private: std::atomic<std::size_t> next{0};
};
}  // namespace

double HorizontalPattern(const SonarConfig &_config, double _offset)
{
  const double theta0 = 0.5 * _config.beamWidth / kHalfPowerX;
  const double x = _offset / theta0;
  const double w = Sinc2(x);
  if (std::abs(x) <= 1.0)
    return w;
  return w * std::pow(10.0,
    (_config.sidelobeLevelDb - kSincSidelobeDb) / 10.0);
}

double VerticalPattern(const SonarConfig &_config, double _elevation)
{
  const double u = 2.0 * _elevation / _config.verticalFov;
  return std::exp(-M_LN2 * u * u);
}

/// \brief What every ping of one engine shares: its threads, the layout of
/// the azimuth columns, the beam weights, and buffers reused from ping to
/// ping.
struct Engine::Implementation
{
  explicit Implementation(std::size_t _threads)
    : pool(_threads)
  {
  }

  /// \brief One ping at a time: the buffers below are the ping's.
  std::mutex mutex;

  /// \brief Azimuth columns: cols of them, colStep wide, from -azHalf.
  double azHalf{0.0};
  double colStep{0.0};
  std::size_t cols{0};

  /// \brief The output's range and azimuth axes.
  std::vector<double> ranges;
  std::vector<double> azimuths;

  /// \brief The columns a beam gathers from, in column order, and their
  /// weights: beam j's are taps[tapStart[j]] to taps[tapStart[j + 1]].
  struct Tap
  {
    std::size_t col;
    double weight;
  };
  std::vector<Tap> taps;
  std::vector<std::size_t> tapStart;

  /// \brief Column buffers, all zero between pings except where
  /// [colLo, colHi] says a column was written, which the ping clears.
  std::vector<double> colData;
  std::vector<double> colBest;
  std::vector<std::uint32_t> colLabel;
  std::vector<long> colLo;
  std::vector<long> colHi;

  /// \brief The strongest contribution each output cell has seen.
  std::vector<double> best;

  /// \brief The hits sorted into columns.
  std::vector<std::size_t> start;
  std::vector<std::size_t> colOf;
  std::vector<std::size_t> order;
  std::vector<std::size_t> next;

  /// \brief Last, so its threads are joined before anything they use goes.
  Pool pool;
};

Engine::Engine(const SonarConfig &_config, const Water &_water)
  : config(_config), water(_water),
    alpha(AbsorptionDbPerMeter(_config.frequency, _water))
{
  const SonarConfig &c = this->config;
  const std::size_t threads = c.threads > 0 ? c.threads :
    std::min<std::size_t>(kDefaultMaxEngineThreads,
      std::max(1u, std::thread::hardware_concurrency()));
  this->impl = std::make_unique<Implementation>(threads);
  Implementation &d = *this->impl;

  for (std::size_t k = 0; k < c.samples; ++k)
    d.ranges.push_back(c.SampleRange(k));
  for (std::size_t j = 0; j < c.beams; ++j)
    d.azimuths.push_back(c.BeamAzimuth(j));

  const double beamSpacing = c.horizontalFov / static_cast<double>(c.beams);

  // How far, in beams, the horizontal pattern stays above the floor.
  double reach = c.beamWidth;
  while (reach < c.horizontalFov + 4.0 * c.beamWidth)
  {
    // Sidelobe peaks fall as 1 / x^2; stop where even a peak is negligible.
    const double theta0 = 0.5 * c.beamWidth / kHalfPowerX;
    const double x = reach / theta0;
    const double peak = 1.0 / (M_PI * M_PI * x * x) *
      std::pow(10.0, (c.sidelobeLevelDb - kSincSidelobeDb) / 10.0);
    if (peak < kPatternFloor)
      break;
    reach += c.beamWidth;
  }
  const auto beamReach = static_cast<long>(std::ceil(reach / beamSpacing));

  // Rays that share an azimuth share their beam weights, so the energy is
  // first summed into fine azimuth columns (the ray fan's spacing) and each
  // beam then gathers from the columns its pattern reaches. Same sums, far
  // fewer of them.
  d.colStep = std::min(beamSpacing, c.beamWidth) /
    static_cast<double>(std::max<std::size_t>(c.raysPerBeam, 1));
  d.azHalf = 0.5 * c.horizontalFov + 2.0 * c.beamWidth;
  d.cols = static_cast<std::size_t>(std::ceil(2.0 * d.azHalf / d.colStep));

  // The weights depend only on the configuration, so they are worked out
  // once here rather than on every ping.
  d.tapStart.push_back(0);
  for (std::size_t j = 0; j < c.beams; ++j)
  {
    const double beamAz = d.azimuths[j];
    const double reachAz = static_cast<double>(beamReach + 1) * beamSpacing;
    const long first = std::max<long>(0, static_cast<long>(
      std::floor((beamAz - reachAz + d.azHalf) / d.colStep)));
    const long last = std::min<long>(static_cast<long>(d.cols) - 1,
      static_cast<long>(std::ceil((beamAz + reachAz + d.azHalf) / d.colStep)));
    for (long col = first; col <= last; ++col)
    {
      const double az = -d.azHalf +
        (static_cast<double>(col) + 0.5) * d.colStep;
      const double w = HorizontalPattern(c, az - beamAz);
      if (w >= kPatternFloor)
        d.taps.push_back({static_cast<std::size_t>(col), w});
    }
    d.tapStart.push_back(d.taps.size());
  }

  d.colData.assign(d.cols * c.samples, 0.0);
  d.colBest.assign(d.cols * c.samples, 0.0);
  d.colLabel.assign(d.cols * c.samples, 0);
  d.colLo.assign(d.cols, static_cast<long>(c.samples));
  d.colHi.assign(d.cols, -1);
  d.best.assign(c.beams * c.samples, 0.0);
}

Engine::~Engine() = default;

Engine::Engine(Engine &&) noexcept = default;

Engine &Engine::operator=(Engine &&) noexcept = default;

const SonarConfig &Engine::Config() const
{
  return this->config;
}

const Water &Engine::WaterProperties() const
{
  return this->water;
}

double Engine::Absorption() const
{
  return this->alpha;
}

std::size_t Engine::Threads() const
{
  return this->impl->pool.Size();
}

BeamIntensity Engine::Ping(const std::vector<RayHit> &_hits,
  std::uint64_t _ping) const
{
  const SonarConfig &c = this->config;
  Implementation &d = *this->impl;
  std::lock_guard<std::mutex> lock(d.mutex);

  BeamIntensity out;
  out.beams = c.beams;
  out.samples = c.samples;
  out.data.assign(c.beams * c.samples, 0.0);
  out.labels.assign(c.beams * c.samples, 0);
  out.ranges = d.ranges;
  out.azimuths = d.azimuths;

  const double assumed = c.AssumedSoundSpeed(this->water.soundSpeed);
  const double rangeScale = assumed / this->water.soundSpeed;
  const double dr = c.SampleSpacing();
  const double pulseRange = 0.5 * assumed * c.pulseLength;
  const std::size_t cols = d.cols;
  const double azHalf = d.azHalf;
  const double colStep = d.colStep;

  // Sort the hits into their columns, keeping their order within each, so
  // every thread owns whole columns and then whole beams. Nothing is shared
  // between threads, and the sums run in the same order for any number of
  // threads: the result is the same bits whatever the machine.
  d.start.assign(cols + 1, 0);
  d.colOf.resize(_hits.size());
  for (std::size_t i = 0; i < _hits.size(); ++i)
  {
    const RayHit &h = _hits[i];
    if (!h.hit || h.range <= 0.0 || std::abs(h.ray.azimuth) >= azHalf)
    {
      d.colOf[i] = cols;
      continue;
    }
    d.colOf[i] = std::min(cols - 1,
      static_cast<std::size_t>((h.ray.azimuth + azHalf) / colStep));
    ++d.start[d.colOf[i] + 1];
  }
  for (std::size_t col = 0; col < cols; ++col)
    d.start[col + 1] += d.start[col];
  d.order.resize(d.start[cols]);
  d.next.assign(d.start.begin(), d.start.end() - 1);
  for (std::size_t i = 0; i < _hits.size(); ++i)
  {
    if (d.colOf[i] < cols)
      d.order[d.next[d.colOf[i]]++] = i;
  }

  // Each column: the sonar equation for every hit in it, spread over the
  // range samples its footprint and the pulse cover.
  auto column = [&](std::size_t _col) {
    for (std::size_t n = d.start[_col]; n < d.start[_col + 1]; ++n)
    {
      const RayHit &h = _hits[d.order[n]];
      const double r = h.range;
      const double apparent = r * rangeScale;
      if (apparent < c.rangeMin || apparent >= c.rangeMax)
        continue;

      const double vertical = VerticalPattern(c, h.ray.elevation);
      if (vertical < kPatternFloor)
        continue;

      // Sonar equation for the patch this ray stands for.
      const double cosInc = std::max(std::cos(h.incidence), kMinCos);
      const double level = vertical * Backscatter(h.properties, h.incidence) *
        h.ray.solidAngle / (r * r * cosInc) *
        std::pow(10.0, -2.0 * this->alpha * r / 10.0);
      if (level <= 0.0)
        continue;

      // The patch's footprint on a slanted surface spans a range interval,
      // and the pulse smears it further; spread the energy uniformly over
      // the wider of the two (at least one sample, at most a few).
      // From the sampler, which knows the neighbors; otherwise estimated
      // from the incidence, and capped since a lone ray cannot tell a
      // slanted plane from the edge of a target.
      double footprint = h.footprint * rangeScale;
      if (footprint <= 0.0)
      {
        const double tanInc = std::min(std::tan(h.incidence), kMaxSlant);
        footprint = std::min(apparent * h.ray.spacing * tanInc,
          kMaxFootprintSamples * dr);
      }
      const double width = std::max({footprint, pulseRange, dr});
      const double lo = apparent - 0.5 * width;
      const double hi = apparent + 0.5 * width;
      const long k0 = std::max<long>(0,
        static_cast<long>(std::floor((lo - c.rangeMin) / dr)));
      const long k1 = std::min<long>(static_cast<long>(c.samples) - 1,
        static_cast<long>(std::floor((hi - c.rangeMin) / dr)));

      d.colLo[_col] = std::min(d.colLo[_col], k0);
      d.colHi[_col] = std::max(d.colHi[_col], k1);
      for (long k = k0; k <= k1; ++k)
      {
        const double cellLo = c.rangeMin + static_cast<double>(k) * dr;
        const double overlap = std::min(hi, cellLo + dr) -
          std::max(lo, cellLo);
        if (overlap <= 0.0)
          continue;
        const double add = level * overlap / width;
        const std::size_t idx = _col * c.samples + static_cast<std::size_t>(k);
        d.colData[idx] += add;
        if (add > d.colBest[idx])
        {
          d.colBest[idx] = add;
          d.colLabel[idx] = h.label;
        }
      }
    }
  };
  d.pool.For(cols, column);

  // Each beam gathers from the columns its pattern reaches, in column order.
  auto beam = [&](std::size_t _j) {
    double *best = d.best.data() + _j * c.samples;
    std::fill(best, best + c.samples, 0.0);
    for (std::size_t t = d.tapStart[_j]; t < d.tapStart[_j + 1]; ++t)
    {
      const std::size_t col = d.taps[t].col;
      if (d.colHi[col] < d.colLo[col])
        continue;
      const double w = d.taps[t].weight;
      for (long k = d.colLo[col]; k <= d.colHi[col]; ++k)
      {
        const auto uk = static_cast<std::size_t>(k);
        const std::size_t from = col * c.samples + uk;
        const std::size_t idx = _j * c.samples + uk;
        out.data[idx] += w * d.colData[from];
        if (w * d.colBest[from] > best[uk])
        {
          best[uk] = w * d.colBest[from];
          out.labels[idx] = d.colLabel[from];
        }
      }
    }
  };
  d.pool.For(c.beams, beam);

  // Leave the column buffers zero for the next ping, clearing only what
  // this one wrote.
  auto clear = [&](std::size_t _col) {
    if (d.colHi[_col] >= d.colLo[_col])
    {
      const std::size_t first = _col * c.samples +
        static_cast<std::size_t>(d.colLo[_col]);
      const std::size_t last = _col * c.samples +
        static_cast<std::size_t>(d.colHi[_col]) + 1;
      std::fill(d.colData.begin() + first, d.colData.begin() + last, 0.0);
      std::fill(d.colBest.begin() + first, d.colBest.begin() + last, 0.0);
      std::fill(d.colLabel.begin() + first, d.colLabel.begin() + last, 0u);
    }
    d.colLo[_col] = static_cast<long>(c.samples);
    d.colHi[_col] = -1;
  };
  d.pool.For(cols, clear);

  // Speckle is drawn in the readout, on the echo and the noise together.
  (void)_ping;
  return out;
}
}  // namespace gz::maritime::sonar
