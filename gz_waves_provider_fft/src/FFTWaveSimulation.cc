/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/FFTWaveSimulation.hh"

#include <algorithm>
#include <cmath>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>  // GZ_PI

#include "gz/sim/waves/Wavefield.hh"

#include "ehukai/All.h"

namespace gz::sim::waves
{

namespace
{
/// Forward-difference step [s] for the lazy particle-velocity computation.
constexpr double kVelDt = 0.05;

/// Row-major float matrix view of Ehukai's spatial buffers (Ehukai stores
/// row-major float; our grids are column-major double).
using RowMatF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                              Eigen::RowMajor>;

//////////////////////////////////////////////////
/// Returns the integer log2 of n if n is a positive power of two; -1 otherwise.
int Log2Pow2(std::size_t _n)
{
  if (_n == 0 || (_n & (_n - 1)) != 0) return -1;
  int r = 0;
  while ((static_cast<std::size_t>(1u) << r) < _n) ++r;
  return r;
}

//////////////////////////////////////////////////
/// Smallest power of two >= n (>= 1).
std::size_t CeilPow2(std::size_t _n)
{
  std::size_t p = 1;
  while (p < _n) p <<= 1;
  return p;
}

//////////////////////////////////////////////////
// ---- Human-readable names for the Ehukai model enums (logging) --------
const char *SpectrumName(ehukai::SpectrumType _t)
{
  switch (_t)
  {
    case ehukai::kPiersonMoskowitzSpectrum: return "pms";
    case ehukai::kJONSWAPSpectrum:          return "jonswap";
    case ehukai::kTMASpectrum:              return "tma";
    default:                                     return "?";
  }
}

//////////////////////////////////////////////////
const char *DispersionName(ehukai::DispersionType _t)
{
  switch (_t)
  {
    case ehukai::kDeepDispersion:        return "deep";
    case ehukai::kFiniteDepthDispersion: return "finite";
    case ehukai::kCapillaryDispersion:   return "capillary";
    default:                                  return "?";
  }
}

//////////////////////////////////////////////////
const char *SpreadingName(ehukai::DirectionalSpreadingType _t)
{
  switch (_t)
  {
    case ehukai::kPosCosThetaSqrDirectionalSpreading: return "poscos2";
    case ehukai::kMitsuyasuDirectionalSpreading:      return "mitsuyasu";
    case ehukai::kHasselmannDirectionalSpreading:     return "hasselmann";
    case ehukai::kDonelanBannerDirectionalSpreading:  return "donelanbanner";
    default:                                               return "?";
  }
}

//////////////////////////////////////////////////
const char *FilterName(ehukai::FilterType _t)
{
  switch (_t)
  {
    case ehukai::kNullFilter:                   return "none";
    case ehukai::kSmoothInvertibleBandPassFilter: return "bandpass";
    default:                                         return "?";
  }
}

//////////////////////////////////////////////////
// ---- <spectrum>/<spreading>/<dispersion> SDF strings -> Ehukai enums ---
// Names match the *Name() helpers above and the SDF tag values. Return false on
// an unrecognised value so the caller can warn and keep the Ehukai default.
bool SpectrumFromString(const std::string &_s, ehukai::SpectrumType &_out)
{
  if (_s == "pms" || _s == "pm") _out = ehukai::kPiersonMoskowitzSpectrum;
  else if (_s == "jonswap")     _out = ehukai::kJONSWAPSpectrum;
  else if (_s == "tma")         _out = ehukai::kTMASpectrum;
  else return false;
  return true;
}

//////////////////////////////////////////////////
bool DispersionFromString(const std::string &_s,
                          ehukai::DispersionType &_out)
{
  if (_s == "deep")           _out = ehukai::kDeepDispersion;
  else if (_s == "finite" || _s == "finite_depth")
                             _out = ehukai::kFiniteDepthDispersion;
  else if (_s == "capillary") _out = ehukai::kCapillaryDispersion;
  else return false;
  return true;
}

//////////////////////////////////////////////////
bool SpreadingFromString(const std::string &_s,
                         ehukai::DirectionalSpreadingType &_out)
{
  if (_s == "poscos2" || _s == "poscossqr")
    _out = ehukai::kPosCosThetaSqrDirectionalSpreading;
  else if (_s == "mitsuyasu")
    _out = ehukai::kMitsuyasuDirectionalSpreading;
  else if (_s == "hasselmann")
    _out = ehukai::kHasselmannDirectionalSpreading;
  else if (_s == "donelanbanner" || _s == "donelan")
    _out = ehukai::kDonelanBannerDirectionalSpreading;
  else return false;
  return true;
}

//////////////////////////////////////////////////
// Map the SDF spectrum selectors and numeric knobs onto `ep`. Unknown selector
// values warn and leave the Ehukai default in place. The numeric knobs default
// (via WaveParameters) to Ehukai's own defaults, so an SDF that sets none of
// them reproduces the stock Horvath "good ocean" config.
void ApplyEhukaiParams(ehukai::Parametersf &_ep, const WaveParameters &_p)
{
  if (!SpectrumFromString(_p.spectrum, _ep.spectrum.type))
    gzerr << "[FFTWaveSimulation] ignoring unknown <spectrum>='"
          << _p.spectrum << "' (want pms|jonswap|tma)" << '\n';
  if (!DispersionFromString(_p.dispersion, _ep.dispersion.type))
    gzerr << "[FFTWaveSimulation] ignoring unknown <dispersion>='"
          << _p.dispersion << "' (want deep|finite|capillary)" << '\n';
  if (!SpreadingFromString(_p.spreading, _ep.directionalSpreading.type))
    gzerr << "[FFTWaveSimulation] ignoring unknown <spreading>='"
          << _p.spreading
          << "' (want poscos2|mitsuyasu|hasselmann|donelanbanner)"
          << '\n';

  _ep.gravity       = static_cast<float>(_p.gravity);
  _ep.depth         = static_cast<float>(_p.depth);
  _ep.fetch         = static_cast<float>(_p.fetch);
  _ep.directionalSpreading.swell = static_cast<float>(_p.swell);
  _ep.troughDamping = static_cast<float>(_p.troughDamping);

  // Optional spectral band-pass over wavelength: keep wavelengths within
  // [filter_min_wl, filter_max_wl] m and roll off outside over filter_soft.
  // filter_min raises the suppression floor (0 = full cut); filter_invert turns
  // the band into a notch. Enabled when a band edge is given. The amplitude
  // calibration renormalises total energy, so the filter reshapes *which*
  // wavelengths survive rather than the overall sea height.
  if (_p.filterMinWavelength > 0.0 || _p.filterMaxWavelength > 0.0)
  {
    _ep.filter.type = ehukai::kSmoothInvertibleBandPassFilter;
    _ep.filter.smallWavelength = static_cast<float>(_p.filterMinWavelength);
    if (_p.filterMaxWavelength > 0.0)
      _ep.filter.bigWavelength = static_cast<float>(_p.filterMaxWavelength);
    _ep.filter.min    = static_cast<float>(_p.filterMin);
    _ep.filter.invert = _p.filterInvert;
    // A zero soft width collapses the smoothstep edges (NaN spectrum); default
    // a positive transition from the lower cutoff when none was supplied.
    _ep.filter.softWidth = (_p.filterSoftWidth > 0.0)
        ? static_cast<float>(_p.filterSoftWidth)
        : std::max(0.25f * _ep.filter.smallWavelength, 1.0f);
  }
}

}  // namespace

//-----------------------------------------------------------------------------
// EhukaiState — pimpl that holds the vendored Horvath-spectrum library's
// per-instance state. Defined here (not in the header) so Ehukai headers
// stay out of the public include surface.
//-----------------------------------------------------------------------------
struct FFTWaveSimulation::EhukaiState
{
  ehukai::Parametersf params;
  std::unique_ptr<ehukai::InitialStatef> initial;
  std::unique_ptr<ehukai::Propagationf>   propagation;
  std::unique_ptr<ehukai::PropagatedStatef> state;
  /// Scratch state propagated at t+dt to finite-difference particle velocity;
  /// filled lazily by ParticleVelocity(), not by Update().
  std::unique_ptr<ehukai::PropagatedStatef> scratch;
};

FFTWaveSimulation::~FFTWaveSimulation() = default;

FFTWaveSimulation::FFTWaveSimulation() = default;

//////////////////////////////////////////////////
FFTWaveSimulation::FFTWaveSimulation(const WaveParameters &_p,
                                     double _tile,
                                     std::size_t _grid,
                                     std::uint32_t _seed)
{
  // The constructor's tile/grid/seed arguments override whatever the params
  // struct carries (callers and tests rely on this). Fold them into a params
  // copy and route through the single SetParameters setup path.
  WaveParameters q = _p;
  q.tileSize = _tile;
  q.gridSize = _grid;
  q.seed     = _seed;
  this->SetParameters(q);
}

//////////////////////////////////////////////////
void FFTWaveSimulation::SetParameters(const WaveParameters &_params)
{
  // Resolve <sea_state> (if set) into period/gain before configuring.
  const WaveParameters p = WithSeaState(_params);
  this->tileSize   = p.tileSize;
  this->gridSize   = p.gridSize;
  this->gain       = p.gain;
  this->tau        = p.tau;
  this->choppiness = p.choppiness;
  // Never serve velocity grids computed under the previous recipe.
  this->velTime    = -1.0;
  const std::uint32_t seed = p.seed;

  // Ehukai requires a power-of-two grid; round up if the SDF asks for
  // something else.
  if (Log2Pow2(this->gridSize) < 0)
  {
    const std::size_t adjusted = CeilPow2(this->gridSize);
    gzerr << "[FFTWaveSimulation] grid_size=" << this->gridSize
          << " is not a power of two (required by the FFT spectrum); using "
          << adjusted << '\n';
    this->gridSize = adjusted;
  }

  // PMS relation: peak omega <-> wind speed at 19.5 m. period -> omegaP -> V19.
  const double omegaP = 2.0 * GZ_PI / p.period;
  this->windSpeed = 0.879 * p.gravity / omegaP;

  const int N = static_cast<int>(this->gridSize);
  this->heightGrid = Eigen::MatrixXd::Zero(N, N);
  this->dispXGrid  = Eigen::MatrixXd::Zero(N, N);
  this->dispYGrid  = Eigen::MatrixXd::Zero(N, N);
  this->minEGrid   = Eigen::MatrixXd::Constant(N, N, 1.0);  // 1 = no foam
  this->velXGrid   = Eigen::MatrixXd::Zero(N, N);
  this->velYGrid   = Eigen::MatrixXd::Zero(N, N);
  this->velZGrid   = Eigen::MatrixXd::Zero(N, N);

  // Build the Ehukai spectral state -- the spectral engine the fft system
  // is built on. The <spectrum>/<spreading>/<dispersion> SDF selectors choose
  // the spectral models (default TMA + Hasselmann + capillary).
  this->ehukai = std::make_unique<EhukaiState>();
  auto &ep = this->ehukai->params;
  ep.resolutionPowerOfTwo = Log2Pow2(this->gridSize);
  ep.domain        = static_cast<float>(this->tileSize);
  ep.windSpeed     = static_cast<float>(this->windSpeed);
  ep.amplitudeGain = static_cast<float>(this->gain);
  ep.random.seed   = static_cast<int>(seed);

  // Map the SDF spectrum selectors + numeric/filter knobs onto Ehukai.
  ApplyEhukaiParams(ep, p);

  this->ehukai->initial =
      std::make_unique<ehukai::InitialStatef>(ep);
  this->ehukai->propagation =
      std::make_unique<ehukai::Propagationf>(ep, /*nthreads=*/-1);
  this->ehukai->state =
      std::make_unique<ehukai::PropagatedStatef>(ep);
  this->ehukai->scratch =
      std::make_unique<ehukai::PropagatedStatef>(ep);

  // --- Physics-based amplitude calibration -------------------------------
  // Ehukai's amplitudeGain only feeds its (here unused) normal computation,
  // not the height field, and its intrinsic variance does not correspond to a
  // physical sea state at our wind speeds. Measure the intrinsic RMS once (one
  // propagation past the ramp) and rescale so the significant wave height
  // follows the standard fully-developed Pierson-Moskowitz relation
  // Hs = 0.21 * V19.5^2 / g, i.e. sigma = Hs/4. The selected spectrum still
  // sets the spectral *shape*; this only fixes the overall energy.
  {
    this->ehukai->propagation->propagate(
        this->ehukai->params, *this->ehukai->initial,
        *this->ehukai->state, 10.0f);
    const int M = ep.resolution();
    const double sigmaEhukai = std::sqrt(
        Eigen::Map<const RowMatF>(this->ehukai->state->Height.cdata(),
                                  M, M).cast<double>().array()
            .square().mean());
    const double sigmaTarget =
        0.21 / (4.0 * p.gravity) * this->windSpeed * this->windSpeed;
    this->ehukaiScale =
        (sigmaEhukai > 1e-9) ? (sigmaTarget / sigmaEhukai) : 1.0;
  }

  gzmsg << "[FFTWaveSimulation] Ehukai spectrum library active "
        << "(res=" << ep.resolution() << " domain=" << ep.domain
        << "m wind=" << ep.windSpeed << "m/s seed=" << ep.random.seed
        << " spectrum=" << SpectrumName(ep.spectrum.type)
        << " dispersion=" << DispersionName(ep.dispersion.type)
        << " spreading=" << SpreadingName(ep.directionalSpreading.type)
        << " depth=" << ep.depth << "m fetch=" << ep.fetch << "km"
        << " swell=" << ep.directionalSpreading.swell
        << " troughDamp=" << ep.troughDamping
        << " filter=" << FilterName(ep.filter.type)
        << " ampCalib=" << this->ehukaiScale
        << " targetHs=" << (4.0 * 0.21 / (4.0 * p.gravity) *
                            this->windSpeed * this->windSpeed)
        << "m)" << '\n';

  this->Update(0.0);
}

//////////////////////////////////////////////////
void FFTWaveSimulation::Update(double _simTime)
{
  // Idempotent: the field at a given time is deterministic, so a repeat call
  // for the same time is a no-op. Lets the source system and any number of
  // WaveBuoyancy consumers that advance the same instance to the same tick
  // share a single recompute instead of each paying for a full propagation.
  if (_simTime == this->lastUpdateT)
    return;
  this->lastUpdateT = _simTime;

  if (!this->ehukai || !this->ehukai->propagation)
    return;  // default-constructed, not yet configured

  const int N = static_cast<int>(this->gridSize);

  this->ehukai->propagation->propagate(
      this->ehukai->params,
      *this->ehukai->initial,
      *this->ehukai->state,
      static_cast<float>(_simTime));

  // Combined output scale per Update:
  //  * ramp         — fade the field in over `tau`;
  //  * this->ehukaiScale — physics-based amplitude calibration to a PM sea state
  //                   (Ehukai's amplitudeGain doesn't scale the height);
  //  * this->gain        — the SDF <gain> user multiplier (a no-op via Ehukai's
  //                   amplitudeGain, so we apply it here to make it work).
  const double scale =
      StartupRamp(_simTime, this->tau) * this->ehukaiScale * this->gain;

  // Map+cast assignment lets Eigen vectorize the float→double conversion and
  // row→column layout swap; the scale folds into the same expression.
  this->heightGrid = Eigen::Map<const RowMatF>(
      this->ehukai->state->Height.cdata(), N, N).cast<double>() * scale;
  // WaveField2D carries the FINAL horizontal displacement, so the Tessendorf
  // choppiness multiplier folds in here (the shader applies dx/dy as-is; a
  // shader-side chopFactor would invert the Gerstner engine's baked chop).
  const double chopScale = scale * this->choppiness;
  this->dispXGrid = Eigen::Map<const RowMatF>(
      this->ehukai->state->Dx.cdata(), N, N).cast<double>() * chopScale;
  this->dispYGrid = Eigen::Map<const RowMatF>(
      this->ehukai->state->Dy.cdata(), N, N).cast<double>() * chopScale;

  // Foam: Ehukai computes MinE = -(min eigenvalue of the displacement
  // Jacobian) at its internal amplitude. At our calibrated amplitude the
  // minimum eigenvalue is 1 - scale*(MinE + 1). Jacobian() bilinear-samples
  // this; Eval::FoamMask turns values below its threshold into whitecaps. At
  // t=0 (scale=0) the surface is flat -> eigenvalue 1 -> no foam.
  this->minEGrid = (1.0 - scale *
      (Eigen::Map<const RowMatF>(this->ehukai->state->MinE.cdata(), N, N)
          .cast<double>().array() + 1.0)).matrix();

  // The particle-velocity grids are NOT refreshed here: their finite
  // difference needs a second full propagation at t+dt, which would double
  // the per-tick cost for consumers that never query velocity (the renderer
  // reads Field() only). ParticleVelocity() fills them lazily, keyed on
  // lastUpdateT.
}

//////////////////////////////////////////////////
double FFTWaveSimulation::BilinearSample(const Eigen::MatrixXd &_grid,
                                         double _x, double _y) const
{
  const int N = static_cast<int>(this->gridSize);
  const double L = this->tileSize;

  // Wrap query into [0, L). Grid cell (i, j) corresponds to world position
  // (i·L/N, j·L/N) — the IFFT output's natural layout, no centring shift.
  auto wrap = [L](double v) {
    double r = std::fmod(v, L);
    if (r < 0.0) r += L;
    return r;
  };
  const double wx = wrap(_x);
  const double wy = wrap(_y);

  // Grid index in [0, N).
  const double fi = wx / L * N;
  const double fj = wy / L * N;
  int i0 = static_cast<int>(std::floor(fi)) % N;
  int j0 = static_cast<int>(std::floor(fj)) % N;
  if (i0 < 0) i0 += N;
  if (j0 < 0) j0 += N;
  const int i1 = (i0 + 1) % N;
  const int j1 = (j0 + 1) % N;
  const double fx = fi - std::floor(fi);
  const double fy = fj - std::floor(fj);

  return _grid(i0, j0) * (1 - fx) * (1 - fy)
       + _grid(i1, j0) * fx       * (1 - fy)
       + _grid(i0, j1) * (1 - fx) * fy
       + _grid(i1, j1) * fx       * fy;
}

//////////////////////////////////////////////////
double FFTWaveSimulation::Elevation(double _x, double _y, double /*_t*/) const
{
  // Caller is expected to have called Update(t) ≤ this tick (the source
  // system does this in PreUpdate). The bilinear sample is time-free.
  return this->BilinearSample(this->heightGrid, _x, _y);
}

//////////////////////////////////////////////////
gz::math::Vector3d FFTWaveSimulation::ParticleVelocity(
  double _x, double _y, double /*_t*/) const
{
  // Particle velocity = Eulerian time derivative of the displacement field
  // (∂Dx/∂t, ∂Dy/∂t, ∂η/∂t). Ehukai exposes no velocity field and no
  // public spectral coefficients, so finite-difference a scratch propagation a
  // small step ahead — done LAZILY on the first velocity query after an
  // Update, because the extra propagation roughly doubles the engine's
  // per-tick cost and most consumers never ask for velocity. The mutable fill
  // relies on the same external serialisation as Update() (one ECM thread on
  // the server; WaterVisual's mutex in the GUI). The same amplitude scale is
  // applied to both samples, so it factors out as the wave motion's velocity
  // (the startup ramp's own d/dt is intentionally excluded — it's a
  // transient, not water motion).
  if (this->ehukai && this->ehukai->propagation &&
      this->velTime != this->lastUpdateT)
  {
    const int N = static_cast<int>(this->gridSize);
    this->ehukai->propagation->propagate(
        this->ehukai->params,
        *this->ehukai->initial,
        *this->ehukai->scratch,
        static_cast<float>(this->lastUpdateT + kVelDt));
    const double scale = StartupRamp(this->lastUpdateT, this->tau) *
                         this->ehukaiScale * this->gain;
    const double velK = scale / kVelDt;
    // Horizontal velocity uses the choppiness-scaled displacement so it
    // tracks the surface's actual motion.
    const double velKxy = velK * this->choppiness;
    this->velZGrid = velK *
        (Eigen::Map<const RowMatF>(this->ehukai->scratch->Height.cdata(), N, N)
           - Eigen::Map<const RowMatF>(
                 this->ehukai->state->Height.cdata(), N, N)).cast<double>();
    this->velXGrid = velKxy *
        (Eigen::Map<const RowMatF>(this->ehukai->scratch->Dx.cdata(), N, N)
           - Eigen::Map<const RowMatF>(
                 this->ehukai->state->Dx.cdata(), N, N)).cast<double>();
    this->velYGrid = velKxy *
        (Eigen::Map<const RowMatF>(this->ehukai->scratch->Dy.cdata(), N, N)
           - Eigen::Map<const RowMatF>(
                 this->ehukai->state->Dy.cdata(), N, N)).cast<double>();
    this->velTime = this->lastUpdateT;
  }

  // Bilinear-sample the cached velocity grids. Caller is expected to have
  // called Update(t) ≤ this tick.
  return gz::math::Vector3d(
      this->BilinearSample(this->velXGrid, _x, _y),
      this->BilinearSample(this->velYGrid, _x, _y),
      this->BilinearSample(this->velZGrid, _x, _y));
}

//////////////////////////////////////////////////
gz::math::Vector3d FFTWaveSimulation::Normal(
  double _x, double _y, double /*_t*/) const
{
  // Finite-difference normal from the height grid (a derivative-grid path via
  // FFT would be more accurate but is not yet wired up).
  const double L = this->tileSize;
  const double h = L / static_cast<double>(this->gridSize);
  const double dx =
    (this->BilinearSample(this->heightGrid, _x + h, _y) -
     this->BilinearSample(this->heightGrid, _x - h, _y)) / (2.0 * h);
  const double dy =
    (this->BilinearSample(this->heightGrid, _x, _y + h) -
     this->BilinearSample(this->heightGrid, _x, _y - h)) / (2.0 * h);
  gz::math::Vector3d n(-dx, -dy, 1.0);
  n.Normalize();
  return n;
}

//////////////////////////////////////////////////
double FFTWaveSimulation::Jacobian(double _x, double _y, double /*_t*/) const
{
  // Bilinear-sample the per-cell minimum eigenvalue of the displacement
  // Jacobian (1 = flat, < 1 → folding). Eval::FoamMask turns values below its
  // threshold into whitecap intensity, so foam appears on the pinched crests.
  return this->BilinearSample(this->minEGrid, _x, _y);
}

//////////////////////////////////////////////////
const WaveField2D *FFTWaveSimulation::Field() const
{
  // Repopulate the view on each call: SetParameters reallocates the grids
  // (Update refreshes them in place), so a view cached across a reconfigure
  // would dangle. Eigen is column-major, matching WaveField2D's documented
  // (i + j*N) layout. The renderer finite-diffs η for normals; foam is the
  // Ehukai MinE folding metric.
  this->field.n    = this->gridSize;
  this->field.tile = this->tileSize;
  this->field.dz   = this->heightGrid.data();
  this->field.dx   = this->dispXGrid.data();
  this->field.dy   = this->dispYGrid.data();
  this->field.foam = this->minEGrid.data();
  return &this->field;
}

//////////////////////////////////////////////////
/// \brief Factory used to register the FFT engine under the "fft" token (see
/// RegisterWaveEngineFactory). Returns a default-constructed engine; the caller
/// applies SetParameters.
std::shared_ptr<IWaveField> MakeFFTWaveField()
{
  return std::make_shared<FFTWaveSimulation>();
}

}  // namespace gz::sim::waves
