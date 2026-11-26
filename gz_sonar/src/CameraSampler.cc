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
#include "gz/maritime/sonar/CameraSampler.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

namespace gz::maritime::sonar
{
namespace
{
/// \brief Incidence for a hit whose surface normal cannot be estimated.
constexpr double kUnknownIncidence = M_PI / 4.0;
}  // namespace

namespace
{
/// \brief Neighboring depths within this fraction are one surface.
constexpr double kContinuous = 0.02;

/// \brief How far 1/z may depart from a plane over the pixels around a ray,
/// relative to 1/z there, for them to count as one plane: the read is then
/// off by about this fraction of the depth at most, 3 mm at 30 m, a tenth
/// of an ECHO sample. Float depth rounds to 6e-8 of its value, so rounding
/// alone moves a second difference by a few 1e-7; a depth buffer whose
/// precision falls off with distance can leave a few 1e-5 at the far end of
/// the window, still well inside. A step between two surfaces fails unless
/// it is shallower than this, when blending it is harmless.
constexpr double kPlanar = 1e-4;

//////////////////////////////////////////////////
math::Vector3d Rotate(double _yaw, const math::Vector3d &_v)
{
  const double c = std::cos(_yaw);
  const double s = std::sin(_yaw);
  return {c * _v.X() - s * _v.Y(), s * _v.X() + c * _v.Y(), _v.Z()};
}
}  // namespace

namespace
{
/// \brief A fan's extent and spacing, as the rays RayFan builds have them.
struct Extent
{
  /// \brief Most negative azimuth, rad.
  double azMin{0.0};
  /// \brief Steps between columns and between rows, rad; a fan of one
  /// column takes the row step, and one of one row has a row step of 0.
  double azStep{0.0};
  double elStep{0.0};
  /// \brief Largest elevation of any ray, either way, rad.
  double elMax{0.0};
  /// \brief Azimuth the cameras cover: the rays and half a step each side.
  double span{0.0};
};

//////////////////////////////////////////////////
/// \param[in] _g The fan's grid, one CheckRayFan accepts.
/// \return Its extent.
Extent FanExtent(const RayGrid &_g)
{
  Extent e;
  const auto columns = static_cast<std::size_t>(_g.columns);
  e.azMin = std::min(0.0, _g.Azimuth(0));
  const double azMax = std::max(0.0, _g.Azimuth(columns - 1));
  e.elMax = std::max(std::abs(_g.Elevation(0)),
    std::abs(_g.Elevation(_g.rows - 1)));
  // With one row there is no step between rows: 0, which adds no margin
  // and takes no part in the pixel size, rather than a sentinel that would.
  e.elStep = _g.rows > 1 ? _g.Elevation(1) - _g.Elevation(0) : 0.0;
  e.azStep = columns > 1 ? _g.Azimuth(1) - _g.Azimuth(0) : e.elStep;
  e.span = azMax - e.azMin + e.azStep;
  return e;
}
}  // namespace

//////////////////////////////////////////////////
std::optional<CameraLayout> PlanCameraLayout(const SonarConfig &_config,
  std::string &_error, double _maxCameraWidth)
{
  if (!CheckRayFan(_config, _error))
    return std::nullopt;
  if (!(_maxCameraWidth > 0.0) || !(_maxCameraWidth < M_PI))
  {
    _error = "a camera's field of view must be between 0 and 180 degrees";
    return std::nullopt;
  }
  const RayGrid g = RayFanGrid(_config);
  const Extent e = FanExtent(g);
  const double azMin = e.azMin, azStep = e.azStep, elStep = e.elStep;
  const double elMax = e.elMax, span = e.span;
  const double step = g.rows > 1 ? std::min(azStep, elStep) : azStep;
  if (!(step > 0.0) || !std::isfinite(step))
  {
    _error = "the fan has one ray; it needs more than one azimuth column or "
      "elevation row";
    return std::nullopt;
  }

  CameraLayout layout;
  const double cameras = std::max(1.0, std::ceil(span / _maxCameraWidth));
  if (cameras > static_cast<double>(kMaxCameras))
  {
    std::ostringstream out;
    out << "the fan needs " << cameras << " cameras to cover "
        << span * 180.0 / M_PI << " degrees in azimuth; at most "
        << kMaxCameras;
    _error = out.str();
    return std::nullopt;
  }
  const auto count = static_cast<std::size_t>(cameras);
  const double half = 0.5 * span / static_cast<double>(count);
  // One ray step of margin above and below the outermost rows, where there
  // are rows to give a step, and never past what a pinhole can look at: with
  // one row the step would be the whole vertical extent, and with a few
  // wide ones the margin alone could pass 90 degrees.
  const double top = std::min(elMax + elStep, kMaxFanElevation);
  layout.focal = 1.0 / step;
  const double width = std::ceil(2.0 * layout.focal * std::tan(half)) + 4.0;
  const double height = std::ceil(
    2.0 * layout.focal * std::tan(top) / std::cos(half)) + 4.0;
  if (!(width <= kMaxCameraSide) || !(height <= kMaxCameraSide) ||
      !(cameras * width * height <= kMaxCameraPixels))
  {
    std::ostringstream out;
    out << "the fan needs " << count << " camera(s) of " << width << " x "
        << height << " pixels, one pixel per " << step << " rad of ray "
        << "spacing over " << 2.0 * half * 180.0 / M_PI << " by "
        << 2.0 * top * 180.0 / M_PI << " degrees; at most " << kMaxCameraSide
        << " pixels a side and " << kMaxCameraPixels << " pixels in all. "
        << "Fewer rays per beam or elevation rays, or a narrower vertical "
        << "field of view or elevation span, make it smaller";
    _error = out.str();
    return std::nullopt;
  }
  layout.width = static_cast<unsigned int>(width);
  layout.height = static_cast<unsigned int>(height);
  layout.hfov = 2.0 * std::atan(0.5 * layout.width / layout.focal);
  layout.farClip = 1.5 * _config.rangeMax;
  // As a float, which is what the camera takes.
  if (!std::isfinite(static_cast<float>(layout.farClip)) ||
      !(layout.farClip > 2.0 * layout.nearClip))
  {
    std::ostringstream out;
    out << "the far clip plane, 1.5 times the maximum range of "
        << _config.rangeMax << " m, must be finite and beyond "
        << 2.0 * layout.nearClip << " m";
    _error = out.str();
    return std::nullopt;
  }
  for (std::size_t c = 0; c < count; ++c)
  {
    layout.yaws.push_back(azMin - 0.5 * azStep +
      (static_cast<double>(c) + 0.5) * span / static_cast<double>(count));
  }
  return layout;
}

//////////////////////////////////////////////////
CameraSampler::CameraSampler(const SonarConfig &_config,
  double _maxCameraWidth)
  : config(_config)
{
  // Plan first: a fan that is refused is not built, so a bad configuration
  // costs nothing here and leaves a sampler with no cameras and no rays.
  std::string error;
  const auto plan = PlanCameraLayout(_config, error, _maxCameraWidth);
  if (!plan)
    return;
  this->layout = *plan;
  this->rays = RayFan(_config);
  this->elevationCount = RayFanElevationCount(_config);

  const Extent e = FanExtent(RayFanGrid(_config));
  const double azMin = e.azMin, azStep = e.azStep, span = e.span;
  const std::size_t count = this->layout.yaws.size();

  this->lookup.resize(this->rays.size());
  for (std::size_t n = 0; n < this->rays.size(); ++n)
  {
    const Ray &r = this->rays[n];
    const auto c = std::min(count - 1, static_cast<std::size_t>(
      (r.azimuth - (azMin - 0.5 * azStep)) / span *
      static_cast<double>(count)));
    const math::Vector3d d = Rotate(-this->layout.yaws[c], r.Direction());
    Lookup &l = this->lookup[n];
    l.camera = c;
    l.u = 0.5 * this->layout.width - this->layout.focal * d.Y() / d.X();
    l.v = 0.5 * this->layout.height - this->layout.focal * d.Z() / d.X();
    l.rangePerDepth = d.Length() / d.X();
  }
}

//////////////////////////////////////////////////
bool CameraSampler::Planar(const float *_depth, long _u0, long _v0,
  const std::uint8_t *_labels, unsigned int _channels) const
{
  const auto w = static_cast<long>(this->layout.width);
  const auto h = static_cast<long>(this->layout.height);
  // The four pixels must show one surface, when labels are drawn.
  if (_labels)
  {
    const std::uint8_t l00 = _labels[(_v0 * w + _u0) * _channels];
    if (_labels[(_v0 * w + _u0 + 1) * _channels] != l00 ||
        _labels[((_v0 + 1) * w + _u0) * _channels] != l00 ||
        _labels[((_v0 + 1) * w + _u0 + 1) * _channels] != l00)
    {
      return false;
    }
  }
  // 1/z over the quad and one pixel beyond it on each axis (on the other
  // side at the image's edge). A plane has no twist over the quad and no
  // second difference along either axis. The twist alone would pass a step
  // along a row or column of pixels; the second differences catch it.
  const long ue = _u0 > 0 ? _u0 - 1 : _u0 + 2;
  const long ve = _v0 > 0 ? _v0 - 1 : _v0 + 2;
  if (ue < 0 || ue >= w || ve < 0 || ve >= h)
    return false;
  auto inv = [&](long _u, long _v) {
    const double z = _depth[_v * w + _u];
    return std::isfinite(z) && z > 0.0 ? 1.0 / z : 0.0;
  };
  const double q00 = inv(_u0, _v0);
  const double q10 = inv(_u0 + 1, _v0);
  const double q01 = inv(_u0, _v0 + 1);
  const double q11 = inv(_u0 + 1, _v0 + 1);
  const double qu0 = inv(ue, _v0);
  const double qu1 = inv(ue, _v0 + 1);
  const double q0v = inv(_u0, ve);
  const double q1v = inv(_u0 + 1, ve);
  if (qu0 <= 0.0 || qu1 <= 0.0 || q0v <= 0.0 || q1v <= 0.0)
    return false;
  // Second difference along u through the extra column: its pixels sit at
  // ue, u0, u0 + 1 in some order, evenly spaced.
  auto second = [](long _e, long _a, double _qe, double _q0, double _q1) {
    return _e < _a ? _qe - 2.0 * _q0 + _q1 : _q0 - 2.0 * _q1 + _qe;
  };
  const double tol = kPlanar * std::max({q00, q10, q01, q11});
  return std::abs(q00 + q11 - q10 - q01) <= tol &&
    std::abs(second(ue, _u0, qu0, q00, q10)) <= tol &&
    std::abs(second(ue, _u0, qu1, q01, q11)) <= tol &&
    std::abs(second(ve, _v0, q0v, q00, q01)) <= tol &&
    std::abs(second(ve, _v0, q1v, q10, q11)) <= tol;
}

//////////////////////////////////////////////////
const CameraLayout &CameraSampler::Layout() const
{
  return this->layout;
}

//////////////////////////////////////////////////
const std::vector<Ray> &CameraSampler::Rays() const
{
  return this->rays;
}

//////////////////////////////////////////////////
Ray CameraSampler::PixelRay(std::size_t _camera, unsigned int _u,
  unsigned int _v) const
{
  const math::Vector3d inCamera(1.0,
    -(static_cast<double>(_u) + 0.5 - 0.5 * this->layout.width) /
      this->layout.focal,
    -(static_cast<double>(_v) + 0.5 - 0.5 * this->layout.height) /
      this->layout.focal);
  const math::Vector3d d =
    Rotate(this->layout.yaws[_camera], inCamera).Normalized();
  Ray r;
  r.azimuth = std::atan2(d.Y(), d.X());
  r.elevation = std::asin(std::clamp(d.Z(), -1.0, 1.0));
  return r;
}

//////////////////////////////////////////////////
std::vector<RayHit> CameraSampler::Sample(
  const std::vector<const float *> &_depth,
  const std::vector<const std::uint8_t *> &_labels, unsigned int _channels,
  const std::vector<AcousticProperties> &_properties) const
{
  const auto w = static_cast<long>(this->layout.width);
  const auto h = static_cast<long>(this->layout.height);
  std::vector<RayHit> hits(this->rays.size());

  for (std::size_t n = 0; n < this->rays.size(); ++n)
  {
    RayHit &hit = hits[n];
    hit.ray = this->rays[n];
    const Lookup &l = this->lookup[n];
    if (l.camera >= _depth.size() || !_depth[l.camera])
      continue;
    const float *dd = _depth[l.camera];

    const long pu = std::clamp(static_cast<long>(std::floor(l.u)), 0L, w - 1);
    const long pv = std::clamp(static_cast<long>(std::floor(l.v)), 0L, h - 1);
    double z = dd[pv * w + pu];

    // 1/z is linear in screen space on a plane; interpolate it across the
    // four pixels around the ray, unless they straddle a silhouette.
    const double x = l.u - 0.5;
    const double y = l.v - 0.5;
    const long u0 = std::clamp(static_cast<long>(std::floor(x)), 0L, w - 2);
    const long v0 = std::clamp(static_cast<long>(std::floor(y)), 0L, h - 2);
    const double fx = std::clamp(x - static_cast<double>(u0), 0.0, 1.0);
    const double fy = std::clamp(y - static_cast<double>(v0), 0.0, 1.0);
    const double z00 = dd[v0 * w + u0];
    const double z10 = dd[v0 * w + u0 + 1];
    const double z01 = dd[(v0 + 1) * w + u0];
    const double z11 = dd[(v0 + 1) * w + u0 + 1];
    const double zMin = std::min({z00, z10, z01, z11});
    const double zMax = std::max({z00, z10, z01, z11});
    bool blend = std::isfinite(zMax) && zMin > 0.0;
    // Close depths are one surface; that keeps curved surfaces seen near
    // head on. On a plane seen at a grazing angle depth changes faster than
    // that from pixel to pixel, but 1/z is still affine in screen space.
    if (blend && zMax - zMin >= kContinuous * zMin)
      blend = this->Planar(dd, u0, v0, _labels.size() > l.camera ?
        _labels[l.camera] : nullptr, _channels);
    if (blend)
    {
      z = 1.0 / ((1 - fx) * (1 - fy) / z00 + fx * (1 - fy) / z10 +
        (1 - fx) * fy / z01 + fx * fy / z11);
    }

    const double range = z * l.rangePerDepth;
    if (!std::isfinite(z) || z <= 0.0 || range >= this->layout.farClip)
      continue;
    hit.hit = true;
    hit.range = range;
    std::uint32_t label = 0;
    if (l.camera < _labels.size() && _labels[l.camera])
      label = _labels[l.camera][(pv * w + pu) * _channels];
    hit.label = label;
    if (!_properties.empty())
    {
      hit.properties = label < _properties.size() ?
        _properties[label] : _properties[0];
    }
  }

  // Normals from neighboring hits on the ray grid, in the sensor frame: for
  // each direction, the one-sided difference with the smaller range jump,
  // so a silhouette does not tilt the surface toward the background when
  // one neighbor is on the surface. With neither on the same surface (an
  // object one ray wide) the normal is unknown and the hit keeps
  // kUnknownIncidence.
  const std::size_t ne = this->elevationCount;
  const std::size_t cols = hits.size() / std::max<std::size_t>(ne, 1);
  std::vector<math::Vector3d> points(hits.size());
  for (std::size_t n = 0; n < hits.size(); ++n)
    points[n] = hits[n].ray.Direction() * hits[n].range;
  // A neighbor counts only on the same surface: a neighbor on the wall
  // behind a pole one ray wide would tilt the pole's normal to grazing.
  auto ok = [&](std::size_t _n, long _i, long _k) {
    return _i >= 0 && _k >= 0 && _i < static_cast<long>(cols) &&
      _k < static_cast<long>(ne) && SameSurface(hits[_n],
        hits[static_cast<std::size_t>(_i) * ne + static_cast<std::size_t>(_k)]);
  };
  for (std::size_t i = 0; i < cols; ++i)
  {
    for (std::size_t k = 0; k < ne; ++k)
    {
      const std::size_t n = i * ne + k;
      if (!hits[n].hit)
        continue;
      // Without two neighbors on the same surface the normal is unknown (a
      // rope, a pole end on, a target one ray wide). Head on would give it
      // the strongest return and a full glint; 45 degrees gives a plain
      // diffuse return and no glint, unless a normal is found below.
      hits[n].incidence = kUnknownIncidence;
      auto pick = [&](long _di, long _dk, math::Vector3d &_out) {
        const long ia = static_cast<long>(i) + _di;
        const long ka = static_cast<long>(k) + _dk;
        const long ib = static_cast<long>(i) - _di;
        const long kb = static_cast<long>(k) - _dk;
        const bool fa = ok(n, ia, ka);
        const bool fb = ok(n, ib, kb);
        if (!fa && !fb)
          return false;
        const std::size_t na = fa ? static_cast<std::size_t>(ia) * ne +
          static_cast<std::size_t>(ka) : 0;
        const std::size_t nb = fb ? static_cast<std::size_t>(ib) * ne +
          static_cast<std::size_t>(kb) : 0;
        const double ea = fa ? std::abs(hits[na].range - hits[n].range) : 1e9;
        const double eb = fb ? std::abs(hits[nb].range - hits[n].range) : 1e9;
        _out = ea <= eb ? points[na] - points[n] : points[n] - points[nb];
        return true;
      };
      math::Vector3d a, b;
      if (!pick(1, 0, a) || !pick(0, 1, b))
        continue;
      math::Vector3d normal = a.Cross(b);
      if (normal.Length() < 1e-12)
        continue;
      normal.Normalize();
      const math::Vector3d dir = hits[n].ray.Direction();
      hits[n].incidence = std::acos(std::min(1.0, std::abs(normal.Dot(dir))));
    }
  }

  AssignFootprints(hits, ne);
  return hits;
}
}  // namespace gz::maritime::sonar
