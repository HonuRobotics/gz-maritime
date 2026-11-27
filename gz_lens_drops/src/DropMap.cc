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
#include "gz/sim/lens_drops/DropMap.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace gz::sim::lens_drops
{
//////////////////////////////////////////////////
double BlurPixels(double _focus, int _mapHeight)
{
  const double focus = std::isfinite(_focus) ? std::clamp(_focus, 0.0, 1.0)
                                             : 0.5;
  return (1.0 - focus) * kMaxBlurPixels360 * _mapHeight / 360.0;
}

//////////////////////////////////////////////////
double CapHeight(double _rho, double _radius, double _contactAngle)
{
  if (!(_radius > 0.0) || !(_contactAngle > 0.0) || !(_rho < _radius))
    return 0.0;
  const double angle = std::min(_contactAngle, kMaxContactAngle);
  const double big = _radius / std::sin(angle);
  return CapHeightOnSphere(_rho, big * big, big * std::cos(angle));
}

//////////////////////////////////////////////////
DropMap::DropMap(int _width, int _height)
  : width(_width), height(_height)
{
  if (_width < 1 || _height < 1 || _width > kMaxMapSize ||
      _height > kMaxMapSize)
  {
    throw std::invalid_argument("DropMap size " + std::to_string(_width) +
        "x" + std::to_string(_height) + " is outside [1, " +
        std::to_string(kMaxMapSize) + "]");
  }
  const std::size_t n = static_cast<std::size_t>(_width) * _height;
  this->data.assign(n * kMapChannels, 0.0f);
  this->plane.resize(n);
  this->other.resize(n);
  this->line.resize(static_cast<std::size_t>(_width));
  this->acc.resize(static_cast<std::size_t>(_width));
}

//////////////////////////////////////////////////
int DropMap::Width() const
{
  return this->width;
}

//////////////////////////////////////////////////
int DropMap::Height() const
{
  return this->height;
}

//////////////////////////////////////////////////
void DropMap::Render(const Drops &_drops)
{
  this->Render(_drops.List(), _drops.Trails(), _drops.Parameters());
}

//////////////////////////////////////////////////
void DropMap::Render(const std::vector<Drop> &_drops,
    const std::vector<Trail> &_trails, const Params &_params)
{
  std::fill(this->data.begin(), this->data.end(), 0.0f);
  const Params &p = _params;
  const double aspect = p.aspect;
  const double w = this->width;
  const double h = this->height;

  // A texel index from a coordinate, clamped as a double first, so a drop
  // far outside the image casts nothing out of range.
  const auto texel = [](double _v, int _last)
  {
    return static_cast<int>(std::clamp(_v, -1.0, static_cast<double>(_last)));
  };
  const double angle = std::min(p.contactAngle, kMaxContactAngle);

  // Drops: a cap each, rims wobbly, sliding ones stretched along their
  // motion; where two meet, the higher water shows.
  for (const Drop &d : _drops)
  {
    if (!(d.fill > 0.0) || !(d.radius > 0.0))
      continue;
    const double speed = std::hypot(d.vx, d.vy);
    const double e = 1.0 + kSlideStretch * std::min(1.0, speed / kSlideSpeed);
    const double ux = speed > 0.0 ? d.vx / speed : 0.0;
    const double uy = speed > 0.0 ? d.vy / speed : 1.0;
    // The rim is at most kWobbleReach wobbles out; stretched, e times
    // longer.
    const double reach = d.radius * (1.0 + kWobbleReach * d.wobble) * e;
    // Bounding box in texels; x in image heights is x * aspect.
    const int i0 = std::max(0, texel((d.x - reach / aspect) * w, this->width) - 1);
    const int i1 = std::min(this->width - 1,
        texel((d.x + reach / aspect) * w, this->width) + 1);
    const int j0 = std::max(0, texel((d.y - reach) * h, this->height) - 1);
    const int j1 = std::min(this->height - 1,
        texel((d.y + reach) * h, this->height) + 1);
    const double sqrtE = std::sqrt(e);
    // The rim's harmonics by angle addition, so no trigonometry per texel.
    const double sa = std::sin(d.phaseA);
    const double ca = std::cos(d.phaseA);
    const double sb = std::sin(d.phaseB);
    const double cb = std::cos(d.phaseB);
    const double sab = std::sin(d.phaseA + d.phaseB);
    const double cab = std::cos(d.phaseA + d.phaseB);
    const double maxRim = d.radius * (1.0 + kWobbleReach * d.wobble);
    const bool still = speed <= 0.0;
    // The sphere the fresh cap is cut from, as CapHeight has it; drying
    // scales its height, not its contact circle.
    const double sphere = d.radius / std::sin(angle);
    const double sphere2 = sphere * sphere;
    const double sphereBase = sphere * std::cos(angle);
    const double fill = std::min(1.0, d.fill);
    for (int j = j0; j <= j1; ++j)
    {
      const double dy = (j + 0.5) / h - d.y;
      int a0 = i0;
      int a1 = i1;
      if (still)
      {
        // A still drop is round within maxRim: only the row's chord.
        const double half2 = maxRim * maxRim - dy * dy;
        if (half2 <= 0.0)
          continue;
        const double half = std::sqrt(half2) / aspect;
        a0 = std::max(i0, texel((d.x - half) * w, this->width) - 1);
        a1 = std::min(i1, texel((d.x + half) * w, this->width) + 1);
      }
      for (int i = a0; i <= a1; ++i)
      {
        const double dx = ((i + 0.5) / w - d.x) * aspect;
        const double along = (dx * ux + dy * uy) / e;
        const double across = (-dx * uy + dy * ux) * sqrtE;
        const double dist = std::sqrt(along * along + across * across);
        if (dist >= maxRim)
          continue;
        double rim = d.radius;
        if (d.wobble > 0.0f && dist > 0.0)
        {
          // phi is the angle of (along, across).
          const double c1 = along / dist;
          const double s1 = across / dist;
          const double s2 = 2.0 * s1 * c1;
          const double c2 = c1 * c1 - s1 * s1;
          const double s3 = s1 * (3.0 - 4.0 * s1 * s1);
          const double c3 = c1 * (4.0 * c1 * c1 - 3.0);
          const double s5 = s2 * c3 + c2 * s3;
          const double c5 = c2 * c3 - s2 * s3;
          // sin(k phi + a) = sin(k phi) cos(a) + cos(k phi) sin(a)
          rim *= 1.0 + d.wobble * ((s2 * ca + c2 * sa) +
              0.6 * (s3 * cb + c3 * sb) + 0.3 * (s5 * cab + c5 * sab));
        }
        if (rim <= 0.0)
          continue;
        // The cap, its rim on the wobbly contact line, its height scaled by
        // the water left
        const double rho = dist * d.radius / rim;
        if (rho >= d.radius)
          continue;
        const double height =
            fill * CapHeightOnSphere(rho, sphere2, sphereBase);
        if (height <= 0.0)
          continue;
        float &dst = this->data[
            (static_cast<std::size_t>(j) * this->width + i) * kMapChannels];
        dst = std::max(dst, static_cast<float>(height));
      }
    }
  }

  // Trails: a film along each segment, thinning as it dries.
  const double trailLife = kTrailLife * p.dryTime;
  for (const Trail &t : _trails)
  {
    const double life = 1.0 - t.age / trailLife;
    if (life <= 0.0 || t.halfWidth <= 0.0)
      continue;
    const double ax = t.x0 * aspect;
    const double ay = t.y0;
    const double bx = t.x1 * aspect;
    const double by = t.y1;
    const double lx = bx - ax;
    const double ly = by - ay;
    const double len2 = lx * lx + ly * ly;
    const double hw = t.halfWidth;
    const int i0 = std::max(0,
        texel((std::min(ax, bx) - hw) / aspect * w, this->width) - 1);
    const int i1 = std::min(this->width - 1,
        texel((std::max(ax, bx) + hw) / aspect * w, this->width) + 1);
    const int j0 = std::max(0,
        texel((std::min(ay, by) - hw) * h, this->height) - 1);
    const int j1 = std::min(this->height - 1,
        texel((std::max(ay, by) + hw) * h, this->height) + 1);
    for (int j = j0; j <= j1; ++j)
    {
      const double py = (j + 0.5) / h;
      for (int i = i0; i <= i1; ++i)
      {
        const double px = (i + 0.5) / w * aspect;
        double s = len2 > 0.0 ? ((px - ax) * lx + (py - ay) * ly) / len2 : 0.0;
        s = std::clamp(s, 0.0, 1.0);
        const double dist = std::hypot(px - (ax + s * lx), py - (ay + s * ly));
        if (dist >= hw)
          continue;
        const double wet = kFilmWetness * life * (1.0 - dist / hw);
        float &dst = this->data[
            (static_cast<std::size_t>(j) * this->width + i) * kMapChannels + 1];
        dst = std::max(dst, static_cast<float>(wet));
      }
    }
  }

  // Out of focus.
  const double sigma = BlurPixels(p.focus, this->height);
  this->Blur(0, sigma);
  this->Blur(1, sigma + 1.0);
}

//////////////////////////////////////////////////
void DropMap::Blur(int _channel, double _sigma)
{
  if (!(_sigma >= 0.3))
    return;

  // Three box blurs whose widths approximate a Gaussian of this sigma
  // (Kutskir, "Fastest Gaussian blur"): a running sum each, so the cost does
  // not grow with the blur. Edges are clamped.
  constexpr int kBoxes = 3;
  const double ideal = std::sqrt(12.0 * _sigma * _sigma / kBoxes + 1.0);
  int lower = static_cast<int>(std::floor(ideal));
  if (lower % 2 == 0)
    --lower;
  const int upper = lower + 2;
  const double m = (12.0 * _sigma * _sigma - kBoxes * lower * lower -
      4.0 * kBoxes * lower - 3.0 * kBoxes) / (-4.0 * lower - 4.0);
  const int smaller = static_cast<int>(std::round(m));

  const int w = this->width;
  const int h = this->height;
  const std::size_t n = static_cast<std::size_t>(w) * h;

  // Nothing to blur on a dry channel: checked before any copy
  const auto channel = static_cast<std::size_t>(_channel);
  bool any = false;
  for (std::size_t k = 0; k < n && !any; ++k)
    any = this->data[k * kMapChannels + channel] != 0.0f;
  if (!any)
    return;

  // The channel as a plane, and a second plane to blur into: members,
  // sized once
  std::vector<float> &plane = this->plane;
  std::vector<float> &other = this->other;
  std::vector<float> &line = this->line;
  std::vector<float> &acc = this->acc;
  for (std::size_t k = 0; k < n; ++k)
    plane[k] = this->data[k * kMapChannels + channel];
  for (int pass = 0; pass < kBoxes; ++pass)
  {
    const int size = pass < smaller ? lower : upper;
    const int r = std::max(0, (size - 1) / 2);
    if (r == 0)
      continue;
    const float inv = 1.0f / static_cast<float>(2 * r + 1);

    // Across each row, a running sum, in place.
    for (int j = 0; j < h; ++j)
    {
      float *row = plane.data() + static_cast<std::size_t>(j) * w;
      std::copy(row, row + w, line.begin());
      float sum = 0.0f;
      for (int k = -r; k <= r; ++k)
        sum += line[static_cast<std::size_t>(std::clamp(k, 0, w - 1))];
      for (int i = 0; i < w; ++i)
      {
        row[i] = sum * inv;
        sum += line[static_cast<std::size_t>(std::min(i + r + 1, w - 1))] -
            line[static_cast<std::size_t>(std::max(i - r, 0))];
      }
    }

    // Down the columns, a running sum per column, row by row, into the
    // other plane, so memory is read in order.
    std::fill(acc.begin(), acc.end(), 0.0f);
    for (int k = -r; k <= r; ++k)
    {
      const float *row =
          plane.data() + static_cast<std::size_t>(std::clamp(k, 0, h - 1)) * w;
      for (int i = 0; i < w; ++i)
        acc[static_cast<std::size_t>(i)] += row[i];
    }
    for (int j = 0; j < h; ++j)
    {
      float *out = other.data() + static_cast<std::size_t>(j) * w;
      const float *add =
          plane.data() + static_cast<std::size_t>(std::min(j + r + 1, h - 1)) * w;
      const float *sub =
          plane.data() + static_cast<std::size_t>(std::max(j - r, 0)) * w;
      for (int i = 0; i < w; ++i)
      {
        out[i] = acc[static_cast<std::size_t>(i)] * inv;
        acc[static_cast<std::size_t>(i)] += add[i] - sub[i];
      }
    }
    plane.swap(other);
  }

  // Running sums leave float residue, around 1e-8, where there is no water:
  // flush it, so dry stays exactly dry.
  for (std::size_t k = 0; k < n; ++k)
  {
    this->data[k * kMapChannels + channel] =
        plane[k] < 1e-6f ? 0.0f : plane[k];
  }
}

//////////////////////////////////////////////////
const std::vector<float> &DropMap::Data() const
{
  return this->data;
}

//////////////////////////////////////////////////
std::size_t DropMap::Covered(double _threshold) const
{
  std::size_t n = 0;
  for (std::size_t k = 0; k < this->data.size(); k += kMapChannels)
  {
    if (this->data[k] > _threshold)
      ++n;
  }
  return n;
}

//////////////////////////////////////////////////
void DropMap::Mask(std::vector<std::uint8_t> &_out, double _threshold) const
{
  _out.resize(static_cast<std::size_t>(this->width) * this->height);
  for (std::size_t k = 0; k < _out.size(); ++k)
    _out[k] = this->data[k * kMapChannels] > _threshold ? 255 : 0;
}
}  // namespace gz::sim::lens_drops
