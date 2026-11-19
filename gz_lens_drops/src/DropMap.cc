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
  const double big = _radius / std::sin(_contactAngle);
  const double h = std::sqrt(std::max(0.0, big * big - _rho * _rho)) -
      big * std::cos(_contactAngle);
  return std::max(0.0, h);
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
  this->data.assign(static_cast<std::size_t>(_width) * _height * 4, 0.0f);
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
  std::fill(this->data.begin(), this->data.end(), 0.0f);
  const Params &p = _drops.Parameters();
  const double aspect = p.aspect;
  const double w = this->width;
  const double h = this->height;

  // Drops: a cap each, rims wobbly, sliding ones stretched along their
  // motion; where two meet, the higher water shows.
  for (const Drop &d : _drops.List())
  {
    const double shown = ShownRadius(d, p.dryTime);
    if (shown <= 0.0)
      continue;
    const double speed = std::hypot(d.vx, d.vy);
    const double e = 1.0 + 0.35 * std::min(1.0, speed / kSlideSpeed);
    const double ux = speed > 0.0 ? d.vx / speed : 0.0;
    const double uy = speed > 0.0 ? d.vy / speed : 1.0;
    // The rim is at most 1.9 wobbles out; stretched, e times longer.
    const double reach = d.radius * (1.0 + 1.9 * d.wobble) * e;
    // Bounding box in texels; x in image heights is x * aspect.
    const int i0 = std::max(0, static_cast<int>((d.x - reach / aspect) * w) - 1);
    const int i1 = std::min(this->width - 1,
        static_cast<int>((d.x + reach / aspect) * w) + 1);
    const int j0 = std::max(0, static_cast<int>((d.y - reach) * h) - 1);
    const int j1 = std::min(this->height - 1,
        static_cast<int>((d.y + reach) * h) + 1);
    const double sqrtE = std::sqrt(e);
    // The rim's harmonics by angle addition, so no trigonometry per texel.
    const double sa = std::sin(d.phaseA);
    const double ca = std::cos(d.phaseA);
    const double sb = std::sin(d.phaseB);
    const double cb = std::cos(d.phaseB);
    const double sab = std::sin(d.phaseA + d.phaseB);
    const double cab = std::cos(d.phaseA + d.phaseB);
    const double maxRim = d.radius * (1.0 + 1.9 * d.wobble);
    const bool still = speed <= 0.0;
    // The sphere the cap is cut from, as CapHeight has it.
    const double sphere = shown / std::sin(p.contactAngle);
    const double sphere2 = sphere * sphere;
    const double sphereBase = sphere * std::cos(p.contactAngle);
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
        a0 = std::max(i0, static_cast<int>((d.x - half) * w) - 1);
        a1 = std::min(i1, static_cast<int>((d.x + half) * w) + 1);
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
        // The cap of the drop as it dries: its contact circle shrinks with
        // the shown radius, scaled from the fresh rim. CapHeight, with the
        // sphere worked out once per drop.
        const double rho = dist * shown / rim;
        if (rho >= shown)
          continue;
        const double height = std::sqrt(sphere2 - rho * rho) - sphereBase;
        if (height <= 0.0)
          continue;
        float &dst = this->data[(static_cast<std::size_t>(j) * this->width + i) * 4];
        dst = std::max(dst, static_cast<float>(height));
      }
    }
  }

  // Trails: a film along each segment, thinning as it dries.
  const double filmLife = 0.5 * p.dryTime;
  for (const Trail &t : _drops.Trails())
  {
    const double life = 1.0 - t.age / filmLife;
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
    const int i0 = std::max(0, static_cast<int>((std::min(ax, bx) - hw) / aspect * w) - 1);
    const int i1 = std::min(this->width - 1,
        static_cast<int>((std::max(ax, bx) + hw) / aspect * w) + 1);
    const int j0 = std::max(0, static_cast<int>((std::min(ay, by) - hw) * h) - 1);
    const int j1 = std::min(this->height - 1,
        static_cast<int>((std::max(ay, by) + hw) * h) + 1);
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
        const double wet = 0.6 * life * (1.0 - dist / hw);
        float &dst = this->data[(static_cast<std::size_t>(j) * this->width + i) * 4 + 1];
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

  // The channel as a plane, and a second plane to blur into.
  std::vector<float> plane(n);
  std::vector<float> other(n);
  bool any = false;
  for (std::size_t k = 0; k < n; ++k)
  {
    plane[k] = this->data[k * 4 + static_cast<std::size_t>(_channel)];
    any = any || plane[k] != 0.0f;
  }
  if (!any)
    return;

  std::vector<float> line(static_cast<std::size_t>(w));
  std::vector<float> acc(static_cast<std::size_t>(w));
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
    this->data[k * 4 + static_cast<std::size_t>(_channel)] =
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
  for (std::size_t k = 0; k < this->data.size(); k += 4)
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
    _out[k] = this->data[k * 4] > _threshold ? 255 : 0;
}
}  // namespace gz::sim::lens_drops
