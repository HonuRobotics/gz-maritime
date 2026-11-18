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
#ifndef GZ_SIM_LENS_DROPS_DROPMAP_HH_
#define GZ_SIM_LENS_DROPS_DROPMAP_HH_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "gz/sim/lens_drops/Drops.hh"

namespace gz::sim::lens_drops
{
/// \brief The largest map side, in texels.
inline constexpr int kMaxMapSize = 4096;

/// \brief Blur at focus 0, in texels of a map 360 texels high.
inline constexpr double kMaxBlurPixels360 = 4.0;

/// \brief The blur of a drop map: drops on the lens are out of focus.
/// \param[in] _focus 0 blurred to 1 sharp; clamped to [0, 1].
/// \param[in] _mapHeight The map's height, texels.
/// \return The Gaussian's sigma, texels.
double BlurPixels(double _focus, int _mapHeight);

/// \brief Height of water in a drop: a spherical cap meeting the lens at a
/// contact angle.
/// \param[in] _rho Distance from the drop's centre, image heights.
/// \param[in] _radius Radius of the contact circle, image heights.
/// \param[in] _contactAngle rad.
/// \return With R = _radius / sin(_contactAngle), sqrt(R^2 - rho^2) -
/// R cos(_contactAngle) for rho < _radius, else 0; 0 if _radius or
/// _contactAngle is not positive.
double CapHeight(double _rho, double _radius, double _contactAngle);

/// \brief The drops on a lens as a map for the shader: per texel the height
/// of water, in image heights, and the wet film, in [0, 1], blurred by the
/// defocus. Built on the CPU; no renderer dependency.
class DropMap
{
  /// \brief Constructor.
  /// \param[in] _width Texels across.
  /// \param[in] _height Texels down.
  /// \throws std::invalid_argument unless both are in [1, kMaxMapSize].
  public: DropMap(int _width, int _height);

  /// \brief Texels across.
  /// \return The width.
  public: int Width() const;

  /// \brief Texels down.
  /// \return The height.
  public: int Height() const;

  /// \brief Draw the drops and trails of a lens, blurred by its focus.
  /// \param[in] _drops The lens.
  public: void Render(const Drops &_drops);

  /// \brief The map.
  /// \return Width * Height * 4 floats, row 0 at the top: height of water
  /// (image heights), wet film [0, 1], 0, 0.
  public: const std::vector<float> &Data() const;

  /// \brief How many texels hold a drop.
  /// \param[in] _threshold Height, image heights, above which a texel
  /// counts.
  /// \return The count.
  public: std::size_t Covered(double _threshold = 1e-4) const;

  /// \brief The drops as a mask.
  /// \param[out] _out Resized to Width * Height: 255 where a drop is, 0
  /// elsewhere, row 0 at the top.
  /// \param[in] _threshold As Covered.
  public: void Mask(std::vector<std::uint8_t> &_out,
                    double _threshold = 1e-4) const;

  /// \brief Blur one channel of the map in place, by three box blurs that
  /// approximate a Gaussian.
  /// \param[in] _channel 0 height, 1 wet.
  /// \param[in] _sigma Texels; nothing below 0.3.
  private: void Blur(int _channel, double _sigma);

  /// \brief Texels across.
  private: int width;

  /// \brief Texels down.
  private: int height;

  /// \brief The map, 4 floats a texel.
  private: std::vector<float> data;
};
}  // namespace gz::sim::lens_drops

#endif  // GZ_SIM_LENS_DROPS_DROPMAP_HH_
