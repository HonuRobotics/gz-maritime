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
#ifndef GZ_SIM_UNDERWATER_CAMERA_UNDERWATERIMAGE_HH_
#define GZ_SIM_UNDERWATER_CAMERA_UNDERWATERIMAGE_HH_

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace gz::sim::underwater_camera
{
/// \brief Per colour channel: X red, Y green, Z blue.
using Rgb = math::Vector3d;

/// \brief A pinhole camera, as its CameraInfo gives it, in pixels.
struct Intrinsics
{
  /// \brief Image size.
  std::uint32_t width{0};
  std::uint32_t height{0};

  /// \brief Focal lengths.
  double fx{1.0};
  double fy{1.0};

  /// \brief Principal point.
  double cx{0.0};
  double cy{0.0};
};

/// \brief Per pixel: the ray through it, in the camera frame (x ahead, y
/// left, z up, as a Gazebo camera looks), unit length; and the factor that
/// turns the depth image's z-depth, the distance along the camera's axis,
/// into the range along that ray.
class RayTable
{
  /// \brief Build the table.
  /// \param[in] _k The camera.
  public: explicit RayTable(const Intrinsics &_k = {});

  /// \brief The camera the table was built for.
  public: const Intrinsics &Camera() const { return this->k; }

  /// \brief The unit ray through a pixel, camera frame.
  /// \param[in] _u Column.
  /// \param[in] _v Row.
  public: const math::Vector3d &Ray(std::uint32_t _u, std::uint32_t _v) const
  { return this->rays[_v * this->k.width + _u]; }

  /// \brief range = z-depth * RangeFactor.
  /// \param[in] _u Column.
  /// \param[in] _v Row.
  public: double RangeFactor(std::uint32_t _u, std::uint32_t _v) const
  { return this->factors[_v * this->k.width + _u]; }

  /// \brief The camera.
  private: Intrinsics k;

  /// \brief Unit rays, row by row.
  private: std::vector<math::Vector3d> rays;

  /// \brief z-depth to range factors, row by row.
  private: std::vector<double> factors;
};

/// \brief sRGB (8 bit) to linear light, by the standard's transfer
/// function.
/// \param[in] _v An 8-bit value.
/// \return Linear, in [0, 1].
double SrgbToLinear(std::uint8_t _v);

/// \brief Linear light to sRGB (8 bit), rounded to nearest, clamped.
/// \param[in] _v Linear.
/// \return The 8-bit value.
std::uint8_t LinearToSrgb(double _v);

/// \brief The water, as the camera sees it this frame.
struct Water
{
  /// \brief beta^D, 1/m: an object's light fades as exp(-attenuation r).
  Rgb attenuation{0, 0, 0};

  /// \brief beta^B, 1/m: the veil builds as 1 - exp(-backscatter r).
  Rgb backscatter{0, 0, 0};

  /// \brief The veil, B-infinity, at the camera's depth, in linear pixel
  /// units: 1 is a white surface lit by the surface daylight.
  Rgb veil{0, 0, 0};

  /// \brief K_d, 1/m: the daylight at depth z is exp(-K_d z) of the
  /// surface's.
  Rgb diffuseAttenuation{0, 0, 0};

  /// \brief The camera's depth below the local water surface, m. At or
  /// below zero, the camera is in air and the image passes through.
  double cameraDepth{0.0};

  /// \brief The camera's orientation in the world, so each ray's climb or
  /// dive, and where it meets the surface, is known.
  math::Quaterniond cameraRotation{math::Quaterniond::Identity};
};

/// \brief How the camera turns light into pixel values.
struct Response
{
  /// \brief Exposure: a fixed gain on linear light, or automatic.
  enum class Exposure { kFixed, kAuto };
  Exposure exposure{Exposure::kFixed};

  /// \brief The fixed gain, or the automatic gain's limit (both ways).
  double gain{1.0};

  /// \brief The mean linear luminance automatic exposure aims for.
  double autoTarget{0.18};

  /// \brief White balance: none, or grey world (each channel's mean made
  /// equal to the mean luminance).
  enum class WhiteBalance { kNone, kGreyWorld };
  WhiteBalance whiteBalance{WhiteBalance::kNone};

  /// \brief Noise on linear light, per pixel and channel: Gaussian, of
  /// variance shot * signal + read^2. Zero for none.
  double shotNoise{0.0};
  double readNoise{0.0};
};

/// \brief The water's effect on the image beyond the model: forward
/// scatter and the camera's port.
struct Optics
{
  /// \brief Forward scatter: a Gaussian blur whose width grows with range,
  /// sigma = forwardScatter * range radians (so fx * forwardScatter * range
  /// pixels). Zero for none.
  double forwardScatter{0.0};

  /// \brief The port in front of the lens.
  enum class Port { kNone, kFlat, kDome };
  Port port{Port::kNone};

  /// \brief Refractive index of the water, for a flat port.
  double waterIndex{1.333};
};

/// \brief At most how many threads the image functions use, 16 at most;
/// 0, the default, for as many as there are cores. The result does not
/// depend on it: noise is drawn per row, and every pixel's sum is the same.
/// \param[in] _n Threads.
void SetMaxThreads(unsigned int _n);

/// \brief A Gaussian blur of a linear image, in place, by a recursive
/// filter (Young and van Vliet 1995): its cost does not depend on sigma.
/// \param[in,out] _img Image, row by row.
/// \param[in] _w Width.
/// \param[in] _h Height.
/// \param[in] _sigma Standard deviation, pixels; below 0.5, no blur.
void GaussianBlur(std::vector<Rgb> &_img, std::uint32_t _w, std::uint32_t _h,
                  double _sigma);

/// \brief Where a flat port moves a pixel: the radius, from the principal
/// point, in the rendered (in-air) image that the port shows at radius _r.
/// A ray leaving the lens at angle a in air left the water at w,
/// sin a = n sin w: r_rendered = f tan(asin(sin(atan(_r / f)) / n)).
/// \param[in] _r Radius in the output image, pixels.
/// \param[in] _f Focal length, pixels.
/// \param[in] _n Refractive index of the water.
/// \return Radius in the rendered image, pixels.
double FlatPortSource(double _r, double _f, double _n);

/// \brief What one frame did, for the tests and the ground truth.
struct FrameStats
{
  /// \brief Whether the water was applied (false: camera in air).
  bool underwater{false};

  /// \brief The exposure gain used.
  double gain{1.0};

  /// \brief The white balance gains used, per channel.
  Rgb whiteBalance{1, 1, 1};
};

/// \brief Make one underwater image from one in-air render.
///
/// Per pixel and channel, on linear light, at range r along the pixel's ray
/// (from the depth image), with w the part of that range under water (up to
/// where the ray meets the surface, when it climbs) and z the depth of what
/// it hits:
///
///   lit = J exp(-K_d max(z, 0))
///   I   = lit exp(-attenuation w) + veil (1 - exp(-backscatter w))
///
/// then, under water, forward scatter (a blur growing with range) and the
/// port (a flat port's refraction remap); then the camera's response, which
/// always applies: noise on the light the sensor caught, then exposure and
/// white balance. A pixel with no
/// depth (nothing hit, or beyond the far clip) sees the open water: w is the
/// distance to the surface for a climbing ray, otherwise infinite, so it is
/// the veil. With the camera at or above the surface (a camera depth of 0,
/// also when the renderer drew the water already) only the response
/// applies: with the default response, the image comes out unchanged.
/// \param[in] _rgb The in-air image, RGB, 8 bits per channel, row by row.
/// \param[in] _depth The z-depth image, m, same size; not finite or not
/// positive for "nothing hit".
/// \param[in] _rays The camera's ray table, same size.
/// \param[in] _water The water.
/// \param[in] _response The camera's response.
/// \param[in,out] _rng Random numbers for the noise.
/// \param[out] _out The underwater image, RGB, 8 bits, same size.
/// \return What the frame did.
FrameStats MakeUnderwater(const std::vector<std::uint8_t> &_rgb,
                          const std::vector<float> &_depth,
                          const RayTable &_rays, const Water &_water,
                          const Response &_response, std::mt19937 &_rng,
                          std::vector<std::uint8_t> &_out,
                          const Optics &_optics = Optics{});
}  // namespace gz::sim::underwater_camera

#endif  // GZ_SIM_UNDERWATER_CAMERA_UNDERWATERIMAGE_HH_
