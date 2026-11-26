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
#ifndef GZ_MARITIME_SONAR_CAMERASAMPLER_HH_
#define GZ_MARITIME_SONAR_CAMERASAMPLER_HH_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Rays.hh"

namespace gz::maritime::sonar
{
/// \brief The pinhole cameras that cover a sonar's fan: side by side in
/// azimuth, each looking along its own yaw about the sensor's z axis, with
/// square pixels no coarser than the ray spacing. Cameras look along their
/// +x axis, with +y to the left and +z up, as gz-rendering's do.
struct CameraLayout
{
  /// \brief Yaw of each camera about the sensor's z axis, rad.
  std::vector<double> yaws;

  /// \brief Image size, pixels; the same for every camera.
  unsigned int width{0};
  unsigned int height{0};

  /// \brief Focal length, pixels.
  double focal{0.0};

  /// \brief Horizontal field of view of one camera, rad.
  double hfov{0.0};

  /// \brief Clip planes, m.
  double nearClip{0.05};
  double farClip{100.0};
};

/// \brief Widest field of view one camera takes by default, rad.
inline constexpr double kDefaultMaxCameraWidth = 70.0 * M_PI / 180.0;

/// \brief Most cameras one sonar may ask for. A fan may span a full circle
/// (CheckRayFan), which is six cameras of 70 degrees.
inline constexpr std::size_t kMaxCameras = 6;

/// \brief Most pixels along one side of a camera's image. The Omniscan
/// preset, the tallest, is 2 200; 8 192 is a texture size every GPU that
/// runs Gazebo has, and one that llvmpipe has too.
inline constexpr unsigned int kMaxCameraSide = 8192;

/// \brief Most pixels over all of one sonar's cameras: each is a 4-byte
/// depth and a 4-byte label drawn and read back every ping, so this is
/// about 540 MB a ping. The ECHO's two cameras are 0.8 million; the ECHO
/// with 2 048 beams, the most the reader takes, is 52 million.
inline constexpr double kMaxCameraPixels = 67108864.0;

/// \brief The cameras a sonar's fan needs, worked out from the fan's shape
/// alone: nothing is allocated and no ray is built, so this is the check to
/// make before building a sonar. Refuses a fan CheckRayFan refuses, and a
/// layout past kMaxCameras, kMaxCameraSide or kMaxCameraPixels, or whose far
/// clip plane is not beyond its near one.
/// \param[in] _config The sonar.
/// \param[out] _error Why not, naming the values, when there is no layout.
/// \param[in] _maxCameraWidth Widest field of view one camera may take,
/// rad; the fan is split into as many cameras as it needs.
/// \return The layout, or empty.
std::optional<CameraLayout> PlanCameraLayout(const SonarConfig &_config,
  std::string &_error, double _maxCameraWidth = kDefaultMaxCameraWidth);

/// \brief Turns the depth and label images of a sonar's cameras into the
/// ray hits the engine takes. No rendering here: the cameras are drawn by
/// the caller (gz-rendering in Gazebo), and this class only knows their
/// layout, so it can be tested on synthetic images.
///
/// For each ray of the fan (RayFan's grid), the depth is read at the ray's
/// exact position in its camera by interpolating 1/z between the four
/// neighboring pixels, which is exact on a plane. It interpolates where
/// their depths are within 2 % of each other, or where 1/z changes evenly
/// over them and the next pixels (second differences under half the first)
/// and they share a label, as on a seabed seen at a grazing angle, flat or
/// not quite; elsewhere, at a step or a silhouette, the nearest pixel is
/// kept, so a target and the background behind it are not blended. Depth is along the
/// camera's optical axis and is converted to range along the ray. Normals
/// come from neighboring hits on the ray grid on the same surface
/// (SameSurface; the one-sided difference with the smaller range jump), and
/// footprints from AssignFootprints.
class CameraSampler
{
  /// \brief A sampler for one sonar. For a sonar PlanCameraLayout refuses
  /// the sampler is empty, with no cameras and no rays, rather than built
  /// at any cost; check with PlanCameraLayout first to learn why.
  /// \param[in] _config The sonar.
  /// \param[in] _maxCameraWidth Widest field of view one camera may take,
  /// rad; the fan is split into as many cameras as it needs.
  public: explicit CameraSampler(const SonarConfig &_config,
              double _maxCameraWidth = kDefaultMaxCameraWidth);

  /// \brief The cameras to draw.
  /// \return The camera layout.
  public: const CameraLayout &Layout() const;

  /// \brief The fan, in RayFan order.
  /// \return The rays.
  public: const std::vector<Ray> &Rays() const;

  /// \brief The ray through the center of a pixel, in the sensor frame, for
  /// drawing synthetic images in tests.
  /// \param[in] _camera The camera's index in the layout.
  /// \param[in] _u The pixel's column.
  /// \param[in] _v The pixel's row.
  /// \return The ray through the pixel's center.
  public: Ray PixelRay(std::size_t _camera, unsigned int _u,
              unsigned int _v) const;

  /// \brief Turn one ping's images into ray hits.
  /// \param[in] _depth One depth image per camera, row-major, width ×
  /// height floats, depth along the optical axis; +inf or 0 where nothing
  /// was hit.
  /// \param[in] _labels One label image per camera, row-major, width ×
  /// height pixels of _channels bytes; the label is the first byte, 0 for
  /// nothing.
  /// \param[in] _channels Bytes per label pixel.
  /// \param[in] _properties Acoustic properties by label; label 0, and any
  /// label past the end, take entry 0.
  /// \return One hit per ray of the fan, in RayFan order.
  public: std::vector<RayHit> Sample(
              const std::vector<const float *> &_depth,
              const std::vector<const std::uint8_t *> &_labels,
              unsigned int _channels,
              const std::vector<AcousticProperties> &_properties) const;

  /// \brief Whether the four pixels from (_u0, _v0) to (_u0 + 1, _v0 + 1)
  /// lie on one smooth surface: along each row and column of the four and
  /// one pixel beyond, 1/z bends by no more than half its change from one
  /// pixel to the next, and they share a label when labels are given. True
  /// on a plane, on a seabed's creases and undulations; false across a
  /// step between two surfaces.
  /// \param[in] _depth The camera's depth image.
  /// \param[in] _u0 The column of the first pixel.
  /// \param[in] _v0 The row of the first pixel.
  /// \param[in] _labels The camera's label image, or null.
  /// \param[in] _channels Bytes per label pixel.
  /// \return True if the four pixels lie on one smooth surface.
  private: bool Smooth(const float *_depth, long _u0, long _v0,
              const std::uint8_t *_labels, unsigned int _channels) const;

  /// \brief Where each ray falls in its camera.
  private: struct Lookup
  {
    std::size_t camera{0};
    double u{0.0};
    double v{0.0};
    /// \brief Range per unit of depth along this ray.
    double rangePerDepth{1.0};
  };

  private: SonarConfig config;
  private: CameraLayout layout;
  private: std::vector<Ray> rays;
  private: std::vector<Lookup> lookup;
  private: std::size_t elevationCount{0};
};
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_CAMERASAMPLER_HH_
