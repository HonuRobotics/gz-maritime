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
#ifndef GZ_SIM_MARITIME_SYSTEMS_UNDERWATERCAMERA_HH_
#define GZ_SIM_MARITIME_SYSTEMS_UNDERWATERCAMERA_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class UnderwaterCameraPrivate;

  /// \brief Turns an RGBD camera into an underwater camera.
  ///
  /// Put it inside an `rgbd_camera` sensor, which renders colour and depth
  /// in one pass. For every frame it pairs the colour image with the depth
  /// image of the same time stamp, reads the world's water
  /// (gz_water_optics) at the camera, and publishes the image the camera
  /// would see through that water, with the CameraInfo.
  ///
  /// ```xml
  /// <sensor name="camera" type="rgbd_camera">
  ///   <topic>rov/camera</topic>
  ///   ...
  ///   <plugin filename="gz-maritime-underwater-camera-system"
  ///           name="gz::sim::maritime::UnderwaterCamera"/>
  /// </sensor>
  /// ```
  ///
  /// Parameters, all optional:
  ///
  /// * `<output_topic>`: base of the output, default `<topic>/underwater`:
  ///   `<output_topic>/image` and `<output_topic>/camera_info`.
  /// * `<exposure>`: a gain on linear light, default 1, or `auto`, which
  ///   aims the mean luminance at 0.18 within `<max_gain>` (default 16).
  /// * `<white_balance>`: `none` (default) or `grey_world`.
  /// * `<shot_noise>`, `<read_noise>`: Gaussian noise on linear light, of
  ///   variance shot_noise * signal + read_noise^2; default 0.
  /// * `<seed>`: of the noise, default 1, mixed with the sensor's scoped
  ///   name, so each camera, say each of a stereo pair, gets its own noise.
  /// * `<forward_scatter>`: forward scatter, a blur growing with range,
  ///   radians per metre; default 0.
  /// * `<port>`: `none` (default), `flat` (refraction remap) or `dome`
  ///   (centred on the lens: no remap); `<water_index>` default 1.333, 1 or
  ///   more.
  ///
  /// It works only while someone listens to its output: with nobody
  /// subscribed it does not listen to the camera either, so the camera is
  /// not rendered for it. Each frame is processed on a thread of the
  /// plugin's own, with the water of the step it was rendered at; a frame
  /// that comes while one is being processed replaces the one waiting.
  class UnderwaterCamera
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate
  {
    /// \brief Constructor.
    public: UnderwaterCamera();

    /// \brief Destructor.
    public: ~UnderwaterCamera() override;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;

    /// \brief Private data.
    private: std::unique_ptr<UnderwaterCameraPrivate> dataPtr;
  };
}  // namespace gz::sim::maritime

#endif  // GZ_SIM_MARITIME_SYSTEMS_UNDERWATERCAMERA_HH_
