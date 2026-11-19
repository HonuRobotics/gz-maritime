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
#ifndef GZ_SIM_SYSTEMS_OGRE2LENSDROPSBRIDGE_HH_
#define GZ_SIM_SYSTEMS_OGRE2LENSDROPSBRIDGE_HH_

// C ABI for the LensDrops system to add an Ogre Next render pass to a
// camera without itself linking libgz-rendering-ogre2.so, which would break
// gz-rendering's engine loader (see gz_waves_rendering). The bridge lives in
// liblens-drops-ogre2-bridge.so and is loaded with dlopen() on the render
// thread, after the render engine. Every function must be called on the
// render thread. No exception crosses the boundary.
//
// Version 2: the pass draws a drop map, a texture of the height of water
// and the wet film per texel, by refraction (see lens_drops_fs_330.glsl).

extern "C"
{
  /// \brief Opaque handle to one render pass, for one camera.
  typedef void *lens_drops_pass_t;

  /// \brief The version of this ABI.
  /// \return 2.
  int lens_drops_ogre2_abi_version(void);

  /// \brief Create a pass. It is not on a camera yet.
  /// \param[in] _vs GLSL source of the vertex shader, not a file name.
  /// \param[in] _fs GLSL source of the fragment shader, not a file name.
  /// \param[in] _mapWidth Texels across the drop map, in [1, 4096].
  /// \param[in] _mapHeight Texels down the drop map, in [1, 4096].
  /// \return The pass, or NULL if a source is NULL or empty or a size is
  /// out of range.
  lens_drops_pass_t lens_drops_ogre2_create(const char *_vs, const char *_fs,
      int _mapWidth, int _mapHeight);

  /// \brief Add the pass to a camera, after the passes it already has.
  /// \param[in] _pass The pass.
  /// \param[in] _camera Raw pointer to a `gz::rendering::Camera` of the
  /// ogre2 engine.
  /// \return 1 if added, 0 if an argument is NULL or Ogre refused.
  int lens_drops_ogre2_attach(lens_drops_pass_t _pass, void *_camera);

  /// \brief Give the pass a new drop map, copied; it is uploaded before the
  /// pass next draws.
  /// \param[in] _pass The pass.
  /// \param[in] _rgba Width * height * 4 floats, row 0 at the top: height of
  /// water in image heights, wet film in [0, 1], 0, 0.
  /// \param[in] _width Must be the width the pass was created with.
  /// \param[in] _height Must be the height the pass was created with.
  /// \param[in] _focus 0 blurred to 1 sharp, clamped.
  /// \return 1 if taken, 0 if an argument is NULL or a size differs.
  int lens_drops_ogre2_set_map(lens_drops_pass_t _pass, const float *_rgba,
      int _width, int _height, float _focus);

  /// \brief Take the pass off its camera and free it, with its texture.
  /// \param[in] _pass The pass. NULL does nothing.
  /// \param[in] _camera The camera it was attached to, or NULL if it never
  /// was or the camera is gone.
  void lens_drops_ogre2_destroy(lens_drops_pass_t _pass, void *_camera);
}

#endif  // GZ_SIM_SYSTEMS_OGRE2LENSDROPSBRIDGE_HH_
