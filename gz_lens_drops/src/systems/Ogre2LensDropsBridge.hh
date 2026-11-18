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

extern "C"
{
  /// \brief Opaque handle to one render pass, for one camera.
  typedef void *lens_drops_pass_t;

  /// \brief The most drops a pass draws.
  /// \return 64.
  int lens_drops_ogre2_max_drops(void);

  /// \brief Create a pass. It is not on a camera yet.
  /// \param[in] _vs GLSL source of the vertex shader, not a file name.
  /// \param[in] _fs GLSL source of the fragment shader, not a file name.
  /// \return The pass, or NULL if a source is NULL or empty.
  lens_drops_pass_t lens_drops_ogre2_create(const char *_vs, const char *_fs);

  /// \brief Add the pass to a camera, after the passes it already has.
  /// \param[in] _pass The pass.
  /// \param[in] _camera Raw pointer to a `gz::rendering::Camera` of the
  /// ogre2 engine.
  /// \return 1 if added, 0 if an argument is NULL or Ogre refused.
  int lens_drops_ogre2_attach(lens_drops_pass_t _pass, void *_camera);

  /// \brief Set what the pass draws from the next frame on.
  /// \param[in] _pass The pass. NULL does nothing.
  /// \param[in] _drops `_count` drops of four floats each: x and y in
  /// [0, 1] from the image's top left, the radius as a fraction of the
  /// image height, and one unused. May be NULL when `_count` is 0.
  /// \param[in] _count How many drops; clamped to [0, 64].
  /// \param[in] _focus 0 for drops on the lens, wide and blurred, to 1 for
  /// drops on a port ahead of it, sharp.
  void lens_drops_ogre2_set(lens_drops_pass_t _pass, const float *_drops,
      int _count, float _focus);

  /// \brief Take the pass off its camera and free it.
  /// \param[in] _pass The pass. NULL does nothing.
  /// \param[in] _camera The camera it was attached to, or NULL if it never
  /// was or the camera is gone.
  void lens_drops_ogre2_destroy(lens_drops_pass_t _pass, void *_camera);
}

#endif  // GZ_SIM_SYSTEMS_OGRE2LENSDROPSBRIDGE_HH_
