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
// thread, after the render engine. No exception crosses the boundary.
//
// The camera owns the pass; a handle only refers to it. The pass pulls its
// map: right before its camera draws a frame, it calls the system back,
// so the map is drawn at most once per frame, and only for frames drawn.

#include <stdint.h>

/// \brief The version of this ABI. Change it with any function below.
#define LENS_DROPS_OGRE2_ABI 3

extern "C"
{
  /// \brief Opaque handle to one render pass, for one camera.
  typedef void *lens_drops_pass_t;

  /// \brief What the pass asks for right before its camera draws a frame.
  /// Called on the render thread, never after lens_drops_ogre2_destroy
  /// returned for the pass's handle.
  /// \param[in] _context The context given to lens_drops_ogre2_create.
  /// \param[in] _stamp Simulation time of the frame, s.
  /// \param[out] _focus 0 blurred to 1 sharp.
  /// \param[out] _enabled Whether the pass draws: 0 for a dry lens.
  /// \return The map to upload, width * height * 2 floats, row 0 at the
  /// top (height of water, wet film), valid until the next call; NULL if it
  /// did not change.
  typedef const float *(*lens_drops_pull_t)(void *_context, double _stamp,
      float *_focus, int *_enabled);

  /// \brief The version of this ABI.
  /// \return LENS_DROPS_OGRE2_ABI.
  int lens_drops_ogre2_abi_version(void);

  /// \brief Create a pass. It is not on a camera yet. Render thread.
  /// \param[in] _vs GLSL source of the vertex shader, not a file name.
  /// \param[in] _fs GLSL source of the fragment shader, not a file name.
  /// \param[in] _mapWidth Texels across the drop map, in [1, 4096].
  /// \param[in] _mapHeight Texels down the drop map, in [1, 4096].
  /// \param[in] _pull Called right before each frame; not NULL.
  /// \param[in] _context Passed to _pull.
  /// \return The handle, or NULL if an argument is NULL, empty or out of
  /// range.
  lens_drops_pass_t lens_drops_ogre2_create(const char *_vs, const char *_fs,
      int _mapWidth, int _mapHeight, lens_drops_pull_t _pull,
      void *_context);

  /// \brief Add the pass to a camera, after the passes it already has. The
  /// camera then owns it. Render thread.
  /// \param[in] _pass The handle.
  /// \param[in] _camera Raw pointer to a `gz::rendering::Camera` of the
  /// ogre2 engine.
  /// \return 1 if added, 0 if an argument is NULL or Ogre refused.
  int lens_drops_ogre2_attach(lens_drops_pass_t _pass, void *_camera);

  /// \brief Let go of a pass, from any thread: it never calls its pull
  /// again once this returns, waiting for a call in progress. The pass
  /// stays with its camera, drawing nothing, until the camera goes.
  /// \param[in] _pass The handle. NULL does nothing.
  void lens_drops_ogre2_destroy(lens_drops_pass_t _pass);
}

#endif  // GZ_SIM_SYSTEMS_OGRE2LENSDROPSBRIDGE_HH_
