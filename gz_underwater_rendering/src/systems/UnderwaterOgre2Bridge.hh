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
#ifndef GZ_UNDERWATER_RENDERING_UNDERWATEROGRE2BRIDGE_HH_
#define GZ_UNDERWATER_RENDERING_UNDERWATEROGRE2BRIDGE_HH_

#include <cstdint>

// What the plugin and the Ogre bridge say to each other. The bridge is a
// library of its own, loaded with dlopen (the plugin must not link
// gz-rendering's ogre2 library), so this is a C interface, and the water is
// a struct the compiler checks on both sides, with a version the plugin
// checks before anything else.
extern "C"
{
/// \brief The version of this interface. Change it with the struct.
#define UW_OGRE2_BRIDGE_VERSION 2

/// \brief The water, as the plugin hands it to the bridge.
struct UwWater
{
  /// \brief beta^D, beta^B, 1/m, per channel.
  float attenuation[3];
  float backscatter[3];

  /// \brief B-infinity just below the surface, per channel.
  float veil[3];

  /// \brief K_d, 1/m, per channel.
  float diffuse[3];

  /// \brief World z of the still water, m.
  float level;
};

/// \brief The bridge's version of this interface.
/// \return UW_OGRE2_BRIDGE_VERSION, as the bridge was built.
int uw_ogre2_version();
typedef int (*UwVersionFn)();

/// \brief Put the underwater pieces and listener into Ogre's PBS. Call on
/// the render thread once Ogre is up.
/// \param[in] _piecesDir The folder of the underwater pieces.
/// \return 1 if installed (or already), 0 if not.
int uw_ogre2_install(const char *_piecesDir);
typedef int (*UwInstallFn)(const char *);

/// \brief Set the water, or turn it off with a null pointer. Render thread.
/// \param[in] _water The water.
void uw_ogre2_set(const UwWater *_water);
typedef void (*UwSetFn)(const UwWater *);

/// \brief Stop drawing the water, from any thread: the plugin is going (its
/// model removed) and cannot reach the render thread any more. The bridge
/// stays installed, drawing nothing, until a plugin sets the water again.
void uw_ogre2_release();
typedef void (*UwReleaseFn)();

/// \brief Put Ogre's PBS back as it was. Safe to call twice. Render thread.
void uw_ogre2_uninstall();
typedef void (*UwUninstallFn)();
}

#endif  // GZ_UNDERWATER_RENDERING_UNDERWATEROGRE2BRIDGE_HH_
