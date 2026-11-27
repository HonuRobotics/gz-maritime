/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

// A bridge of another version, found before the real one: the plugin must
// refuse it before calling anything else, and the reference camera draws
// the water.
#include "../src/systems/UnderwaterOgre2Bridge.hh"

extern "C"
{
int uw_ogre2_version() { return UW_OGRE2_BRIDGE_VERSION + 1; }
int uw_ogre2_install(const char *) { return 1; }
void uw_ogre2_set(const UwWater *) {}
void uw_ogre2_release() {}
void uw_ogre2_uninstall() {}
}
