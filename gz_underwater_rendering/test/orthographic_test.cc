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
#include <dlfcn.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

#include "../src/systems/UnderwaterOgre2Bridge.hh"

using namespace gz;
using namespace rendering;

namespace
{
/// \brief The green at the image's centre.
int CentreGreen(CameraPtr _camera)
{
  Image image = _camera->CreateImage();
  _camera->Capture(image);
  const auto *data = image.Data<unsigned char>();
  const unsigned int w = _camera->ImageWidth(), h = _camera->ImageHeight();
  return data[((h / 2) * w + w / 2) * 3 + 1];
}
}  // namespace

/////////////////////////////////////////////////
// An orthographic camera beside a perspective one, the bridge driven
// through its C interface, without Gazebo. The perspective camera sees the
// box through the water, as the model has it; the orthographic one (the
// GUI's orthographic view), which the model does not fit, sees it as in air,
// with no backdrop over it.
TEST(Orthographic, DrawnAsInAirBesideAPerspectiveCamera)
{
  RenderEngine *engine = rendering::engine("ogre2");
  if (!engine)
    GTEST_SKIP() << "ogre2 could not start: no GPU?";
  ScenePtr scene = engine->CreateScene("orthographic");
  ASSERT_NE(nullptr, scene);
  scene->SetAmbientLight(1.0, 1.0, 1.0);
  scene->SetBackgroundColor(0.0, 0.0, 0.0);

  VisualPtr box = scene->CreateVisual();
  box->AddGeometry(scene->CreateBox());
  box->SetLocalScale(1.0, 100.0, 100.0);
  box->SetLocalPosition(2.5, 0.0, 0.0);
  MaterialPtr white = scene->CreateMaterial();
  white->SetAmbient(0.8, 0.8, 0.8);
  white->SetDiffuse(0.8, 0.8, 0.8);
  box->SetMaterial(white);
  scene->RootVisual()->AddChild(box);

  std::map<std::string, CameraPtr> cameras;
  for (const std::string name : {"perspective", "orthographic"})
  {
    CameraPtr camera = scene->CreateCamera(name);
    camera->SetImageWidth(64);
    camera->SetImageHeight(48);
    camera->SetHFOV(0.6);
    camera->SetNearClipPlane(0.1);
    camera->SetFarClipPlane(100.0);
    // gz-rendering's own orthographic view: as many metres wide as the
    // image has pixels, so the box is made wide enough to fill it
    if (name == "orthographic")
      camera->SetProjectionType(CPT_ORTHOGRAPHIC);
    scene->RootVisual()->AddChild(camera);
    cameras[name] = camera;
  }
  std::map<std::string, int> air;
  for (auto &[name, camera] : cameras)
    air[name] = CentreGreen(camera);

  void *bridge = dlopen("libunderwater-ogre2-bridge.so", RTLD_NOW | RTLD_LOCAL);
  ASSERT_NE(nullptr, bridge) << dlerror();
  const auto version = reinterpret_cast<UwVersionFn>(
      dlsym(bridge, "uw_ogre2_version"));
  const auto install = reinterpret_cast<UwInstallFn>(
      dlsym(bridge, "uw_ogre2_install"));
  const auto set = reinterpret_cast<UwSetFn>(dlsym(bridge, "uw_ogre2_set"));
  const auto uninstall = reinterpret_cast<UwUninstallFn>(
      dlsym(bridge, "uw_ogre2_uninstall"));
  ASSERT_TRUE(version && install && set && uninstall);
  ASSERT_EQ(UW_OGRE2_BRIDGE_VERSION, version());
  ASSERT_EQ(1, install(UW_PIECES_DIR));

  // 10 m down, murky, the veil green
  UwWater water{};
  for (int c = 0; c < 3; ++c)
  {
    water.attenuation[c] = 0.5f;
    water.backscatter[c] = 0.5f;
    water.diffuse[c] = 0.0f;
  }
  water.veil[1] = 0.5f;
  water.level = 10.0f;
  water.daylight = 1.0f;
  water.quality = 1;
  set(&water);
  std::map<std::string, int> under;
  for (auto &[name, camera] : cameras)
    under[name] = CentreGreen(camera);

  set(nullptr);
  uninstall();
  std::cout << "centre green, in air and under water: perspective "
            << air["perspective"] << " and " << under["perspective"]
            << ", orthographic " << air["orthographic"] << " and "
            << under["orthographic"] << "\n";
  ASSERT_GT(air["perspective"], 50) << "the box is not in view";
  EXPECT_EQ(air["orthographic"], air["perspective"]);
  // 2 m of water: J t + veil (1 - t), t = exp(-0.5 * 2)
  const double j = std::pow((air["perspective"] / 255.0 + 0.055) / 1.055, 2.4);
  const double t = std::exp(-1.0);
  const double lin = j * t + 0.5 * (1 - t);
  const double expected =
      255.0 * (1.055 * std::pow(lin, 1 / 2.4) - 0.055);
  EXPECT_NEAR(under["perspective"], expected, 2.0) << "not the model's water";
  EXPECT_EQ(under["orthographic"], air["orthographic"])
      << "the orthographic camera is not drawn as in air";
  engine->DestroyScene(scene);
}
