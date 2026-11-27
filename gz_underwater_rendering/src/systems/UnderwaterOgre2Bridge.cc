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
// The Ogre side of the underwater rendering: puts the underwater pieces into
// Ogre's PBS shaders and the water into each pass's buffer. A shared library
// of its own, loaded with dlopen by the plugin, so the plugin never links
// gz-rendering's ogre2 library (which would stop gz-rendering loading its
// engine; see gz_waves_rendering's CMakeLists.txt).
//
// Two things are chained, never replaced:
// * the pieces that define Ogre's customization hooks we use: each gets
//   one more insertpiece, ours, in a copy of its archive kept in memory, so
//   the rest is always the installed gz-rendering's own. A hook nothing
//   defines is defined by us. Anything else, e.g. a hook defined twice or
//   a piece that does more than insert others, and nothing is installed;
// * gz-rendering's Hlms listener: ours calls it first, then appends.
//
// It is written against gz-rendering 10's pieces and refuses any other
// major version. When it does not install, the plugin does not mark the
// world, and the reference camera draws the water instead.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "UnderwaterOgre2Bridge.hh"

#include <gz/common/Console.hh>
#include <gz/rendering/BoundingBoxCamera.hh>
#include <gz/rendering/Light.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/SegmentationCamera.hh>
#include <gz/rendering/ThermalCamera.hh>
#include <gz/rendering/config.hh>
#include <gz/rendering/ogre2/Ogre2Light.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>

#include <OgreArchive.h>
#include <OgreArchiveManager.h>
#include <OgreDataStream.h>
#include <OgreFrameListener.h>
#include <OgreCamera.h>
#include <OgreHlms.h>
#include <OgreHlmsListener.h>
#include <OgreHlmsManager.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreHlmsPbs.h>
#include <OgreHlmsPbsDatablock.h>
#include <OgreItem.h>
#include <OgreLight.h>
#include <OgreMatrix4.h>
#include <OgreMesh2.h>
#include <OgreMeshManager2.h>
#include <OgreRenderSystem.h>
#include <OgreSceneNode.h>
#include <OgreString.h>
#include <OgreSubItem.h>
#include <OgreSubMesh2.h>
#include <Vao/OgreVaoManager.h>

namespace
{
/// \brief The gz-rendering major version whose pieces this was written
/// against. A build against another fails here; at run time, the strict
/// parse of the pieces is the guard.
constexpr int kGzRenderingMajor = 10;
static_assert(GZ_RENDERING_MAJOR_VERSION == kGzRenderingMajor,
              "the underwater pieces were written against gz-rendering 10");

/// \brief Ogre's customization hooks we add to: the pass buffer block, the
/// functions the lamps use, and the final colour.
const std::vector<std::string> kPbsHooks{"custom_passBuffer",
                                         "custom_ps_functions",
                                         "custom_ps_posExecution"};

/// \brief A read only Ogre archive of files kept in memory: the pieces we
/// change, with nothing written to disk.
class MemoryArchive : public Ogre::Archive
{
  public: explicit MemoryArchive(const std::string &_name)
    : Ogre::Archive(_name, "UnderwaterMemory") {}

  public: std::map<std::string, std::string> files;

  public: bool isCaseSensitive() const override { return true; }
  public: void load() override {}
  public: void unload() override {}

  public: Ogre::DataStreamPtr open(const Ogre::String &_name,
                                   bool) override
  {
    const auto it = this->files.find(_name);
    if (it == this->files.end())
      return Ogre::DataStreamPtr();
    auto *stream = OGRE_NEW Ogre::MemoryDataStream(_name, it->second.size(),
                                                   true, true);
    std::memcpy(stream->getPtr(), it->second.data(), it->second.size());
    return Ogre::DataStreamPtr(stream);
  }

  public: Ogre::StringVectorPtr list(bool, bool _dirs) override
  {
    return this->find("*", true, _dirs);
  }

  public: Ogre::FileInfoListPtr listFileInfo(bool, bool _dirs) override
  {
    return this->findFileInfo("*", true, _dirs);
  }

  public: Ogre::StringVectorPtr find(const Ogre::String &_pattern, bool,
                                     bool _dirs) override
  {
    Ogre::StringVectorPtr names(OGRE_NEW_T(Ogre::StringVector,
        Ogre::MEMCATEGORY_GENERAL)(), Ogre::SPFM_DELETE_T);
    for (const auto &file : this->files)
    {
      if (!_dirs && Ogre::StringUtil::match(file.first, _pattern, true))
        names->push_back(file.first);
    }
    return names;
  }

  public: bool exists(const Ogre::String &_name) override
  {
    return this->files.count(_name) > 0;
  }

  public: time_t getModifiedTime(const Ogre::String &) override
  {
    return 0;
  }

  public: Ogre::FileInfoListPtr findFileInfo(const Ogre::String &_pattern,
                                             bool, bool _dirs) override
  {
    Ogre::FileInfoListPtr infos(OGRE_NEW_T(Ogre::FileInfoList,
        Ogre::MEMCATEGORY_GENERAL)(), Ogre::SPFM_DELETE_T);
    for (const auto &file : this->files)
    {
      if (_dirs || !Ogre::StringUtil::match(file.first, _pattern, true))
        continue;
      Ogre::FileInfo info;
      info.archive = this;
      info.filename = file.first;
      info.basename = file.first;
      info.compressedSize = file.second.size();
      info.uncompressedSize = file.second.size();
      infos->push_back(info);
    }
    return infos;
  }
};
/// \brief The water, as the plugin last set it.
struct Water
{
  float attenuation[3]{0, 0, 0};
  float backscatter[3]{0, 0, 0};
  float veil[3]{0, 0, 0};
  float diffuse[3]{0, 0, 0};
  float level{0};
  float daylight{1};
  float caustics{0};
  float snow{0};
  double time{0};
  int quality{1};
  bool active{false};
};

/// \brief Lamps the shader lights with and draws the beams of.
constexpr int kMaxLamps = 4;

/// \brief float4s per lamp: position (view) and type; direction (view) and
/// range; diffuse and linear attenuation; spot parameters and quadratic
/// attenuation; specular.
constexpr int kLampFloat4 = 5;

/// \brief Bytes the underwater block adds: the water (8 float4), the beam
/// (2), the effects (2) and the lamps.
constexpr Ogre::uint32 kBlockBytes =
    (12u + kMaxLamps * kLampFloat4) * 4u * sizeof(float);

/// \brief Henyey-Greenstein asymmetry of the water's scattering, and the
/// beam's ray march.
constexpr float kPhaseG = 0.9f;
constexpr float kBeamRange = 20.0f;

/// \brief What the plugin's <quality> sets, for low, medium and high: the
/// steps of each lamp's beam march, how far from the camera marine snow is
/// drawn, m, and how many lamps are used (the first in the scene).
constexpr float kBeamSteps[3] = {8.0f, 12.0f, 16.0f};
constexpr float kSnowRange[3] = {1.5f, 3.0f, 3.0f};
constexpr int kQualityLamps[3] = {2, 4, 4};

/// \brief A lamp dimmer than this (its power scale times its brightest
/// diffuse channel) counts as off: the shader neither lights with it nor
/// marches its beam. With every lamp off the lamp code is compiled out.
constexpr float kLampMinPower = 0.01f;

/// \brief The backdrop's size, m: its half width, how far above the water
/// its top is and how far below its bottom. Farther than any camera sees.
constexpr float kBackdropHalfWidth = 50000.0f;
constexpr float kBackdropAbove = 100.0f;
constexpr float kBackdropBelow = 12000.0f;

/// \brief What marks the backdrop's material for the shader (its first
/// user value), which draws it for underwater cameras only.
constexpr float kBackdropMark = 1977.0f;

/// \brief Water behind everything: a black box around the world, seen from
/// inside, drawn after the sky and at the far plane wherever nothing else
/// is. Through the water it is the water's own colour, the veil, as open
/// water is; without it a camera under water would see the sky or the
/// background colour past the last thing in view. The shader makes it
/// clear (alpha 0) for cameras above the water.
class Backdrop : public Ogre::Camera::Listener
{
  /// \brief Hidden while a camera that is not a colour camera renders
  /// (bounding boxes, segmentation, thermal): those swap the materials of
  /// gz-rendering's own items only, and would read the backdrop, which is
  /// not one, as a thing filling the view. Before culling, as their own
  /// material switchers work.
  public: void cameraPreRenderScene(Ogre::Camera *_camera) override
  {
    const auto it = this->items.find(_camera->getSceneManager());
    if (it == this->items.end())
      return;
    this->wasVisible = it->second->getVisible();
    it->second->setVisible(false);
  }

  /// \brief Shown again as it was, after such a camera.
  public: void cameraPostRenderScene(Ogre::Camera *_camera) override
  {
    const auto it = this->items.find(_camera->getSceneManager());
    if (it != this->items.end())
      it->second->setVisible(this->wasVisible);
  }

  /// \brief Listen to the cameras that must not see the backdrop: the
  /// bounding box, segmentation and thermal cameras of every scene, whose
  /// Ogre camera carries the sensor's name. New ones are found as they come.
  private: void WatchCameras()
  {
    if (!gz::rendering::isEngineLoaded("ogre2"))
      return;
    gz::rendering::RenderEngine *engine = gz::rendering::engine("ogre2");
    for (unsigned int s = 0; engine && s < engine->SceneCount(); ++s)
    {
      const auto scene = engine->SceneByIndex(s);
      const auto *ogreScene =
          dynamic_cast<const gz::rendering::Ogre2Scene *>(scene.get());
      if (!ogreScene)
        continue;
      Ogre::SceneManager *sm = ogreScene->OgreSceneManager();
      for (unsigned int i = 0; i < scene->SensorCount(); ++i)
      {
        const auto sensor = scene->SensorByIndex(i);
        if (!std::dynamic_pointer_cast<gz::rendering::BoundingBoxCamera>(
                sensor) &&
            !std::dynamic_pointer_cast<gz::rendering::SegmentationCamera>(
                sensor) &&
            !std::dynamic_pointer_cast<gz::rendering::ThermalCamera>(sensor))
        {
          continue;
        }
        Ogre::Camera *camera = sm->findCameraNoThrow(sensor->Name());
        if (camera && this->watched.insert({sm, sensor->Name(), camera})
                          .second)
        {
          camera->addListener(this);
        }
      }
    }
  }

  /// \brief Show it in every scene, around the water level, or hide it.
  public: void Update(bool _show, float _level)
  {
    this->WatchCameras();
    Ogre::Root *root = Ogre::Root::getSingletonPtr();
    if (!root)
      return;
    auto managers = root->getSceneManagerIterator();
    while (managers.hasMoreElements())
    {
      Ogre::SceneManager *sm = managers.getNext();
      auto it = this->items.find(sm);
      if (it == this->items.end())
      {
        if (!_show || this->failed || !this->Create(root, sm))
          continue;
        it = this->items.find(sm);
      }
      const float half = 0.5f * (kBackdropAbove + kBackdropBelow);
      Ogre::SceneNode *node = it->second->getParentSceneNode();
      node->setPosition(0.0f, 0.0f, _level + kBackdropAbove - half);
      node->setScale(kBackdropHalfWidth, kBackdropHalfWidth, half);
      it->second->setVisible(_show);
    }
  }

  /// \brief Call before each pass of a scene. gz-rendering's solid-colour
  /// cameras (segmentation, thermal, selection) read custom parameter 1
  /// from every renderable, set it on the visuals they know and take it
  /// off all of them afterwards, so the backdrop gets its own here; the
  /// shader makes the backdrop clear for those cameras.
  public: void BeforePass(Ogre::SceneManager *_sm)
  {
    const auto it = this->items.find(_sm);
    if (it == this->items.end())
      return;
    for (std::size_t i = 0; i < it->second->getNumSubItems(); ++i)
    {
      it->second->getSubItem(i)->setCustomParameter(1u,
                                                    Ogre::Vector4::ZERO);
    }
  }

  /// \brief Take it out of the scenes that are still there.
  public: void Destroy()
  {
    Ogre::Root *root = Ogre::Root::getSingletonPtr();
    if (root)
    {
      auto managers = root->getSceneManagerIterator();
      while (managers.hasMoreElements())
      {
        Ogre::SceneManager *sm = managers.getNext();
        auto it = this->items.find(sm);
        if (it == this->items.end())
          continue;
        Ogre::SceneNode *node = it->second->getParentSceneNode();
        node->detachObject(it->second);
        sm->destroyItem(it->second);
        sm->destroySceneNode(node);
      }
    }
    this->items.clear();
    // Stop listening to the cameras still there
    if (gz::rendering::isEngineLoaded("ogre2"))
    {
      for (const auto &[sm, name, camera] : this->watched)
      {
        if (root && Alive(root, sm) &&
            sm->findCameraNoThrow(name) == camera)
        {
          camera->removeListener(this);
        }
      }
    }
    this->watched.clear();
  }

  /// \brief Forget the boxes and the cameras, touching none: the Ogre
  /// they were in may be gone.
  public: void Forget()
  {
    this->items.clear();
    this->watched.clear();
    this->failed = false;
  }

  /// \brief Whether a scene manager is still Ogre's.
  private: static bool Alive(Ogre::Root *_root, Ogre::SceneManager *_sm)
  {
    auto managers = _root->getSceneManagerIterator();
    while (managers.hasMoreElements())
    {
      if (managers.getNext() == _sm)
        return true;
    }
    return false;
  }

  /// \brief The box in one scene.
  private: bool Create(Ogre::Root *_root, Ogre::SceneManager *_sm)
  try
  {
    const char *name = "gz_underwater_backdrop";
    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().getByName(name);
    if (!mesh)
      mesh = this->Mesh(_root, name);
    auto *pbs = static_cast<Ogre::HlmsPbs *>(
        _root->getHlmsManager()->getHlms(Ogre::HLMS_PBS));
    Ogre::HlmsDatablock *datablock = pbs->getDatablock(name);
    if (!datablock)
    {
      // Seen from inside, whichever way its faces wind, and past the far
      // plane: clamped onto it instead of clipped.
      Ogre::HlmsMacroblock macroblock;
      macroblock.mCullMode = Ogre::CULL_NONE;
      macroblock.mDepthClamp = true;
      macroblock.mDepthWrite = false;
      // Blended by alpha: the shader writes 1 for a camera under water and
      // 0 for any other, which then sees through it.
      Ogre::HlmsBlendblock blendblock;
      blendblock.setBlendType(Ogre::SBT_TRANSPARENT_ALPHA);
      auto *block = static_cast<Ogre::HlmsPbsDatablock *>(
          pbs->createDatablock(name, name, macroblock, blendblock,
                               Ogre::HlmsParamVec()));
      block->setDiffuse(Ogre::Vector3::ZERO);
      block->setSpecular(Ogre::Vector3::ZERO);
      block->setUserValue(0, Ogre::Vector4(kBackdropMark, 0, 0, 0));
      datablock = block;
    }
    Ogre::Item *item = _sm->createItem(mesh, Ogre::SCENE_DYNAMIC);
    item->setDatablock(datablock);
    item->setCastShadows(false);
    // gz-rendering's own queue (10), after the sky, which it draws between
    // its queues 1 and 2 wherever the depth is still the far plane's. Being
    // blended, Ogre draws it after the queue's opaque items, so the depth
    // test rejects the pixels they cover and the beams and the snow are not
    // run there twice; and, the farthest, first of the blended ones, which
    // still blend over it.
    item->setRenderQueueGroup(10u);
    _sm->getRootSceneNode()->createChildSceneNode()->attachObject(item);
    this->items[_sm] = item;
    return true;
  }
  catch (const std::exception &_e)
  {
    gzerr << "[underwater] no backdrop: " << _e.what() << "\n";
    this->failed = true;
    return false;
  }

  /// \brief A box from -1 to 1, with normals facing in.
  private: Ogre::MeshPtr Mesh(Ogre::Root *_root, const char *_name)
  {
    Ogre::VaoManager *vao = _root->getRenderSystem()->getVaoManager();
    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
        _name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::SubMesh *sub = mesh->createSubMesh();
    Ogre::VertexElement2Vec elements;
    elements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3,
                                            Ogre::VES_POSITION));
    elements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3,
                                            Ogre::VES_NORMAL));
    // Six faces of four vertices: the face's axis, then its two others.
    auto *vertices = reinterpret_cast<float *>(OGRE_MALLOC_SIMD(
        sizeof(float) * 6 * 24, Ogre::MEMCATEGORY_GEOMETRY));
    auto *indices = reinterpret_cast<Ogre::uint16 *>(OGRE_MALLOC_SIMD(
        sizeof(Ogre::uint16) * 36, Ogre::MEMCATEGORY_GEOMETRY));
    float *v = vertices;
    Ogre::uint16 *ix = indices;
    for (int face = 0; face < 6; ++face)
    {
      const int axis = face / 2;
      const float side = (face % 2) ? 1.0f : -1.0f;
      const int a = (axis + 1) % 3, b = (axis + 2) % 3;
      const float corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
      for (const auto &c : corners)
      {
        float pos[3], normal[3] = {0, 0, 0};
        pos[axis] = side;
        pos[a] = c[0];
        pos[b] = c[1];
        normal[axis] = -side;
        for (float f : pos)
          *v++ = f;
        for (float f : normal)
          *v++ = f;
      }
      const Ogre::uint16 base = static_cast<Ogre::uint16>(4 * face);
      for (int k : {0, 1, 2, 0, 2, 3})
        *ix++ = static_cast<Ogre::uint16>(base + k);
    }
    Ogre::VertexBufferPacked *vertexBuffer = vao->createVertexBuffer(
        elements, 24, Ogre::BT_IMMUTABLE, vertices, true);
    Ogre::IndexBufferPacked *indexBuffer = vao->createIndexBuffer(
        Ogre::IndexBufferPacked::IT_16BIT, 36, Ogre::BT_IMMUTABLE, indices,
        true);
    Ogre::VertexBufferPackedVec buffers;
    buffers.push_back(vertexBuffer);
    Ogre::VertexArrayObject *object = vao->createVertexArrayObject(
        buffers, indexBuffer, Ogre::OT_TRIANGLE_LIST);
    sub->mVao[Ogre::VpNormal].push_back(object);
    sub->mVao[Ogre::VpShadow].push_back(object);
    mesh->_setBounds(Ogre::Aabb(Ogre::Vector3::ZERO,
                                Ogre::Vector3::UNIT_SCALE), false);
    mesh->_setBoundingSphereRadius(1.7320508f);
    return mesh;
  }

  private: std::map<Ogre::SceneManager *, Ogre::Item *> items;

  /// \brief The cameras listened to: scene, name, camera.
  private: std::set<std::tuple<Ogre::SceneManager *, std::string,
                               Ogre::Camera *>> watched;

  /// \brief The backdrop's visibility before a camera hid it.
  private: bool wasVisible{false};

  /// \brief Set when the box could not be made: not tried again.
  private: bool failed{false};
};

class UnderwaterListener : public Ogre::HlmsListener
{
  public: Ogre::HlmsListener *previous{nullptr};
  public: Water water;
  public: Backdrop backdrop;

  /// \brief Set from any thread when the plugin is gone: draw nothing, and
  /// on the next frame turn the water off as a plugin would.
  public: std::atomic<bool> released{false};

  private: bool On(bool _caster) const
  { return this->water.active && !this->released && !_caster; }

  /// \brief The scene's lamps (spot and point lights), as of the last
  /// preparePassHash.
  private: std::vector<const Ogre::Light *> lamps;

  /// \brief Lamps this listener hid from Ogre's own lighting, so that the
  /// shader lights with them once, through the water.
  public: std::set<Ogre::Light *> hidden;

  /// \brief The scene's lamps: its spot and point lights whose own name,
  /// after the model and link that scope it, contains "lamp" (any case).
  /// Other lights are not lamps: Ogre lights with them as in air. At most
  /// kMaxLamps, the first by name; a warning, once, for the others, which
  /// are left to Ogre too.
  /// \param[out] _alive Every light of the scenes, lamp or not.
  /// \return The lamps.
  private: std::set<Ogre::Light *> Lamps(std::set<Ogre::Light *> &_alive)
  {
    std::vector<std::pair<std::string, Ogre::Light *>> named;
    if (!gz::rendering::isEngineLoaded("ogre2"))
      return {};
    gz::rendering::RenderEngine *engine = gz::rendering::engine("ogre2");
    for (unsigned int s = 0; engine && s < engine->SceneCount(); ++s)
    {
      const auto scene = engine->SceneByIndex(s);
      for (unsigned int i = 0; scene && i < scene->LightCount(); ++i)
      {
        const auto light = scene->LightByIndex(i);
        const auto *ogre2 =
            dynamic_cast<const gz::rendering::Ogre2Light *>(light.get());
        if (!ogre2 || !ogre2->Light())
          continue;
        Ogre::Light *l = ogre2->Light();
        _alive.insert(l);
        if (l->getType() != Ogre::Light::LT_SPOTLIGHT &&
            l->getType() != Ogre::Light::LT_POINT)
        {
          continue;
        }
        std::string own = light->Name();
        const auto scope = own.rfind("::");
        if (scope != std::string::npos)
          own = own.substr(scope + 2);
        std::transform(own.begin(), own.end(), own.begin(),
                       [](unsigned char _c) { return std::tolower(_c); });
        if (own.find("lamp") != std::string::npos)
          named.emplace_back(light->Name(), l);
      }
    }
    std::sort(named.begin(), named.end());
    if (static_cast<int>(named.size()) > kMaxLamps && !this->warnedLamps)
    {
      gzwarn << "[underwater] " << named.size() << " lamps, more than "
             << kMaxLamps << ": the lamps after " << named[kMaxLamps - 1].first
             << " are lit by Ogre, as in air\n";
      this->warnedLamps = true;
    }
    std::set<Ogre::Light *> lamps;
    for (const auto &[name, l] : named)
    {
      if (static_cast<int>(lamps.size()) < kMaxLamps)
        lamps.insert(l);
    }
    return lamps;
  }

  /// \brief Hide the scene's lamps from Ogre's lighting (the shader lights
  /// with them), or give them back. Call on the render thread before
  /// rendering. Only lights still in a scene are touched: a lamp removed
  /// with its vehicle is gone from Ogre, and from here.
  public: void HideLamps(bool _hide)
  {
    std::set<Ogre::Light *> alive;
    std::set<Ogre::Light *> lamps = this->Lamps(alive);
    if (!_hide)
      lamps.clear();
    for (Ogre::Light *l : this->hidden)
    {
      if (alive.count(l) && !lamps.count(l))
        l->setVisible(true);
    }
    for (Ogre::Light *l : lamps)
      l->setVisible(false);
    this->hidden = lamps;
  }

  /// \brief Forget the lamps, touching none: the Ogre they were in may be
  /// gone.
  public: void Forget()
  {
    this->lamps.clear();
    this->hidden.clear();
  }

  /// \brief Whether there were more lamps than the shader takes, warned
  private: bool warnedLamps{false};

  private: void CollectLamps(Ogre::SceneManager *_sm)
  {
    this->lamps.clear();
    auto it = _sm->getMovableObjectIterator(
        Ogre::LightFactory::FACTORY_TYPE_NAME);
    while (it.hasMoreElements() &&
           static_cast<int>(this->lamps.size()) <
               kQualityLamps[this->water.quality])
    {
      auto *light = static_cast<Ogre::Light *>(it.getNext());
      if (!this->hidden.count(light) || !light->getParentNode())
        continue;
      if (light->getType() != Ogre::Light::LT_SPOTLIGHT &&
          light->getType() != Ogre::Light::LT_POINT)
      {
        continue;
      }
      // The beam's ray march is the costliest part of the shader and it
      // runs for every lamp in the list, so a lamp that gives no light
      // stays out of it.
      const Ogre::ColourValue &c = light->getDiffuseColour();
      const float power = light->getPowerScale() *
          std::max(c.r, std::max(c.g, c.b));
      if (power >= kLampMinPower)
        this->lamps.push_back(light);
    }
  }

  public: void propertiesMergedPreGenerationStep(
      Ogre::Hlms *_hlms, const Ogre::HlmsCache &_passCache,
      const Ogre::HlmsPropertyVec &_renderableProps,
      const Ogre::PiecesMap _pieces[Ogre::NumShaderTypes],
      const Ogre::HlmsPropertyVec &_properties,
      const Ogre::QueuedRenderable &_queued) override
  {
    if (this->previous)
    {
      this->previous->propertiesMergedPreGenerationStep(
          _hlms, _passCache, _renderableProps, _pieces, _properties, _queued);
    }
  }

  public: Ogre::uint16 getNumExtraPassTextures(
      const Ogre::HlmsPropertyVec &_properties, bool _caster) const override
  {
    return this->previous ?
        this->previous->getNumExtraPassTextures(_properties, _caster) : 0u;
  }

  public: void setupRootLayout(Ogre::RootLayout &_layout,
      const Ogre::HlmsPropertyVec &_properties) const override
  {
    if (this->previous)
      this->previous->setupRootLayout(_layout, _properties);
  }

  public: void shaderCacheEntryCreated(const Ogre::String &_profile,
      const Ogre::HlmsCache *_entry, const Ogre::HlmsCache &_passCache,
      const Ogre::HlmsPropertyVec &_properties,
      const Ogre::QueuedRenderable &_queued) override
  {
    if (this->previous)
    {
      this->previous->shaderCacheEntryCreated(_profile, _entry, _passCache,
                                              _properties, _queued);
    }
  }

  public: void preparePassHash(const Ogre::CompositorShadowNode *_shadow,
      bool _caster, bool _dual, Ogre::SceneManager *_sm,
      Ogre::Hlms *_hlms) override
  {
    if (this->previous)
      this->previous->preparePassHash(_shadow, _caster, _dual, _sm, _hlms);
    this->backdrop.BeforePass(_sm);
    if (!this->On(_caster))
      return;
    _hlms->_setProperty("gz_underwater", 1);
    this->CollectLamps(_sm);
    // The shader lights with the lamps itself, through the water; they are
    // hidden from Ogre's own lighting (HideLamps), so they count once.
    if (!this->lamps.empty())
      _hlms->_setProperty("gz_underwater_lamps", 1);
  }

  public: Ogre::uint32 getPassBufferSize(
      const Ogre::CompositorShadowNode *_shadow, bool _caster, bool _dual,
      Ogre::SceneManager *_sm) const override
  {
    Ogre::uint32 size = this->previous ?
        this->previous->getPassBufferSize(_shadow, _caster, _dual, _sm) : 0u;
    // Ogre aligns each listener's block to 16 bytes.
    size = (size + 15u) & ~15u;
    return size + (this->On(_caster) ? kBlockBytes : 0u);
  }

  public: float *preparePassBuffer(const Ogre::CompositorShadowNode *_shadow,
      bool _caster, bool _dual, Ogre::SceneManager *_sm,
      float *_ptr) override
  {
    if (this->previous)
      _ptr = this->previous->preparePassBuffer(_shadow, _caster, _dual, _sm,
                                               _ptr);
    if (!this->On(_caster))
      return _ptr;
    // Collect the lamps again for this pass: the block's size does not
    // depend on how many there are, so the data never depends on the order
    // Ogre calls the listener in.
    this->CollectLamps(_sm);
    const Ogre::Camera *camera = _sm->getCamerasInProgress().renderingCamera;
    Ogre::Vector3 pos = Ogre::Vector3::ZERO;
    Ogre::Matrix3 rot = Ogre::Matrix3::IDENTITY;
    if (camera)
    {
      pos = camera->getDerivedPosition();
      camera->getDerivedOrientation().ToRotationMatrix(rot);
    }
    const auto &w = this->water;
    const float depth = std::max(0.0f, w.level - static_cast<float>(pos.z));
    const auto put4 = [&_ptr](float _a, float _b, float _c, float _d)
    {
      *_ptr++ = _a; *_ptr++ = _b; *_ptr++ = _c; *_ptr++ = _d;
    };
    put4(pos.x, pos.y, pos.z, w.level);
    for (int c = 0; c < 3; ++c)
    {
      const auto col = rot.GetColumn(c);
      put4(col.x, col.y, col.z, 0.0f);
    }
    put4(w.attenuation[0], w.attenuation[1], w.attenuation[2], 0.0f);
    put4(w.backscatter[0], w.backscatter[1], w.backscatter[2], 0.0f);
    put4(w.daylight * w.veil[0] * std::exp(-w.diffuse[0] * depth),
         w.daylight * w.veil[1] * std::exp(-w.diffuse[1] * depth),
         w.daylight * w.veil[2] * std::exp(-w.diffuse[2] * depth), 0.0f);
    put4(w.diffuse[0], w.diffuse[1], w.diffuse[2], 0.0f);
    // The scattering, b = veil c (the veil is b / c from the type), and the
    // phase function's asymmetry.
    put4(w.veil[0] * w.attenuation[0], w.veil[1] * w.attenuation[1],
         w.veil[2] * w.attenuation[2], kPhaseG);
    put4(static_cast<float>(this->lamps.size()), kBeamSteps[w.quality],
         kBeamRange, 1.0f);
    put4(w.caustics, w.snow, 0.0f, kSnowRange[w.quality]);
    const auto phase = [&w](double _rate)
    {
      return static_cast<float>(std::fmod(_rate * w.time, 6.283185307179586));
    };
    put4(phase(0.8), phase(1.0), phase(0.9), phase(1.1));
    // The lamps, in view space, as Ogre's own light buffer has them.
    Ogre::Matrix4 view = Ogre::Matrix4::IDENTITY;
    if (camera)
      view = camera->getViewMatrix(true);
    Ogre::Matrix3 viewRot;
    view.extract3x3Matrix(viewRot);
    for (int i = 0; i < kMaxLamps; ++i)
    {
      if (i >= static_cast<int>(this->lamps.size()))
      {
        for (int k = 0; k < kLampFloat4; ++k)
          put4(0, 0, 0, 0);
        continue;
      }
      const Ogre::Light *l = this->lamps[i];
      const bool spot = l->getType() == Ogre::Light::LT_SPOTLIGHT;
      const Ogre::Vector3 p = view * l->getParentNode()->_getDerivedPosition();
      const Ogre::Vector3 dir = (viewRot * l->getDerivedDirection())
                                    .normalisedCopy();
      const Ogre::ColourValue c = l->getDiffuseColour() * l->getPowerScale();
      const Ogre::ColourValue sc = l->getSpecularColour() * l->getPowerScale();
      const float inner = l->getSpotlightInnerAngle().valueRadians();
      const float outer = l->getSpotlightOuterAngle().valueRadians();
      const float cInner = std::cos(inner * 0.5f);
      const float cOuter = std::cos(outer * 0.5f);
      put4(p.x, p.y, p.z, spot ? 1.0f : 0.0f);
      put4(dir.x, dir.y, dir.z, l->getAttenuationRange());
      put4(c.r, c.g, c.b, l->getAttenuationLinear());
      put4(spot ? 1.0f / std::max(cInner - cOuter, 1e-4f) : 0.0f,
           spot ? cOuter : -1.0f, spot ? l->getSpotlightFalloff() : 1.0f,
           l->getAttenuationQuadric());
      put4(sc.r, sc.g, sc.b, 0.0f);
    }
    return _ptr;
  }

  public: void hlmsTypeChanged(bool _caster, Ogre::CommandBuffer *_cmd,
      const Ogre::HlmsDatablock *_db, size_t _texUnit) override
  {
    if (this->previous)
      this->previous->hlmsTypeChanged(_caster, _cmd, _db, _texUnit);
  }
};

void SetWater(const UwWater *_v);

/// \brief Finishes a release on the render thread, at the start of each
/// frame: what turning the water off does, done by nobody else once the
/// plugin is gone.
class Settler : public Ogre::FrameListener
{
  public: bool frameStarted(const Ogre::FrameEvent &) override;
};

/// \brief Everything installed, to put back.
struct Installed
{
  /// \brief The Ogre we installed into. Compared, never followed, once
  /// it may be gone.
  Ogre::Root *root{nullptr};
  Ogre::HlmsPbs *pbs{nullptr};
  UnderwaterListener listener;
  Settler settler;
  Ogre::ArchiveVec originalLibrary;
  /// \brief The pieces we changed or added, read by the Hlms
  std::vector<std::unique_ptr<MemoryArchive>> archives;
  bool done{false};
};
Installed &State()
{
  static Installed installed;
  return installed;
}

/// \brief Whether what we installed is still in this Ogre: its PBS is the
/// one we changed and still has our listener. A world after ours, in the
/// same process, has a new Ogre, even at the same address.
/// \param[in] _pbs The PBS of the Ogre running now, or null.
/// \return True if ours is installed there.
bool Live(Ogre::HlmsPbs *_pbs)
{
  const auto &s = State();
  return s.done && _pbs && _pbs == s.pbs &&
         _pbs->getListener() == &s.listener;
}

/// \brief Forget what was installed, touching no Ogre object: the Ogre it
/// was in may be gone.
void Drop()
{
  auto &s = State();
  s.archives.clear();
  s.originalLibrary.clear();
  s.pbs = nullptr;
  s.root = nullptr;
  s.done = false;
  s.listener.previous = nullptr;
  s.listener.water = Water{};
  s.listener.released = false;
  s.listener.Forget();
  s.listener.backdrop.Forget();
}

/// \brief The PBS of the Ogre running now.
/// \return It, or null if there is none.
Ogre::HlmsPbs *CurrentPbs()
{
  Ogre::Root *root = Ogre::Root::getSingletonPtr();
  if (!root || !root->getHlmsManager())
    return nullptr;
  return dynamic_cast<Ogre::HlmsPbs *>(
      root->getHlmsManager()->getHlms(Ogre::HLMS_PBS));
}

bool Settler::frameStarted(const Ogre::FrameEvent &)
{
  if (State().listener.released.exchange(false))
    SetWater(nullptr);
  return true;
}

/// \brief Set the water, or turn it off. Render thread.
/// \param[in] _v The water, or null.
void SetWater(const UwWater *_v)
{
  auto &s = State();
  auto &w = s.listener.water;
  w.active = nullptr != _v;
  // The lamps hidden while the water is drawn, given back when it is not.
  // Lamps that appear later (a vehicle spawned with its lights) are hidden
  // on the frame they appear.
  s.listener.HideLamps(w.active);
  s.listener.backdrop.Update(s.done && w.active, _v ? _v->level : 0.0f);
  if (!_v)
    return;
  std::memcpy(w.attenuation, _v->attenuation, sizeof(w.attenuation));
  std::memcpy(w.backscatter, _v->backscatter, sizeof(w.backscatter));
  std::memcpy(w.veil, _v->veil, sizeof(w.veil));
  std::memcpy(w.diffuse, _v->diffuse, sizeof(w.diffuse));
  w.level = _v->level;
  w.daylight = _v->daylight;
  w.caustics = _v->caustics;
  w.snow = _v->snow;
  w.time = _v->time;
  w.quality = std::clamp(_v->quality, 0, 2);
}

/// \brief Where a piece is defined: "@piece( name )", any spacing.
/// \param[in] _text A pieces file.
/// \param[in] _piece The piece.
/// \return Each definition's start and the length of its opening tag.
std::vector<std::pair<std::size_t, std::size_t>> Definitions(
    const std::string &_text, const std::string &_piece)
{
  std::vector<std::pair<std::size_t, std::size_t>> out;
  const std::regex open("@piece\\(\\s*" + _piece + "\\s*\\)");
  for (auto it = std::sregex_iterator(_text.begin(), _text.end(), open);
       it != std::sregex_iterator(); ++it)
  {
    out.emplace_back(it->position(), it->length());
  }
  return out;
}

/// \brief Whether an archive defines `<hook>_<suffix>` for every hook.
/// \param[in] _archive The archive.
/// \param[in] _hooks The hooks.
/// \param[in] _suffix The suffix.
/// \return True if it defines all of them.
bool Defines(Ogre::Archive *_archive, const std::vector<std::string> &_hooks,
             const std::string &_suffix)
{
  std::string text;
  const Ogre::StringVectorPtr names = _archive->list(false, false);
  for (const auto &name : *names)
    text += _archive->open(name)->getAsString();
  for (const auto &hook : _hooks)
  {
    if (Definitions(text, hook + "_" + _suffix).empty())
      return false;
  }
  return true;
}

/// \brief Insert a line in a piece that only inserts other pieces.
/// \param[in,out] _text The pieces file.
/// \param[in] _start Where the piece's opening tag starts.
/// \param[in] _openLength Length of that tag.
/// \param[in] _line The line to add before its @end.
/// \return False if the piece holds anything but insertpieces, blank lines
/// and comments: we would not know where our line goes.
bool InsertIntoComposite(std::string &_text, std::size_t _start,
                         std::size_t _openLength, const std::string &_line)
{
  const std::size_t bodyStart = _start + _openLength;
  const std::size_t end = _text.find("@end", bodyStart);
  if (end == std::string::npos)
    return false;
  static const std::regex allowed(
      "^\\s*(@insertpiece\\(\\s*\\w+\\s*\\)\\s*)?(//.*)?$");
  std::size_t lineStart = bodyStart;
  while (lineStart < end)
  {
    std::size_t lineEnd = _text.find('\n', lineStart);
    if (lineEnd == std::string::npos || lineEnd > end)
      lineEnd = end;
    if (!std::regex_match(_text.substr(lineStart, lineEnd - lineStart),
                          allowed))
    {
      return false;
    }
    lineStart = lineEnd + 1;
  }
  _text.insert(end, "  " + _line + "\n");
  return true;
}

/// \brief Chain our pieces into an Hlms' pieces library: for each hook,
/// insert `<hook>_<suffix>` where the hook is defined, or define it.
/// \param[in,out] _library The library; a changed archive is replaced by
/// its copy in memory, and the definitions we add come last.
/// \param[in] _hooks The hooks.
/// \param[in] _suffix Suffix of our pieces, e.g. "underwater".
/// \param[out] _archives Owns the archives in memory.
/// \return False, with an error, if a hook is defined more than once or in
/// a piece we do not understand.
bool Chain(Ogre::ArchiveVec &_library, const std::vector<std::string> &_hooks,
           const std::string &_suffix,
           std::vector<std::unique_ptr<MemoryArchive>> &_archives)
{
  // Every file of the library, to find the hooks in
  struct File
  {
    std::size_t archive;
    std::string name;
    std::string text;
  };
  std::vector<File> files;
  for (std::size_t i = 0; i < _library.size(); ++i)
  {
    const Ogre::StringVectorPtr names = _library[i]->list(false, false);
    for (const auto &name : *names)
      files.push_back({i, name, _library[i]->open(name)->getAsString()});
  }

  std::map<std::size_t, bool> changed;
  std::string ours;
  for (const auto &hook : _hooks)
  {
    const std::string line = "@insertpiece( " + hook + "_" + _suffix + " )";
    File *where = nullptr;
    std::pair<std::size_t, std::size_t> at;
    int count = 0;
    for (auto &file : files)
    {
      const auto found = Definitions(file.text, hook);
      count += static_cast<int>(found.size());
      if (!found.empty())
      {
        where = &file;
        at = found.front();
      }
    }
    if (count > 1)
    {
      gzerr << "[underwater] " << hook << " is defined " << count
            << " times in the pieces library; not installed\n";
      return false;
    }
    if (!where)
    {
      ours += "@piece( " + hook + " )\n  " + line + "\n@end\n";
      continue;
    }
    if (!InsertIntoComposite(where->text, at.first, at.second, line))
    {
      gzerr << "[underwater] " << hook << " in " << where->name
            << " does more than insert pieces; not installed\n";
      return false;
    }
    changed[where->archive] = true;
  }

  for (const auto &[index, unused] : changed)
  {
    auto copy = std::make_unique<MemoryArchive>(
        _library[index]->getName() + " (underwater)");
    for (const auto &file : files)
    {
      if (file.archive == index)
        copy->files[file.name] = file.text;
    }
    _library[index] = copy.get();
    _archives.push_back(std::move(copy));
  }
  if (!ours.empty())
  {
    auto defined = std::make_unique<MemoryArchive>("Underwater hooks");
    defined->files["500.UnderwaterHooks_piece_all.any"] = ours;
    _library.push_back(defined.get());
    _archives.push_back(std::move(defined));
  }
  return true;
}
}  // namespace

extern "C"
{
//////////////////////////////////////////////////
/// \brief Put the underwater pieces and listener into Ogre's PBS. Call on
/// the render thread once Ogre is up.
/// \param[in] _piecesDir The folder of the underwater pieces.
/// \return 1 if installed (or already), 0 if not.
int uw_ogre2_install(const char *_piecesDir)
try
{
  auto &s = State();
  Ogre::Root *root = Ogre::Root::getSingletonPtr();
  if (!root || !root->getHlmsManager())
    return 0;
  auto *pbs = CurrentPbs();
  if (!pbs)
  {
    gzerr << "[underwater] no PBS Hlms\n";
    return 0;
  }
  if (Live(pbs))
    return 1;
  // Installed in an Ogre that is gone, by a world before this one
  Drop();

  // Our pieces: each hook's must be there, or the shaders would insert
  // nothing while the world is marked as drawn. Ogre reads a folder that is
  // not there as an empty one.
  Ogre::Archive *ours = Ogre::ArchiveManager::getSingleton().load(
      _piecesDir, "FileSystem", true);
  if (!Defines(ours, kPbsHooks, "underwater"))
  {
    gzerr << "[underwater] the underwater pieces are not in [" << _piecesDir
          << "]; not installed\n";
    return 0;
  }

  Ogre::ArchiveVec library = pbs->getPiecesLibraryAsArchiveVec();
  std::vector<std::unique_ptr<MemoryArchive>> archives;
  if (!Chain(library, kPbsHooks, "underwater", archives))
    return 0;
  library.push_back(ours);

  s.originalLibrary = pbs->getPiecesLibraryAsArchiveVec();
  s.archives = std::move(archives);
  s.root = root;
  s.pbs = pbs;
  root->addFrameListener(&s.settler);
  s.listener.previous = pbs->getListener();
  pbs->setListener(&s.listener);
  pbs->reloadFrom(pbs->getDataFolder(), &library);
  s.done = true;
  gzmsg << "[underwater] installed in Ogre's PBS shaders\n";
  return 1;
}
catch (const std::exception &_e)
{
  gzerr << "[underwater] install failed: " << _e.what() << "\n";
  return 0;
}

//////////////////////////////////////////////////
int uw_ogre2_version()
{
  return UW_OGRE2_BRIDGE_VERSION;
}

//////////////////////////////////////////////////
void uw_ogre2_release()
{
  State().listener.released = true;
}

//////////////////////////////////////////////////
void uw_ogre2_set(const UwWater *_v)
{
  // A plugin sets the water: drawn again, if it was released
  State().listener.released = false;
  SetWater(_v);
}

//////////////////////////////////////////////////
/// \brief Put Ogre's PBS back as it was. Safe to call twice.
void uw_ogre2_uninstall()
try
{
  auto &s = State();
  if (!s.done)
    return;
  // Put back only into the Ogre we changed, if it is still there
  if (Live(CurrentPbs()))
  {
    s.listener.HideLamps(false);
    s.listener.backdrop.Destroy();
    s.pbs->setListener(s.listener.previous);
    s.pbs->reloadFrom(s.pbs->getDataFolder(), &s.originalLibrary);
    s.root->removeFrameListener(&s.settler);
  }
  Drop();
}
catch (...)
{
  Drop();
}
}  // extern "C"
