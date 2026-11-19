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
// * gz-rendering's own composite pieces (Hlms/Gz/Pbs): its
//   custom_passBuffer and custom_ps_posExecution get one more insertpiece
//   each, ours, written into a copy of that folder read at run time, so the
//   rest is always the installed gz-rendering's own;
// * gz-rendering's Hlms listener: ours calls it first, then appends.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

#include <gz/common/Console.hh>

#include <OgreArchive.h>
#include <OgreArchiveManager.h>
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
#include <OgrePixelFormatGpuUtils.h>
#include <OgreRenderPassDescriptor.h>
#include <OgreRenderSystem.h>
#include <OgreSceneNode.h>
#include <OgreSubItem.h>
#include <OgreSubMesh2.h>
#include <OgreTextureGpu.h>
#include <Vao/OgreVaoManager.h>

namespace
{
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
  float time{0};
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
/// (2) and the lamps.
constexpr Ogre::uint32 kBlockBytes =
    (11u + kMaxLamps * kLampFloat4) * 4u * sizeof(float);

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

/// \brief How many cameras the waves are followed for.
constexpr int kMaxCameras = 32;

/// \brief The waves above each camera. The shader measures depth below the
/// water's surface over the camera, as the reference camera does: the still
/// level plus the wave there. Only the plugin can ask the wave field, and
/// only this side knows where the cameras are, so each pass leaves its
/// camera's place here, the plugin reads the list (List) and gives back the
/// wave's height over each (Set). A camera's height is that of the place it
/// had a frame ago, at the time the plugin last asked.
class Surface
{
  /// \brief The wave's height over a camera, m above the still water; 0
  /// until the plugin has answered. Call in each pass the camera draws.
  public: float Elevation(const Ogre::Camera *_camera,
                          const Ogre::Vector3 &_position)
  {
    auto it = this->cameras.find(_camera);
    if (it == this->cameras.end())
    {
      if (static_cast<int>(this->cameras.size()) >= kMaxCameras)
        return 0.0f;
      it = this->cameras.emplace(_camera, Entry{}).first;
      it->second.id = this->nextId++;
    }
    it->second.x = static_cast<float>(_position.x);
    it->second.y = static_cast<float>(_position.y);
    it->second.idle = 0;
    return it->second.elevation;
  }

  /// \brief The cameras that drew lately: id, x, y each.
  /// \return How many.
  public: int List(float *_out, int _max)
  {
    int n = 0;
    for (auto it = this->cameras.begin(); it != this->cameras.end();)
    {
      // A camera that has not drawn for a while is gone.
      if (++it->second.idle > 300)
      {
        it = this->cameras.erase(it);
        continue;
      }
      if (n < _max)
      {
        _out[3 * n] = static_cast<float>(it->second.id);
        _out[3 * n + 1] = it->second.x;
        _out[3 * n + 2] = it->second.y;
        ++n;
      }
      ++it;
    }
    return n;
  }

  /// \brief The wave's height over the cameras: id, height each. A camera
  /// not in the list is under still water.
  public: void Set(const float *_idElevation, int _n)
  {
    for (auto &[camera, entry] : this->cameras)
    {
      entry.elevation = 0.0f;
      for (int i = 0; i < _n; ++i)
      {
        if (static_cast<int>(std::lround(_idElevation[2 * i])) == entry.id)
          entry.elevation = _idElevation[2 * i + 1];
      }
    }
  }

  public: void Clear() { this->cameras.clear(); }

  private: struct Entry
  {
    int id{0};
    float x{0};
    float y{0};
    float elevation{0};
    int idle{0};
  };
  private: std::map<const Ogre::Camera *, Entry> cameras;
  private: int nextId{1};
};

/// \brief Water behind everything: a black box around the world, seen from
/// inside, drawn after the sky and at the far plane wherever nothing else
/// is. Through the water it is the water's own colour, the veil, as open
/// water is; without it a camera under water would see the sky or the
/// background colour past the last thing in view. The shader makes it
/// clear (alpha 0) for cameras above the water.
class Backdrop
{
  /// \brief Show it in every scene, around the water level, or hide it.
  public: void Update(bool _show, float _level)
  {
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
    // After the sky, which gz-rendering draws between its queues 1 and 2
    // wherever the depth is still the far plane's.
    item->setRenderQueueGroup(2u);
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

  /// \brief Set when the box could not be made: not tried again.
  private: bool failed{false};
};

class UnderwaterListener : public Ogre::HlmsListener
{
  public: Ogre::HlmsListener *previous{nullptr};
  public: Water water;
  public: Backdrop backdrop;
  public: Surface surface;

  private: bool On(bool _caster) const
  { return this->water.active && !_caster; }

  /// \brief The scene's lamps (spot and point lights), as of the last
  /// preparePassHash.
  private: std::vector<const Ogre::Light *> lamps;

  /// \brief Lamps this listener hid from Ogre's own lighting, so that the
  /// shader lights with them once, through the water.
  public: std::set<Ogre::Light *> hidden;

  /// \brief Hide the scene's lamps from Ogre's lighting (the shader lights
  /// with them), or give them back. Call before rendering.
  public: void HideLamps(bool _hide)
  {
    Ogre::Root *root = Ogre::Root::getSingletonPtr();
    if (!root)
      return;
    if (!_hide)
    {
      for (auto *l : this->hidden)
        l->setVisible(true);
      this->hidden.clear();
      return;
    }
    auto managers = root->getSceneManagerIterator();
    while (managers.hasMoreElements())
    {
      Ogre::SceneManager *sm = managers.getNext();
      auto it = sm->getMovableObjectIterator(
          Ogre::LightFactory::FACTORY_TYPE_NAME);
      while (it.hasMoreElements())
      {
        auto *l = static_cast<Ogre::Light *>(it.getNext());
        if ((l->getType() == Ogre::Light::LT_SPOTLIGHT ||
             l->getType() == Ogre::Light::LT_POINT) && l->isVisible())
        {
          l->setVisible(false);
          this->hidden.insert(l);
        }
      }
    }
  }

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
    // The water's surface over the camera: the still level and the wave.
    const float level = w.level +
        (camera ? this->surface.Elevation(camera, pos) : 0.0f);
    const float depth = std::max(0.0f, level - static_cast<float>(pos.z));
    const auto put4 = [&_ptr](float _a, float _b, float _c, float _d)
    {
      *_ptr++ = _a; *_ptr++ = _b; *_ptr++ = _c; *_ptr++ = _d;
    };
    put4(pos.x, pos.y, pos.z, level);
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
    put4(w.caustics, w.snow, w.time, kSnowRange[w.quality]);
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

/// \brief Bytes the underwater block adds to the unlit pass buffer: the
/// camera, three of the water's colours and three vectors.
constexpr Ogre::uint32 kUnlitBlockBytes = 7u * 4u * sizeof(float);

/// \brief The water in Ogre's unlit shaders (particles): what nothing
/// lights fades over the range and takes the veil, like everything else.
/// The water and the cameras' levels are the PBS listener's.
class UnlitListener : public Ogre::HlmsListener
{
  public: Ogre::HlmsListener *previous{nullptr};
  public: UnderwaterListener *shared{nullptr};

  /// \brief Whether gz-rendering's own pieces are in the unlit shaders, so
  /// that what its listener adds to the pass buffer is declared there.
  public: bool chained{false};

  /// \brief Whether the water is drawn in a pass. When gz-rendering's
  /// pieces are not in the shaders, what its listener adds to the pass
  /// buffer (the lidar's spherical clipping) is not declared before our
  /// block, which would then be read from the wrong place: such a pass,
  /// which is not a colour image, is left alone.
  private: bool On(const Ogre::CompositorShadowNode *_shadow, bool _caster,
                   bool _dual, Ogre::SceneManager *_sm) const
  {
    if (!this->shared || !this->shared->water.active || _caster)
      return false;
    return this->chained || !this->previous ||
        this->previous->getPassBufferSize(_shadow, _caster, _dual, _sm) == 0u;
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
    if (!this->On(_shadow, _caster, _dual, _sm))
      return;
    _hlms->_setProperty("gz_underwater", 1);
    // Ogre's unlit shaders write a colour as it is and do not say whether
    // the image keeps linear light (an sRGB image, as gz-rendering's are).
    const Ogre::RenderPassDescriptor *pass = Ogre::Root::getSingleton()
        .getRenderSystem()->getCurrentPassDescriptor();
    if (pass && pass->getNumColourEntries() > 0u && pass->mColour[0].texture &&
        Ogre::PixelFormatGpuUtils::isSRgb(
            pass->mColour[0].texture->getPixelFormat()))
    {
      _hlms->_setProperty("gz_underwater_linear", 1);
    }
  }

  public: Ogre::uint32 getPassBufferSize(
      const Ogre::CompositorShadowNode *_shadow, bool _caster, bool _dual,
      Ogre::SceneManager *_sm) const override
  {
    const Ogre::uint32 size = this->previous ?
        this->previous->getPassBufferSize(_shadow, _caster, _dual, _sm) : 0u;
    return size + (this->On(_shadow, _caster, _dual, _sm) ? kUnlitBlockBytes : 0u);
  }

  public: float *preparePassBuffer(const Ogre::CompositorShadowNode *_shadow,
      bool _caster, bool _dual, Ogre::SceneManager *_sm,
      float *_ptr) override
  {
    if (this->previous)
      _ptr = this->previous->preparePassBuffer(_shadow, _caster, _dual, _sm,
                                               _ptr);
    if (!this->On(_shadow, _caster, _dual, _sm))
      return _ptr;
    const Ogre::Camera *camera = _sm->getCamerasInProgress().renderingCamera;
    Ogre::Vector3 pos = Ogre::Vector3::ZERO;
    // From the camera to a point, in the world, out of its clip-space x, y
    // and w: for a perspective camera w is the depth along its axis, so
    // the view-space point is ((x + P02 w) / P00, (y + P12 w) / P11, -w),
    // whatever the depth buffer's convention. Zero for an orthographic
    // camera, which then draws what is unlit as in air.
    Ogre::Vector3 clipX = Ogre::Vector3::ZERO;
    Ogre::Vector3 clipY = Ogre::Vector3::ZERO;
    Ogre::Vector3 clipW = Ogre::Vector3::ZERO;
    if (camera)
    {
      pos = camera->getDerivedPosition();
      Ogre::Matrix4 projection = camera->getProjectionMatrixWithRSDepth();
      // Images drawn to a texture are flipped, as Ogre's unlit shaders do.
      const Ogre::RenderPassDescriptor *pass = Ogre::Root::getSingleton()
          .getRenderSystem()->getCurrentPassDescriptor();
      if (pass && pass->requiresTextureFlipping())
      {
        for (int c = 0; c < 4; ++c)
          projection[1][c] = -projection[1][c];
      }
      if (camera->getProjectionType() == Ogre::PT_PERSPECTIVE &&
          std::abs(projection[0][0]) > 1e-9f &&
          std::abs(projection[1][1]) > 1e-9f)
      {
        Ogre::Matrix3 rot;
        camera->getDerivedOrientation().ToRotationMatrix(rot);
        const Ogre::Vector3 right = rot.GetColumn(0);
        const Ogre::Vector3 up = rot.GetColumn(1);
        const Ogre::Vector3 back = rot.GetColumn(2);
        clipX = right / projection[0][0];
        clipY = up / projection[1][1];
        clipW = right * (projection[0][2] / projection[0][0]) +
                up * (projection[1][2] / projection[1][1]) - back;
      }
    }
    const auto &w = this->shared->water;
    const float level = w.level +
        (camera ? this->shared->surface.Elevation(camera, pos) : 0.0f);
    const float depth = std::max(0.0f, level - static_cast<float>(pos.z));
    const auto put4 = [&_ptr](float _a, float _b, float _c, float _d)
    {
      *_ptr++ = _a; *_ptr++ = _b; *_ptr++ = _c; *_ptr++ = _d;
    };
    put4(pos.x, pos.y, pos.z, level);
    put4(w.attenuation[0], w.attenuation[1], w.attenuation[2], 0.0f);
    put4(w.backscatter[0], w.backscatter[1], w.backscatter[2], 0.0f);
    put4(w.daylight * w.veil[0] * std::exp(-w.diffuse[0] * depth),
         w.daylight * w.veil[1] * std::exp(-w.diffuse[1] * depth),
         w.daylight * w.veil[2] * std::exp(-w.diffuse[2] * depth), 0.0f);
    put4(clipX.x, clipX.y, clipX.z, 0.0f);
    put4(clipY.x, clipY.y, clipY.z, 0.0f);
    put4(clipW.x, clipW.y, clipW.z, 0.0f);
    return _ptr;
  }

  public: void hlmsTypeChanged(bool _caster, Ogre::CommandBuffer *_cmd,
      const Ogre::HlmsDatablock *_db, size_t _texUnit) override
  {
    if (this->previous)
      this->previous->hlmsTypeChanged(_caster, _cmd, _db, _texUnit);
  }
};

/// \brief Everything installed, to put back.
struct Installed
{
  Ogre::HlmsPbs *pbs{nullptr};
  UnderwaterListener listener;
  Ogre::ArchiveVec originalLibrary;
  /// \brief Ogre's unlit shaders, when the water went into them too.
  Ogre::Hlms *unlit{nullptr};
  UnlitListener unlitListener;
  Ogre::ArchiveVec originalUnlitLibrary;
  std::filesystem::path workDir;
  bool done{false};
};
Installed &State()
{
  static Installed installed;
  return installed;
}

/// \brief Insert a line before the @end that closes a piece.
bool InsertIntoPiece(std::string &_text, const std::string &_piece,
                     const std::string &_line)
{
  const std::string open = "@piece( " + _piece + " )";
  const auto start = _text.find(open);
  if (start == std::string::npos)
    return false;
  const auto end = _text.find("@end", start + open.size());
  if (end == std::string::npos)
    return false;
  _text.insert(end, "  " + _line + "\n");
  return true;
}

/// \brief A copy of gz-rendering's composite pieces with one more
/// insertpiece in each of the pieces named.
/// \param[in] _inserts The piece and the line to add to it, for each.
/// \return False if one of the pieces is not there.
bool CopyWithInserts(Ogre::Archive *_gzPbs,
    const std::filesystem::path &_dir,
    const std::vector<std::pair<std::string, std::string>> &_inserts)
{
  std::filesystem::create_directories(_dir);
  std::vector<bool> found(_inserts.size(), false);
  // Keep the list alive: the range-for would only extend the vector's
  // reference, not the shared pointer that owns it.
  const Ogre::StringVectorPtr files = _gzPbs->list(false, false);
  for (const auto &file : *files)
  {
    std::string text = _gzPbs->open(file)->getAsString();
    for (std::size_t i = 0; i < _inserts.size(); ++i)
    {
      if (InsertIntoPiece(text, _inserts[i].first,
                          "@insertpiece( " + _inserts[i].second + " )"))
      {
        found[i] = true;
      }
    }
    std::ofstream(_dir / file) << text;
  }
  return std::find(found.begin(), found.end(), false) == found.end();
}

/// \brief gz-rendering's composite pieces in a pieces library.
/// \return Its place in the library, or its size if it is not there.
std::size_t FindGzPbs(const Ogre::ArchiveVec &_library)
{
  std::size_t index = _library.size();
  for (std::size_t i = 0; i < _library.size(); ++i)
  {
    const std::string name = _library[i]->getName();
    if (name.size() >= 7 && name.compare(name.size() - 7, 7, "/Gz/Pbs") == 0)
      index = i;
  }
  return index;
}

/// \brief Put the water into Ogre's unlit shaders too (particles). They
/// read gz-rendering's same composite pieces; the unlit pieces are in the
/// folder next to the PBS ones, named with Unlit at the end. Without it
/// the rest still works: what is unlit is drawn as in air.
void InstallUnlit(const char *_piecesDir)
try
{
  auto &s = State();
  Ogre::Hlms *unlit = Ogre::Root::getSingleton().getHlmsManager()->getHlms(
      Ogre::HLMS_UNLIT);
  const std::string pieces = std::string(_piecesDir) + "Unlit";
  if (!unlit || !std::filesystem::is_directory(pieces))
  {
    gzwarn << "[underwater] no unlit Hlms or no pieces in [" << pieces
           << "]: unlit materials are drawn as in air\n";
    return;
  }
  Ogre::ArchiveVec library = unlit->getPiecesLibraryAsArchiveVec();
  s.originalUnlitLibrary = library;
  const std::vector<std::pair<std::string, std::string>> inserts = {
      {"custom_passBuffer", "custom_passBuffer_underwater_unlit"},
      {"custom_VStoPS", "custom_VStoPS_underwater_unlit"},
      {"custom_vs_posExecution", "custom_vs_posExecution_underwater_unlit"},
      {"custom_ps_posExecution", "custom_ps_posExecution_underwater_unlit"}};
  const std::size_t index = FindGzPbs(library);
  const std::filesystem::path dir = s.workDir.parent_path() / "GzUnlit";
  auto &archives = Ogre::ArchiveManager::getSingleton();
  if (index != library.size())
  {
    // gz-rendering's composite pieces are in the unlit shaders: chain.
    if (!CopyWithInserts(library[index], dir, inserts))
    {
      gzwarn << "[underwater] gz-rendering's pieces for the unlit shaders "
             << "not as expected: unlit materials are drawn as in air\n";
      return;
    }
    library[index] = archives.load(dir.string(), "FileSystem", true);
  }
  else
  {
    // This gz-rendering gives the unlit shaders no pieces of its own (its
    // Hlms/Gz folder holds none directly): the composite pieces are ours.
    std::filesystem::create_directories(dir);
    std::ofstream file(dir / "500.UnderwaterUnlitComposite_piece_all.any");
    for (const auto &[piece, ours] : inserts)
    {
      file << "@piece( " << piece << " )\n  @insertpiece( " << ours
           << " )\n@end\n";
    }
    file.close();
    library.push_back(archives.load(dir.string(), "FileSystem", true));
  }
  s.unlitListener.chained = index < s.originalUnlitLibrary.size();
  library.push_back(archives.load(pieces, "FileSystem", true));
  s.unlitListener.shared = &s.listener;
  s.unlitListener.previous = unlit->getListener();
  unlit->setListener(&s.unlitListener);
  unlit->reloadFrom(unlit->getDataFolder(), &library);
  s.unlit = unlit;
  gzmsg << "[underwater] installed in Ogre's unlit shaders\n";
}
catch (const std::exception &_e)
{
  gzwarn << "[underwater] not installed in the unlit shaders: " << _e.what()
         << "\n";
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
  if (s.done)
    return 1;
  Ogre::Root *root = Ogre::Root::getSingletonPtr();
  if (!root || !root->getHlmsManager())
    return 0;
  auto *pbs = dynamic_cast<Ogre::HlmsPbs *>(
      root->getHlmsManager()->getHlms(Ogre::HLMS_PBS));
  if (!pbs)
  {
    gzerr << "[underwater] no PBS Hlms\n";
    return 0;
  }

  Ogre::ArchiveVec library = pbs->getPiecesLibraryAsArchiveVec();
  s.originalLibrary = library;
  const std::size_t gzPbsIndex = FindGzPbs(library);
  if (gzPbsIndex == library.size())
  {
    gzerr << "[underwater] gz-rendering's Hlms/Gz/Pbs pieces not found; "
          << "this gz-rendering is not supported\n";
    return 0;
  }

  // A copy of gz's composite pieces, with ours inserted.
  s.workDir = std::filesystem::temp_directory_path() /
      ("gz_underwater_" + std::to_string(::getpid())) / "GzPbs";
  if (!CopyWithInserts(library[gzPbsIndex], s.workDir,
          {{"custom_passBuffer", "custom_passBuffer_underwater"},
           {"custom_ps_posExecution", "custom_ps_posExecution_underwater"}}))
  {
    gzerr << "[underwater] gz-rendering's custom_passBuffer or "
          << "custom_ps_posExecution piece not found; not installed\n";
    std::filesystem::remove_all(s.workDir.parent_path());
    return 0;
  }

  auto &archives = Ogre::ArchiveManager::getSingleton();
  library[gzPbsIndex] = archives.load(s.workDir.string(), "FileSystem", true);
  library.push_back(archives.load(_piecesDir, "FileSystem", true));

  s.pbs = pbs;
  s.listener.previous = pbs->getListener();
  pbs->setListener(&s.listener);
  pbs->reloadFrom(pbs->getDataFolder(), &library);
  s.done = true;
  gzmsg << "[underwater] installed in Ogre's PBS shaders\n";
  InstallUnlit(_piecesDir);
  return 1;
}
catch (const std::exception &_e)
{
  gzerr << "[underwater] install failed: " << _e.what() << "\n";
  return 0;
}

//////////////////////////////////////////////////
/// \brief Set the water: 18 floats, beta^D, beta^B, the veil at the surface
/// and K_d (each r g b), then the still water level (the waves over each
/// camera come with uw_ogre2_set_elevations), the daylight, the
/// caustics' strength (already scaled by the waves), the marine snow's
/// density, the time, s, and the quality (0 low, 1 medium, 2 high). A null
/// pointer turns the water off.
void uw_ogre2_set(const float *_v)
{
  auto &s = State();
  // gz-rendering sets its own Hlms listener again when its scene changes
  // (a light added, for one); wrap whichever is there now.
  if (s.done && s.pbs && s.pbs->getListener() != &s.listener)
  {
    s.listener.previous = s.pbs->getListener();
    s.pbs->setListener(&s.listener);
    gzmsg << "[underwater] listener re-attached after gz-rendering replaced "
          << "it\n";
  }
  if (s.done && s.unlit && s.unlit->getListener() != &s.unlitListener)
  {
    s.unlitListener.previous = s.unlit->getListener();
    s.unlit->setListener(&s.unlitListener);
  }
  auto &w = s.listener.water;
  w.active = nullptr != _v;
  // Lamps that appear later (a vehicle spawned with its lights) are hidden
  // on the frame they appear.
  s.listener.HideLamps(w.active);
  s.listener.backdrop.Update(s.done && w.active, _v ? _v[12] : 0.0f);
  if (!_v)
    return;
  std::memcpy(w.attenuation, _v, 3 * sizeof(float));
  std::memcpy(w.backscatter, _v + 3, 3 * sizeof(float));
  std::memcpy(w.veil, _v + 6, 3 * sizeof(float));
  std::memcpy(w.diffuse, _v + 9, 3 * sizeof(float));
  w.level = _v[12];
  w.daylight = _v[13];
  w.caustics = _v[14];
  w.snow = _v[15];
  w.time = _v[16];
  w.quality = std::clamp(static_cast<int>(std::lround(_v[17])), 0, 2);
}

//////////////////////////////////////////////////
/// \brief The cameras that drew lately, for the plugin to ask the wave
/// field about: an id, x and y for each (3 floats).
/// \param[out] _out Room for 3 * _max floats.
/// \return How many cameras were written.
int uw_ogre2_cameras(float *_out, int _max)
{
  return State().listener.surface.List(_out, _max);
}

//////////////////////////////////////////////////
/// \brief The wave's height over the cameras, m above the still water: an
/// id (from uw_ogre2_cameras) and the height for each (2 floats). A camera
/// left out is under still water.
void uw_ogre2_set_elevations(const float *_idElevation, int _n)
{
  State().listener.surface.Set(_idElevation, _n);
}

//////////////////////////////////////////////////
/// \brief Put Ogre's PBS back as it was. Safe to call twice.
void uw_ogre2_uninstall()
try
{
  auto &s = State();
  if (!s.done)
    return;
  if (Ogre::Root::getSingletonPtr() && s.pbs)
  {
    s.listener.HideLamps(false);
    s.listener.backdrop.Destroy();
    s.listener.surface.Clear();
    s.pbs->setListener(s.listener.previous);
    s.pbs->reloadFrom(s.pbs->getDataFolder(), &s.originalLibrary);
    if (s.unlit)
    {
      s.unlit->setListener(s.unlitListener.previous);
      s.unlit->reloadFrom(s.unlit->getDataFolder(), &s.originalUnlitLibrary);
    }
  }
  std::error_code ec;
  std::filesystem::remove_all(s.workDir.parent_path(), ec);
  s.done = false;
  s.pbs = nullptr;
  s.unlit = nullptr;
}
catch (...)
{
}
}  // extern "C"
