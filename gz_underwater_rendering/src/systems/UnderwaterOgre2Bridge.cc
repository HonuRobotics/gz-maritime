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

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
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
#include <OgreLight.h>
#include <OgreMatrix4.h>
#include <OgreSceneNode.h>

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
    (10u + kMaxLamps * kLampFloat4) * 4u * sizeof(float);

/// \brief Henyey-Greenstein asymmetry of the water's scattering, and the
/// beam's ray march.
constexpr float kPhaseG = 0.9f;
constexpr float kBeamSteps = 12.0f;
constexpr float kBeamRange = 20.0f;

/// \brief A lamp dimmer than this (its power scale times its brightest
/// diffuse channel) counts as off: the shader neither lights with it nor
/// marches its beam. With every lamp off the lamp code is compiled out.
constexpr float kLampMinPower = 0.01f;

class UnderwaterListener : public Ogre::HlmsListener
{
  public: Ogre::HlmsListener *previous{nullptr};
  public: Water water;

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
           static_cast<int>(this->lamps.size()) < kMaxLamps)
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
    put4(static_cast<float>(this->lamps.size()), kBeamSteps, kBeamRange,
         1.0f);
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

/// \brief Everything installed, to put back.
struct Installed
{
  Ogre::HlmsPbs *pbs{nullptr};
  UnderwaterListener listener;
  Ogre::ArchiveVec originalLibrary;
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
  Ogre::Archive *gzPbs = nullptr;
  std::size_t gzPbsIndex = 0;
  for (std::size_t i = 0; i < library.size(); ++i)
  {
    const std::string name = library[i]->getName();
    if (name.size() >= 7 && name.compare(name.size() - 7, 7, "/Gz/Pbs") == 0)
    {
      gzPbs = library[i];
      gzPbsIndex = i;
    }
  }
  if (!gzPbs)
  {
    gzerr << "[underwater] gz-rendering's Hlms/Gz/Pbs pieces not found; "
          << "this gz-rendering is not supported\n";
    return 0;
  }

  // A copy of gz's composite pieces, with ours inserted.
  s.workDir = std::filesystem::temp_directory_path() /
      ("gz_underwater_" + std::to_string(::getpid())) / "GzPbs";
  std::filesystem::create_directories(s.workDir);
  bool passBuffer = false, posExecution = false;
  // Keep the list alive: the range-for would only extend the vector's
  // reference, not the shared pointer that owns it.
  const Ogre::StringVectorPtr files = gzPbs->list(false, false);
  for (const auto &file : *files)
  {
    std::string text = gzPbs->open(file)->getAsString();
    passBuffer |= InsertIntoPiece(text, "custom_passBuffer",
        "@insertpiece( custom_passBuffer_underwater )");
    posExecution |= InsertIntoPiece(text, "custom_ps_posExecution",
        "@insertpiece( custom_ps_posExecution_underwater )");
    std::ofstream(s.workDir / file) << text;
  }
  if (!passBuffer || !posExecution)
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
  return 1;
}
catch (const std::exception &_e)
{
  gzerr << "[underwater] install failed: " << _e.what() << "\n";
  return 0;
}

//////////////////////////////////////////////////
/// \brief Set the water: 14 floats, beta^D, beta^B, the veil at the surface
/// and K_d (each r g b), then the still water level and the daylight. A
/// null pointer turns the water off.
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
  auto &w = s.listener.water;
  w.active = nullptr != _v;
  // Lamps that appear later (a vehicle spawned with its lights) are hidden
  // on the frame they appear.
  s.listener.HideLamps(w.active);
  if (!_v)
    return;
  std::memcpy(w.attenuation, _v, 3 * sizeof(float));
  std::memcpy(w.backscatter, _v + 3, 3 * sizeof(float));
  std::memcpy(w.veil, _v + 6, 3 * sizeof(float));
  std::memcpy(w.diffuse, _v + 9, 3 * sizeof(float));
  w.level = _v[12];
  w.daylight = _v[13];
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
    s.pbs->setListener(s.listener.previous);
    s.pbs->reloadFrom(s.pbs->getDataFolder(), &s.originalLibrary);
  }
  std::error_code ec;
  std::filesystem::remove_all(s.workDir.parent_path(), ec);
  s.done = false;
  s.pbs = nullptr;
}
catch (...)
{
}
}  // extern "C"
