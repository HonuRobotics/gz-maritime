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
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
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
  bool active{false};
};

/// \brief Bytes the underwater block adds: eight float4.
constexpr Ogre::uint32 kBlockBytes = 8u * 4u * sizeof(float);

class UnderwaterListener : public Ogre::HlmsListener
{
  public: Ogre::HlmsListener *previous{nullptr};
  public: Water water;

  private: bool On(bool _caster) const
  { return this->water.active && !_caster; }

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
    if (this->On(_caster))
      _hlms->_setProperty("gz_underwater", 1);
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
    put4(w.veil[0] * std::exp(-w.diffuse[0] * depth),
         w.veil[1] * std::exp(-w.diffuse[1] * depth),
         w.veil[2] * std::exp(-w.diffuse[2] * depth), 0.0f);
    put4(w.diffuse[0], w.diffuse[1], w.diffuse[2], 0.0f);
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
/// \brief Set the water: 13 floats, beta^D, beta^B, the veil at the surface
/// and K_d (each r g b), then the still water level. A null pointer turns
/// the water off.
void uw_ogre2_set(const float *_v)
{
  auto &w = State().listener.water;
  w.active = nullptr != _v;
  if (!_v)
    return;
  std::memcpy(w.attenuation, _v, 3 * sizeof(float));
  std::memcpy(w.backscatter, _v + 3, 3 * sizeof(float));
  std::memcpy(w.veil, _v + 6, 3 * sizeof(float));
  std::memcpy(w.diffuse, _v + 9, 3 * sizeof(float));
  w.level = _v[12];
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
