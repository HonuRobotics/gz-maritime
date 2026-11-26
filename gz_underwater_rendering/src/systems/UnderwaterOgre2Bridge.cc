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

#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/rendering/config.hh>

#include <OgreArchive.h>
#include <OgreArchiveManager.h>
#include <OgreDataStream.h>
#include <OgreCamera.h>
#include <OgreHlms.h>
#include <OgreHlmsListener.h>
#include <OgreHlmsManager.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreHlmsPbs.h>
#include <OgreString.h>

namespace
{
/// \brief The gz-rendering major version whose pieces this was written
/// against.
constexpr int kGzRenderingMajor = 10;

/// \brief Ogre's customization hooks we add to: the pass buffer block and
/// the final colour.
const std::vector<std::string> kPbsHooks{"custom_passBuffer",
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
  /// \brief The pieces we changed or added, read by the Hlms
  std::vector<std::unique_ptr<MemoryArchive>> archives;
  bool done{false};
};
Installed &State()
{
  static Installed installed;
  return installed;
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

  if (GZ_RENDERING_MAJOR_VERSION != kGzRenderingMajor)
  {
    gzerr << "[underwater] written against gz-rendering "
          << kGzRenderingMajor << ", not " << GZ_RENDERING_MAJOR_VERSION
          << "; not installed\n";
    return 0;
  }

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
  s.archives.clear();
  s.done = false;
  s.pbs = nullptr;
}
catch (...)
{
}
}  // extern "C"
