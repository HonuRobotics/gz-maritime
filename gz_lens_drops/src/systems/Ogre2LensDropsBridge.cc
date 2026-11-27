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
#include "Ogre2LensDropsBridge.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>
#include <gz/rendering/ogre2/Ogre2RenderPass.hh>

#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorNodeDef.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuadDef.h>
#include <OgreBitwise.h>
#include <OgreGpuProgramParams.h>
#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreHlmsDatablock.h>
#include <OgreHlmsSamplerblock.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpu.h>
#include <OgreRenderSystem.h>
#include <OgreResourceGroupManager.h>
#include <OgreRoot.h>
#include <OgreStagingTexture.h>
#include <OgreTechnique.h>
#include <OgreTextureBox.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>

namespace
{
/// \brief The largest map side.
constexpr int kMaxMapSize = 4096;

/// \brief What a handle and its pass share: the call back into the system,
/// cut when the handle goes.
struct Link
{
  /// \brief Held across a call, and when the call is cut.
  std::mutex mutex;

  /// \brief The system's function; null once the handle went.
  lens_drops_pull_t pull{nullptr};

  /// \brief Its context.
  void *context{nullptr};
};

/// \brief A full screen pass that refracts the image the passes before it
/// left through the water of a drop map.
class LensDropsPass : public gz::rendering::Ogre2RenderPass
{
  /// \brief Constructor.
  /// \param[in] _vs GLSL source of the vertex shader.
  /// \param[in] _fs GLSL source of the fragment shader.
  /// \param[in] _width Texels across the map.
  /// \param[in] _height Texels down the map.
  /// \param[in] _link The call back into the system.
  public: LensDropsPass(const std::string &_vs, const std::string &_fs,
                        unsigned int _width, unsigned int _height,
                        std::shared_ptr<Link> _link);

  /// \brief Destructor: frees what it made in Ogre. Render thread, as the
  /// camera that owns it goes.
  public: ~LensDropsPass() override;

  // Documentation inherited
  public: bool IsEnabled() const override;

  // Documentation inherited
  public: void PreRender(const gz::rendering::CameraPtr &_camera) override;

  // Documentation inherited
  public: void CreateRenderPass() override;

  /// \brief Texels across the map.
  public: const unsigned int width;

  /// \brief Texels down the map.
  public: const unsigned int height;

  /// \brief Give up: log once, draw nothing from now on.
  /// \param[in] _why What went wrong.
  private: void Fail(const std::string &_why);

  /// \brief Copy a map into the texture, as half floats.
  /// \param[in] _map width * height * 2 floats.
  private: void Upload(const float *_map);

  /// \brief Whether Ogre found a shader not to compile when it first drew
  /// it.
  /// \return True if one did not.
  private: bool CompileError() const;

  /// \brief GLSL source of the vertex shader.
  private: std::string vs;

  /// \brief GLSL source of the fragment shader.
  private: std::string fs;

  /// \brief The call back into the system.
  private: std::shared_ptr<Link> link;

  /// \brief This pass's material and programs. Null until created.
  private: Ogre::MaterialPtr material;
  private: Ogre::HighLevelGpuProgramPtr vsProgram;
  private: Ogre::HighLevelGpuProgramPtr fsProgram;

  /// \brief The map's texture. Null until created.
  private: Ogre::TextureGpu *texture{nullptr};

  /// \brief The texture manager, which owns the texture.
  private: Ogre::TextureGpuManager *manager{nullptr};

  /// \brief 0 blurred to 1 sharp, from the last pull.
  private: float focus{0.5f};

  /// \brief Whether the system wants the pass drawn, from the last pull.
  private: bool wanted{false};

  /// \brief Set once the pass gave up.
  private: bool failed{false};
};

/// \brief What a handle holds: the pass until a camera owns it, and the
/// link to cut.
struct Handle
{
  /// \brief The pass, until it is attached.
  std::shared_ptr<LensDropsPass> pass;

  /// \brief The call back into the system.
  std::shared_ptr<Link> link;
};

//////////////////////////////////////////////////
LensDropsPass::LensDropsPass(const std::string &_vs, const std::string &_fs,
    unsigned int _width, unsigned int _height, std::shared_ptr<Link> _link)
  : width(_width), height(_height), vs(_vs), fs(_fs), link(std::move(_link))
{
  // Off until the system says there is water to draw
  this->enabled = false;
}

//////////////////////////////////////////////////
LensDropsPass::~LensDropsPass()
{
  if (!Ogre::Root::getSingletonPtr())
    return;
  try
  {
    // The material, which samples the texture, first
    if (this->material)
      Ogre::MaterialManager::getSingleton().remove(this->material);
    if (this->manager && this->texture)
      this->manager->destroyTexture(this->texture);
    auto &programs = Ogre::HighLevelGpuProgramManager::getSingleton();
    if (this->vsProgram)
      programs.remove(this->vsProgram);
    if (this->fsProgram)
      programs.remove(this->fsProgram);
  }
  catch (...)
  {
    // Ogre may have shut its managers down first, on exit.
  }
}

//////////////////////////////////////////////////
void LensDropsPass::Fail(const std::string &_why)
{
  if (!this->failed)
  {
    gzerr << "[LensDrops] " << _why << "; the drops are not drawn\n";
  }
  this->failed = true;
}

//////////////////////////////////////////////////
bool LensDropsPass::CompileError() const
{
  return (this->vsProgram && this->vsProgram->hasCompileError()) ||
      (this->fsProgram && this->fsProgram->hasCompileError());
}

//////////////////////////////////////////////////
bool LensDropsPass::IsEnabled() const
{
  // Asked before the passes are chained for a frame: a shader that did not
  // compile in the frame before takes the pass out of the next one
  return this->wanted && !this->failed && !this->CompileError();
}

//////////////////////////////////////////////////
void LensDropsPass::PreRender(const gz::rendering::CameraPtr &_camera)
{
  if (this->failed || !_camera)
    return;
  try
  {
    if (this->CompileError())
      throw std::runtime_error("the drops' shaders do not compile");
    const float *map{nullptr};
    {
      const std::lock_guard<std::mutex> lock(this->link->mutex);
      if (!this->link->pull)
      {
        this->wanted = false;
        return;
      }
      const double stamp = std::chrono::duration<double>(
          _camera->Scene()->Time()).count();
      int enabled{0};
      map = this->link->pull(this->link->context, stamp, &this->focus,
                             &enabled);
      this->wanted = enabled != 0;
      // The map is the system's until its next pull, which this lock keeps
      // from coming: upload it now
      if (map && this->texture)
        this->Upload(map);
    }
    if (!this->material)
      return;
    Ogre::GpuProgramParametersSharedPtr params =
        this->material->getTechnique(0)->getPass(0)
        ->getFragmentProgramParameters();
    const double h = std::max(1u, _camera->ImageHeight());
    params->setNamedConstant("aspect",
        static_cast<Ogre::Real>(_camera->ImageWidth() / h));
    params->setNamedConstant("tanHalfH",
        static_cast<Ogre::Real>(std::tan(_camera->HFOV().Radian() / 2.0)));
    params->setNamedConstant("focus", static_cast<Ogre::Real>(this->focus));
    params->setNamedConstant("texel", Ogre::Vector2(
        1.0f / static_cast<float>(this->width),
        1.0f / static_cast<float>(this->height)));
  }
  catch (const std::exception &_e)
  {
    this->Fail(_e.what());
  }
}

//////////////////////////////////////////////////
void LensDropsPass::Upload(const float *_map)
{
  // Height and wet film as two half floats a texel: what the shader reads,
  // a quarter of four floats, converted in the one copy into the staging
  // texture
  this->texture->scheduleTransitionTo(Ogre::GpuResidency::Resident);
  Ogre::StagingTexture *staging = this->manager->getStagingTexture(
      this->width, this->height, 1u, 1u, Ogre::PFG_RG16_FLOAT);
  if (!staging)
    throw std::runtime_error("no staging texture for the drop map");
  staging->startMapRegion();
  Ogre::TextureBox box = staging->mapRegion(this->width, this->height,
      1u, 1u, Ogre::PFG_RG16_FLOAT);
  for (unsigned int row = 0; row < this->height; ++row)
  {
    auto *dst = reinterpret_cast<std::uint16_t *>(box.at(0, row, 0));
    const float *src = _map + static_cast<std::size_t>(row) * this->width * 2;
    for (unsigned int k = 0; k < this->width * 2; ++k)
      dst[k] = Ogre::Bitwise::floatToHalf(src[k]);
  }
  staging->stopMapRegion();
  staging->upload(box, this->texture, 0u);
  this->manager->removeStagingTexture(staging);
  if (!this->texture->isDataReady())
    this->texture->notifyDataIsReady();
}

//////////////////////////////////////////////////
void LensDropsPass::CreateRenderPass()
{
  if (!this->ogreCompositorNodeDefName.empty() || this->failed)
    return;
  try
  {
    // One material, texture and node definition per pass, so two cameras
    // never share drops.
    static int counter = 0;
    const std::string id = std::to_string(counter++);
    const std::string group =
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

    auto &programs = Ogre::HighLevelGpuProgramManager::getSingleton();
    this->vsProgram = programs.createProgram(
        "LensDropsVS_" + id, group, "glsl", Ogre::GPT_VERTEX_PROGRAM);
    this->vsProgram->setSource(this->vs);
    this->vsProgram->load();
    this->fsProgram = programs.createProgram(
        "LensDropsFS_" + id, group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
    this->fsProgram->setSource(this->fs);
    this->fsProgram->load();
    if (this->CompileError())
      throw std::runtime_error("the drops' shaders do not compile");

    const std::string materialName = "LensDrops_" + id;
    this->material =
        Ogre::MaterialManager::getSingleton().create(materialName, group);
    Ogre::Pass *pass = this->material->getTechnique(0)->getPass(0);
    Ogre::HlmsMacroblock macroblock;
    macroblock.mDepthCheck = false;
    macroblock.mDepthWrite = false;
    macroblock.mCullMode = Ogre::CULL_NONE;
    pass->setMacroblock(macroblock);
    pass->setVertexProgram(this->vsProgram->getName());
    pass->setFragmentProgram(this->fsProgram->getName());
    pass->getVertexProgramParameters()->setNamedAutoConstant("worldViewProj",
        Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);

    Ogre::HlmsSamplerblock samplerblock;
    samplerblock.setAddressingMode(Ogre::TAM_CLAMP);
    samplerblock.setFiltering(Ogre::TFO_BILINEAR);
    // Unit 0: the image so far, set by the compositor.
    pass->createTextureUnitState()->setSamplerblock(samplerblock);

    // Unit 1: the drop map, dry until the first pull with water.
    this->manager = gz::rendering::Ogre2RenderEngine::Instance()->OgreRoot()
        ->getRenderSystem()->getTextureGpuManager();
    this->texture = this->manager->createOrRetrieveTexture(
        "LensDropsMap_" + id, Ogre::GpuPageOutStrategy::SaveToSystemRam,
        Ogre::TextureFlags::ManualTexture, Ogre::TextureTypes::Type2D);
    this->texture->setResolution(this->width, this->height);
    this->texture->setNumMipmaps(1u);
    this->texture->setPixelFormat(Ogre::PFG_RG16_FLOAT);
    Ogre::TextureUnitState *unit = pass->createTextureUnitState();
    unit->setTexture(this->texture);
    unit->setSamplerblock(samplerblock);
    {
      std::vector<float> dry(static_cast<std::size_t>(this->width) *
                             this->height * 2, 0.0f);
      this->Upload(dry.data());
    }

    Ogre::GpuProgramParametersSharedPtr fsParams =
        pass->getFragmentProgramParameters();
    fsParams->setNamedConstant("RT", 0);
    fsParams->setNamedConstant("dropMap", 1);
    this->material->load();

    // The node gz-rendering's render target chains between the scene pass
    // and the final pass, as Ogre2GaussianNoisePass builds it:
    //
    // compositor_node LensDropsNode_<id>
    // {
    //   in 0 rt_input
    //   in 1 rt_output
    //   target rt_output
    //   {
    //     pass render_quad
    //     {
    //       material LensDrops_<id>
    //       input 0 rt_input
    //     }
    //   }
    //   out 0 rt_output
    //   out 1 rt_input
    // }
    Ogre::CompositorManager2 *compositorManager =
        gz::rendering::Ogre2RenderEngine::Instance()->OgreRoot()
        ->getCompositorManager2();
    const std::string nodeName = "LensDropsNode_" + id;
    Ogre::CompositorNodeDef *nodeDef =
        compositorManager->addNodeDefinition(nodeName);
    nodeDef->addTextureSourceName("rt_input", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    nodeDef->addTextureSourceName("rt_output", 1,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    nodeDef->setNumTargetPass(1);
    Ogre::CompositorTargetDef *targetDef =
        nodeDef->addTargetPass("rt_output");
    targetDef->setNumPasses(1);
    auto *quad = static_cast<Ogre::CompositorPassQuadDef *>(
        targetDef->addPass(Ogre::PASS_QUAD));
    quad->mMaterialName = materialName;
    quad->addQuadTextureSource(0, "rt_input");
    nodeDef->mapOutputChannel(0, "rt_output");
    nodeDef->mapOutputChannel(1, "rt_input");

    // Last, so a throw above leaves the pass uncreated.
    this->ogreCompositorNodeDefName = nodeName;
  }
  catch (const std::exception &_e)
  {
    this->Fail(_e.what());
  }
}
}  // namespace

extern "C"
{
//////////////////////////////////////////////////
int lens_drops_ogre2_abi_version(void)
{
  return LENS_DROPS_OGRE2_ABI;
}

//////////////////////////////////////////////////
lens_drops_pass_t lens_drops_ogre2_create(const char *_vs, const char *_fs,
    int _mapWidth, int _mapHeight, lens_drops_pull_t _pull, void *_context)
{
  if (!_vs || !_fs || !*_vs || !*_fs || !_pull || _mapWidth < 1 ||
      _mapHeight < 1 || _mapWidth > kMaxMapSize || _mapHeight > kMaxMapSize)
  {
    return nullptr;
  }
  try
  {
    auto handle = std::make_unique<Handle>();
    handle->link = std::make_shared<Link>();
    handle->link->pull = _pull;
    handle->link->context = _context;
    handle->pass = std::make_shared<LensDropsPass>(_vs, _fs,
        static_cast<unsigned int>(_mapWidth),
        static_cast<unsigned int>(_mapHeight), handle->link);
    return handle.release();
  }
  catch (...)
  {
    return nullptr;
  }
}

//////////////////////////////////////////////////
int lens_drops_ogre2_attach(lens_drops_pass_t _pass, void *_camera)
{
  if (!_pass || !_camera)
    return 0;
  auto *handle = static_cast<Handle *>(_pass);
  if (!handle->pass)
    return 0;
  try
  {
    auto *camera = static_cast<gz::rendering::Camera *>(_camera);
    camera->AddRenderPass(handle->pass);
    // The camera owns it now; the handle keeps only the link
    handle->pass.reset();
    return 1;
  }
  catch (const std::exception &_e)
  {
    gzerr << "[LensDrops] the camera did not take the pass: " << _e.what()
          << "\n";
  }
  catch (...)
  {
    gzerr << "[LensDrops] the camera did not take the pass\n";
  }
  return 0;
}

//////////////////////////////////////////////////
void lens_drops_ogre2_destroy(lens_drops_pass_t _pass)
{
  if (!_pass)
    return;
  auto *handle = static_cast<Handle *>(_pass);
  {
    // Waits for a pull in progress; none comes after
    const std::lock_guard<std::mutex> lock(handle->link->mutex);
    handle->link->pull = nullptr;
    handle->link->context = nullptr;
  }
  delete handle;
}
}  // extern "C"
