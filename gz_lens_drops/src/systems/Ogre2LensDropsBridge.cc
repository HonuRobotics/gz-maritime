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
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/ogre2/Ogre2RenderEngine.hh>
#include <gz/rendering/ogre2/Ogre2RenderPass.hh>

#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorNodeDef.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuadDef.h>
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
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>

namespace
{
/// \brief The largest map side.
constexpr int kMaxMapSize = 4096;

/// \brief A full screen pass that refracts the image the passes before it
/// left through the water of a drop map.
class LensDropsPass : public gz::rendering::Ogre2RenderPass
{
  /// \brief Constructor.
  /// \param[in] _vs GLSL source of the vertex shader.
  /// \param[in] _fs GLSL source of the fragment shader.
  /// \param[in] _width Texels across the map.
  /// \param[in] _height Texels down the map.
  public: LensDropsPass(const std::string &_vs, const std::string &_fs,
                        unsigned int _width, unsigned int _height);

  /// \brief Destructor: frees the texture.
  public: ~LensDropsPass() override;

  // Documentation inherited
  public: void PreRender(const gz::rendering::CameraPtr &_camera) override;

  // Documentation inherited
  public: void CreateRenderPass() override;

  /// \brief The map to upload, 4 floats a texel.
  public: std::vector<float> map;

  /// \brief True when map holds data not uploaded yet.
  public: bool dirty{false};

  /// \brief 0 blurred to 1 sharp.
  public: float focus{0.5f};

  /// \brief Texels across the map.
  public: const unsigned int width;

  /// \brief Texels down the map.
  public: const unsigned int height;

  /// \brief Copy the map into the texture.
  private: void Upload();

  /// \brief GLSL source of the vertex shader.
  private: std::string vs;

  /// \brief GLSL source of the fragment shader.
  private: std::string fs;

  /// \brief This pass's material, owned by Ogre. Null until created.
  private: Ogre::Material *material{nullptr};

  /// \brief The map's texture. Null until created.
  private: Ogre::TextureGpu *texture{nullptr};

  /// \brief The texture unit that samples it.
  private: Ogre::TextureUnitState *unit{nullptr};

  /// \brief The texture manager, which owns the texture.
  private: Ogre::TextureGpuManager *manager{nullptr};
};

/// \brief What a handle points at: the pointer the camera shares.
using PassPtr = std::shared_ptr<LensDropsPass>;

//////////////////////////////////////////////////
LensDropsPass::LensDropsPass(const std::string &_vs, const std::string &_fs,
    unsigned int _width, unsigned int _height)
  : width(_width), height(_height), vs(_vs), fs(_fs)
{
  this->map.assign(static_cast<std::size_t>(_width) * _height * 4, 0.0f);
  this->dirty = true;
}

//////////////////////////////////////////////////
LensDropsPass::~LensDropsPass()
{
  try
  {
    if (this->unit)
      this->unit->setTexture(nullptr);
    if (this->manager && this->texture)
      this->manager->destroyTexture(this->texture);
  }
  catch (...)
  {
    // Ogre may have shut its texture manager down first, on exit.
  }
}

//////////////////////////////////////////////////
void LensDropsPass::PreRender(const gz::rendering::CameraPtr &_camera)
{
  if (!this->material || !this->enabled || !_camera)
    return;
  if (this->dirty && this->texture)
    this->Upload();

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

//////////////////////////////////////////////////
void LensDropsPass::Upload()
{
  // The staging sequence of gz_waves_rendering's Ogre2HeightMapBridge:
  // acquire a staging texture for this upload and give it back at once.
  this->texture->scheduleTransitionTo(Ogre::GpuResidency::Resident, nullptr);
  Ogre::StagingTexture *staging = this->manager->getStagingTexture(
      this->width, this->height, 1u, 1u, Ogre::PFG_RGBA32_FLOAT);
  if (!staging)
    return;
  staging->startMapRegion();
  const Ogre::TextureBox box = staging->mapRegion(this->width, this->height,
      1u, 1u, Ogre::PFG_RGBA32_FLOAT);
  const std::size_t rowFloats = static_cast<std::size_t>(this->width) * 4;
  for (unsigned int row = 0; row < this->height; ++row)
  {
    auto *dst = reinterpret_cast<float *>(box.at(0, row, 0));
    std::copy(this->map.begin() + static_cast<std::ptrdiff_t>(row * rowFloats),
              this->map.begin() + static_cast<std::ptrdiff_t>((row + 1) * rowFloats),
              dst);
  }
  staging->stopMapRegion();
  staging->upload(box, this->texture, 0u, nullptr, nullptr);
  this->manager->removeStagingTexture(staging);
  if (!this->texture->isDataReady())
    this->texture->notifyDataIsReady();
  this->dirty = false;
}

//////////////////////////////////////////////////
void LensDropsPass::CreateRenderPass()
{
  if (!this->ogreCompositorNodeDefName.empty())
    return;

  // One material, texture and node definition per pass, so two cameras
  // never share drops.
  static int counter = 0;
  const std::string id = std::to_string(counter++);
  const std::string group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

  auto &programs = Ogre::HighLevelGpuProgramManager::getSingleton();
  Ogre::HighLevelGpuProgramPtr vsProgram = programs.createProgram(
      "LensDropsVS_" + id, group, "glsl", Ogre::GPT_VERTEX_PROGRAM);
  vsProgram->setSource(this->vs);
  vsProgram->load();
  Ogre::HighLevelGpuProgramPtr fsProgram = programs.createProgram(
      "LensDropsFS_" + id, group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
  fsProgram->setSource(this->fs);
  fsProgram->load();

  const std::string materialName = "LensDrops_" + id;
  Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().create(materialName, group);
  Ogre::Pass *pass = mat->getTechnique(0)->getPass(0);
  Ogre::HlmsMacroblock macroblock;
  macroblock.mDepthCheck = false;
  macroblock.mDepthWrite = false;
  macroblock.mCullMode = Ogre::CULL_NONE;
  pass->setMacroblock(macroblock);
  pass->setVertexProgram(vsProgram->getName());
  pass->setFragmentProgram(fsProgram->getName());
  pass->getVertexProgramParameters()->setNamedAutoConstant("worldViewProj",
      Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);

  Ogre::HlmsSamplerblock samplerblock;
  samplerblock.setAddressingMode(Ogre::TAM_CLAMP);
  samplerblock.setFiltering(Ogre::TFO_BILINEAR);
  // Unit 0: the image so far, set by the compositor.
  Ogre::TextureUnitState *input = pass->createTextureUnitState();
  input->setSamplerblock(samplerblock);

  // Unit 1: the drop map.
  this->manager = gz::rendering::Ogre2RenderEngine::Instance()->OgreRoot()
      ->getRenderSystem()->getTextureGpuManager();
  this->texture = this->manager->createOrRetrieveTexture(
      "LensDropsMap_" + id, Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::ManualTexture, Ogre::TextureTypes::Type2D);
  this->texture->setResolution(this->width, this->height);
  this->texture->setNumMipmaps(1u);
  this->texture->setPixelFormat(Ogre::PFG_RGBA32_FLOAT);
  this->unit = pass->createTextureUnitState();
  this->unit->setTexture(this->texture);
  this->unit->setSamplerblock(samplerblock);

  Ogre::GpuProgramParametersSharedPtr fsParams =
      pass->getFragmentProgramParameters();
  fsParams->setNamedConstant("RT", 0);
  fsParams->setNamedConstant("dropMap", 1);
  mat->load();
  this->material = mat.get();

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
  Ogre::CompositorTargetDef *targetDef = nodeDef->addTargetPass("rt_output");
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
}  // namespace

extern "C"
{
//////////////////////////////////////////////////
int lens_drops_ogre2_abi_version(void)
{
  return 2;
}

//////////////////////////////////////////////////
lens_drops_pass_t lens_drops_ogre2_create(const char *_vs, const char *_fs,
    int _mapWidth, int _mapHeight)
{
  if (!_vs || !_fs || !*_vs || !*_fs || _mapWidth < 1 || _mapHeight < 1 ||
      _mapWidth > kMaxMapSize || _mapHeight > kMaxMapSize)
  {
    return nullptr;
  }
  try
  {
    return new PassPtr(std::make_shared<LensDropsPass>(_vs, _fs,
        static_cast<unsigned int>(_mapWidth),
        static_cast<unsigned int>(_mapHeight)));
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
  try
  {
    auto *camera = static_cast<gz::rendering::Camera *>(_camera);
    camera->AddRenderPass(*static_cast<PassPtr *>(_pass));
    return 1;
  }
  catch (const std::exception &e)
  {
    gzerr << "[LensDrops] the camera did not take the pass: " << e.what()
          << "\n";
  }
  catch (...)
  {
    gzerr << "[LensDrops] the camera did not take the pass\n";
  }
  return 0;
}

//////////////////////////////////////////////////
int lens_drops_ogre2_set_map(lens_drops_pass_t _pass, const float *_rgba,
    int _width, int _height, float _focus)
{
  if (!_pass || !_rgba)
    return 0;
  LensDropsPass &pass = **static_cast<PassPtr *>(_pass);
  if (_width < 1 || _height < 1 ||
      static_cast<unsigned int>(_width) != pass.width ||
      static_cast<unsigned int>(_height) != pass.height)
  {
    return 0;
  }
  std::copy(_rgba, _rgba + pass.map.size(), pass.map.begin());
  pass.dirty = true;
  pass.focus = std::isfinite(_focus) ? std::clamp(_focus, 0.0f, 1.0f) : 0.5f;
  return 1;
}

//////////////////////////////////////////////////
void lens_drops_ogre2_destroy(lens_drops_pass_t _pass, void *_camera)
{
  if (!_pass)
    return;
  auto *pass = static_cast<PassPtr *>(_pass);
  try
  {
    if (_camera)
    {
      static_cast<gz::rendering::Camera *>(_camera)->RemoveRenderPass(*pass);
    }
  }
  catch (...)
  {
  }
  delete pass;
}
}  // extern "C"
