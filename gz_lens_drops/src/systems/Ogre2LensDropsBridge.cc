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
#include <array>
#include <exception>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
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
#include <OgreResourceGroupManager.h>
#include <OgreRoot.h>
#include <OgreTechnique.h>
#include <OgreTextureUnitState.h>

namespace
{
/// \brief The most drops the fragment shader loops over. The shader's
/// `drops` array has this size.
constexpr int kMaxDrops = 64;

/// \brief Floats per drop: x, y, radius, unused.
constexpr int kDropFloats = 4;

/// \brief A full screen pass that draws drops of water over the image the
/// passes before it left.
class LensDropsPass : public gz::rendering::Ogre2RenderPass
{
  /// \brief Constructor.
  /// \param[in] _vs GLSL source of the vertex shader.
  /// \param[in] _fs GLSL source of the fragment shader.
  public: LensDropsPass(const std::string &_vs, const std::string &_fs);

  // Documentation inherited
  public: void PreRender(const gz::rendering::CameraPtr &_camera) override;

  // Documentation inherited
  public: void CreateRenderPass() override;

  /// \brief The drops, four floats each.
  public: std::array<float, kMaxDrops * kDropFloats> drops{};

  /// \brief How many of them are set.
  public: int count{0};

  /// \brief 0 for blurred drops on the lens to 1 for sharp ones.
  public: float focus{0.5f};

  /// \brief GLSL source of the vertex shader.
  private: std::string vs;

  /// \brief GLSL source of the fragment shader.
  private: std::string fs;

  /// \brief This pass's material, owned by Ogre. Null until the pass is
  /// created.
  private: Ogre::Material *material{nullptr};
};

/// \brief What a handle points at: the pointer the camera shares.
using PassPtr = std::shared_ptr<LensDropsPass>;

//////////////////////////////////////////////////
LensDropsPass::LensDropsPass(const std::string &_vs, const std::string &_fs)
  : vs(_vs), fs(_fs)
{
}

//////////////////////////////////////////////////
void LensDropsPass::PreRender(const gz::rendering::CameraPtr &_camera)
{
  if (!this->material || !this->enabled || !_camera)
    return;

  Ogre::GpuProgramParametersSharedPtr params =
      this->material->getTechnique(0)->getPass(0)
      ->getFragmentProgramParameters();
  params->setNamedConstant("drops", this->drops.data(), kMaxDrops,
      kDropFloats);
  params->setNamedConstant("dropCount", this->count);
  params->setNamedConstant("focus", static_cast<Ogre::Real>(this->focus));
  const double height = std::max(1u, _camera->ImageHeight());
  params->setNamedConstant("aspect",
      static_cast<Ogre::Real>(_camera->ImageWidth() / height));
}

//////////////////////////////////////////////////
void LensDropsPass::CreateRenderPass()
{
  if (!this->ogreCompositorNodeDefName.empty())
    return;

  // One material and one node definition per pass, so two cameras never
  // share drops.
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
  pass->getFragmentProgramParameters()->setNamedConstant("RT", 0);
  Ogre::TextureUnitState *unit = pass->createTextureUnitState();
  Ogre::HlmsSamplerblock samplerblock;
  samplerblock.setAddressingMode(Ogre::TAM_CLAMP);
  samplerblock.setFiltering(Ogre::TFO_BILINEAR);
  unit->setSamplerblock(samplerblock);
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
int lens_drops_ogre2_max_drops(void)
{
  return kMaxDrops;
}

//////////////////////////////////////////////////
lens_drops_pass_t lens_drops_ogre2_create(const char *_vs, const char *_fs)
{
  if (!_vs || !_fs || !*_vs || !*_fs)
    return nullptr;
  try
  {
    return new PassPtr(std::make_shared<LensDropsPass>(_vs, _fs));
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
void lens_drops_ogre2_set(lens_drops_pass_t _pass, const float *_drops,
    int _count, float _focus)
{
  if (!_pass)
    return;
  LensDropsPass &pass = **static_cast<PassPtr *>(_pass);
  const int count = _drops ? std::clamp(_count, 0, kMaxDrops) : 0;
  if (count > 0)
    std::copy(_drops, _drops + count * kDropFloats, pass.drops.begin());
  pass.count = count;
  pass.focus = std::clamp(_focus, 0.0f, 1.0f);
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
