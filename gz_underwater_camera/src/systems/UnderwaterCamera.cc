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
#include "UnderwaterCamera.hh"

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <cstring>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/components/RgbdCamera.hh>

#include "gz/sim/components/UnderwaterRendering.hh"
#include "gz/sim/underwater_camera/UnderwaterImage.hh"
#include "gz/sim/water_optics/WaterOpticsSampler.hh"

using namespace gz;
using namespace sim;
using namespace maritime;
namespace uc = underwater_camera;

class gz::sim::maritime::UnderwaterCameraPrivate
{
  /// \brief A colour frame arrived.
  public: void OnImage(const msgs::Image &_msg);

  /// \brief A depth frame arrived.
  public: void OnDepth(const msgs::Image &_msg);

  /// \brief The camera's intrinsics arrived.
  public: void OnInfo(const msgs::CameraInfo &_msg);

  /// \brief If the colour and depth frames match, make and publish the
  /// underwater image. Call with the mutex held.
  public: void TryProcess();

  /// \brief The sensor entity.
  public: Entity sensor{kNullEntity};

  /// \brief The camera's response.
  public: uc::Response response;

  /// \brief Noise.
  public: std::mt19937 rng{1};

  /// \brief Reads the world's water.
  public: water_optics::WaterOpticsSampler sampler;

  /// \brief Guards everything below across transport and ECM threads.
  public: std::mutex mutex;

  /// \brief The water at the camera, from the last step.
  public: uc::Water water;

  /// \brief Whether a step has filled the water in yet.
  public: bool haveWater{false};

  /// \brief The latest frames, waiting for their partner.
  public: std::optional<msgs::Image> image;
  public: std::optional<msgs::Image> depth;

  /// \brief The camera's ray table, rebuilt when its intrinsics change.
  public: std::optional<uc::RayTable> rays;

  /// \brief The last CameraInfo.
  public: std::optional<msgs::CameraInfo> info;

  /// \brief Transport.
  public: transport::Node node;
  public: transport::Node::Publisher imagePub;
  public: transport::Node::Publisher infoPub;

  /// \brief Frames made.
  public: std::uint64_t frames{0};

  /// \brief Whether the pass-through was announced.
  public: bool warnedDrawn{false};
};

namespace
{
bool SameStamp(const msgs::Image &_a, const msgs::Image &_b)
{
  return _a.header().stamp().sec() == _b.header().stamp().sec() &&
         _a.header().stamp().nsec() == _b.header().stamp().nsec();
}
}  // namespace

//////////////////////////////////////////////////
void UnderwaterCameraPrivate::OnImage(const msgs::Image &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  this->image = _msg;
  this->TryProcess();
}

//////////////////////////////////////////////////
void UnderwaterCameraPrivate::OnDepth(const msgs::Image &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  this->depth = _msg;
  this->TryProcess();
}

//////////////////////////////////////////////////
void UnderwaterCameraPrivate::OnInfo(const msgs::CameraInfo &_msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex);
  uc::Intrinsics k;
  k.width = _msg.width();
  k.height = _msg.height();
  if (_msg.intrinsics().k_size() >= 9)
  {
    k.fx = _msg.intrinsics().k(0);
    k.cx = _msg.intrinsics().k(2);
    k.fy = _msg.intrinsics().k(4);
    k.cy = _msg.intrinsics().k(5);
  }
  if (!this->rays || this->rays->Camera().width != k.width ||
      this->rays->Camera().height != k.height ||
      this->rays->Camera().fx != k.fx || this->rays->Camera().fy != k.fy ||
      this->rays->Camera().cx != k.cx || this->rays->Camera().cy != k.cy)
  {
    this->rays.emplace(k);
  }
  this->info = _msg;
}

//////////////////////////////////////////////////
void UnderwaterCameraPrivate::TryProcess()
{
  if (!this->image || !this->depth || !this->rays || !this->haveWater ||
      !SameStamp(*this->image, *this->depth))
  {
    return;
  }
  const auto &img = *this->image;
  const auto &dep = *this->depth;
  const auto &k = this->rays->Camera();
  if (img.width() != k.width || img.height() != k.height ||
      dep.width() != k.width || dep.height() != k.height ||
      img.pixel_format_type() != msgs::PixelFormatType::RGB_INT8 ||
      dep.pixel_format_type() != msgs::PixelFormatType::R_FLOAT32)
  {
    gzwarn << "UnderwaterCamera: frames of " << img.width() << "x"
           << img.height() << " do not match the camera's " << k.width
           << "x" << k.height << " RGB and float depth; skipped\n";
    this->image.reset();
    this->depth.reset();
    return;
  }

  const std::size_t n = static_cast<std::size_t>(k.width) * k.height;
  std::vector<std::uint8_t> rgb(n * 3);
  for (std::uint32_t v = 0; v < k.height; ++v)
  {
    std::memcpy(rgb.data() + v * k.width * 3,
                img.data().data() + v * img.step(), k.width * 3);
  }
  std::vector<float> z(n);
  std::memcpy(z.data(), dep.data().data(),
              std::min(dep.data().size(), n * sizeof(float)));

  std::vector<std::uint8_t> out;
  uc::MakeUnderwater(rgb, z, *this->rays, this->water, this->response,
                     this->rng, out);

  msgs::Image msg;
  *msg.mutable_header() = img.header();
  msg.set_width(k.width);
  msg.set_height(k.height);
  msg.set_step(k.width * 3);
  msg.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
  msg.set_data(out.data(), out.size());
  this->imagePub.Publish(msg);
  if (this->info)
  {
    msgs::CameraInfo info = *this->info;
    *info.mutable_header() = img.header();
    this->infoPub.Publish(info);
  }
  ++this->frames;
  this->image.reset();
  this->depth.reset();
}

//////////////////////////////////////////////////
UnderwaterCamera::UnderwaterCamera()
  : dataPtr(std::make_unique<UnderwaterCameraPrivate>())
{
}

//////////////////////////////////////////////////
UnderwaterCamera::~UnderwaterCamera() = default;

//////////////////////////////////////////////////
void UnderwaterCamera::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  d.sensor = _entity;
  const auto *camera = _ecm.Component<components::RgbdCamera>(_entity);
  if (!camera)
  {
    gzerr << "UnderwaterCamera: goes inside an rgbd_camera sensor; "
          << "disabled\n";
    return;
  }
  const std::string topic = camera->Data().Topic();
  if (topic.empty())
  {
    gzerr << "UnderwaterCamera: the rgbd_camera needs a <topic>; "
          << "disabled\n";
    return;
  }

  // The camera's response.
  const std::string exposure =
      _sdf->Get<std::string>("exposure", "1").first;
  if (exposure == "auto")
  {
    d.response.exposure = uc::Response::Exposure::kAuto;
    d.response.gain = _sdf->Get<double>("max_gain", 16.0).first;
  }
  else
  {
    try
    {
      d.response.gain = std::stod(exposure);
    }
    catch (...)
    {
      gzerr << "UnderwaterCamera: <exposure> is a gain or 'auto', not '"
            << exposure << "'; using 1\n";
      d.response.gain = 1.0;
    }
  }
  const std::string wb =
      _sdf->Get<std::string>("white_balance", "none").first;
  if (wb == "grey_world")
    d.response.whiteBalance = uc::Response::WhiteBalance::kGreyWorld;
  else if (wb != "none")
    gzerr << "UnderwaterCamera: unknown <white_balance> '" << wb
          << "'; using none\n";
  d.response.shotNoise = std::max(0.0, _sdf->Get<double>("shot_noise",
                                                         0.0).first);
  d.response.readNoise = std::max(0.0, _sdf->Get<double>("read_noise",
                                                         0.0).first);
  d.rng.seed(_sdf->Get<unsigned int>("seed", 1u).first);

  std::string base = "/" + topic;
  while (base.size() > 1 && base[1] == '/')
    base.erase(0, 1);
  const std::string out = _sdf->Get<std::string>("output_topic",
                                                 base + "/underwater").first;
  d.imagePub = d.node.Advertise<msgs::Image>(out + "/image");
  d.infoPub = d.node.Advertise<msgs::CameraInfo>(out + "/camera_info");
  d.node.Subscribe(base + "/image", &UnderwaterCameraPrivate::OnImage, &d);
  d.node.Subscribe(base + "/depth_image", &UnderwaterCameraPrivate::OnDepth,
                   &d);
  d.node.Subscribe(base + "/camera_info", &UnderwaterCameraPrivate::OnInfo,
                   &d);
  gzmsg << "UnderwaterCamera: " << base << " -> " << out << "/image\n";
}

//////////////////////////////////////////////////
void UnderwaterCamera::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (d.sensor == kNullEntity)
    return;
  d.sampler.Sync(_ecm);
  // The renderer already draws the water into every camera's image: pass
  // it through rather than apply the water twice.
  const bool drawn = nullptr != _ecm.Component<components::UnderwaterRendering>(
      worldEntity(_ecm));
  const auto pose = worldPose(d.sensor, _ecm);
  const double depth = d.sampler.Depth(pose.Pos(), _info.simTime);
  const auto &optics = d.sampler.Data();

  const std::lock_guard<std::mutex> lock(d.mutex);
  d.water.attenuation = optics.attenuation;
  d.water.backscatter = optics.backscatter;
  d.water.diffuseAttenuation = optics.diffuseAttenuation;
  d.water.veil = d.sampler.Veil(depth);
  d.water.cameraDepth = drawn ? 0.0 : depth;
  if (drawn && !d.warnedDrawn)
  {
    gzmsg << "UnderwaterCamera: the renderer draws the water "
          << "(gz_underwater_rendering); passing the image through\n";
    d.warnedDrawn = true;
  }
  d.water.cameraRotation = pose.Rot();
  d.haveWater = true;
}

GZ_ADD_PLUGIN(UnderwaterCamera,
              System,
              UnderwaterCamera::ISystemConfigure,
              UnderwaterCamera::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(UnderwaterCamera, "gz::sim::maritime::UnderwaterCamera")
