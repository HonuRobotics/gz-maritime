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

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <locale>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
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

namespace
{
using Duration = std::chrono::steady_clock::duration;

/// \brief A message's stamp as a duration.
Duration Stamp(const msgs::Header &_h)
{
  return std::chrono::seconds(_h.stamp().sec()) +
         std::chrono::nanoseconds(_h.stamp().nsec());
}

/// \brief How long the water of past steps is kept, for the frames still
/// on their way.
constexpr auto kWaterHistory = std::chrono::seconds(2);

/// \brief What the camera's callbacks leave for the worker, shared with
/// them: gz-transport may still be inside a callback after the plugin
/// unsubscribed, so what it touches lives as long as the callback holds it.
struct Mailbox
{
  /// \brief Guards everything here.
  std::mutex mutex;

  /// \brief Wakes the worker.
  std::condition_variable cv;

  /// \brief The newest colour and depth frames, by stamp.
  std::map<Duration, msgs::Image> images, depths;

  /// \brief The newest CameraInfo.
  std::optional<msgs::CameraInfo> info;

  /// \brief The water at the camera at each recent step.
  std::deque<std::pair<Duration, uc::Water>> waters;

  /// \brief Set to stop the worker.
  bool stop{false};

  /// \brief Keep a frame, dropping all but the newest few.
  void Keep(std::map<Duration, msgs::Image> &_frames, const msgs::Image &_m)
  {
    {
      const std::lock_guard<std::mutex> lock(this->mutex);
      _frames[Stamp(_m.header())] = _m;
      while (_frames.size() > 4)
        _frames.erase(_frames.begin());
    }
    this->cv.notify_one();
  }
};

/// \brief A gain from text: a positive number, read the same in any
/// locale, with nothing after it.
std::optional<double> ParseGain(const std::string &_text)
{
  std::istringstream is(_text);
  is.imbue(std::locale::classic());
  double v{0};
  std::string rest;
  if (!(is >> v) || (is >> rest) || !std::isfinite(v) || v <= 0.0)
    return std::nullopt;
  return v;
}
}  // namespace

class gz::sim::maritime::UnderwaterCameraPrivate
{
  /// \brief Stop the worker and the callbacks. Before anything else goes.
  public: ~UnderwaterCameraPrivate();

  /// \brief Listen to the camera, or stop.
  /// \param[in] _listen True to listen.
  public: void Listen(bool _listen);

  /// \brief The worker: process the newest pair of frames, one at a time.
  public: void Run();

  /// \brief Check a pair of frames against the camera and make the
  /// underwater image. Outside the lock.
  /// \param[in] _img Colour.
  /// \param[in] _dep Depth.
  /// \param[in] _water The water at the camera at their step.
  /// \param[out] _out The underwater image.
  /// \return False, with a warning once, for frames that do not fit.
  public: bool Make(const msgs::Image &_img, const msgs::Image &_dep,
                    const uc::Water &_water, msgs::Image &_out);

  /// \brief Whether Configure set the camera up.
  public: bool active{false};

  /// \brief The sensor entity.
  public: Entity sensor{kNullEntity};

  /// \brief The camera's topics, and the output's base.
  public: std::string base;

  /// \brief The camera's response.
  public: uc::Response response;

  /// \brief Noise. Worker only.
  public: std::mt19937 rng{1};

  /// \brief Reads the world's water. Simulation thread only.
  public: water_optics::WaterOpticsSampler sampler;

  /// \brief The callbacks' and the steps' mailbox.
  public: std::shared_ptr<Mailbox> mailbox{std::make_shared<Mailbox>()};

  /// \brief The camera's ray table and the info it was built from. Worker
  /// only.
  public: std::optional<uc::RayTable> rays;
  public: std::optional<msgs::CameraInfo> info;

  /// \brief Whether a frame that does not fit was warned about. Worker
  /// only.
  public: bool warnedFrame{false};

  /// \brief Transport.
  public: transport::Node node;
  public: transport::Node::Publisher imagePub;
  public: transport::Node::Publisher infoPub;

  /// \brief Whether the camera's topics are subscribed.
  public: bool listening{false};

  /// \brief The worker.
  public: std::thread worker;

  /// \brief Whether the pass-through was announced.
  public: bool warnedDrawn{false};
};

//////////////////////////////////////////////////
UnderwaterCameraPrivate::~UnderwaterCameraPrivate()
{
  this->Listen(false);
  {
    const std::lock_guard<std::mutex> lock(this->mailbox->mutex);
    this->mailbox->stop = true;
  }
  this->mailbox->cv.notify_all();
  if (this->worker.joinable())
    this->worker.join();
}

//////////////////////////////////////////////////
void UnderwaterCameraPrivate::Listen(bool _listen)
{
  if (_listen == this->listening || this->base.empty())
    return;
  this->listening = _listen;
  if (!_listen)
  {
    for (const char *t : {"/image", "/depth_image", "/camera_info"})
      this->node.Unsubscribe(this->base + t);
    return;
  }
  // The callbacks hold the mailbox, not the plugin.
  std::shared_ptr<Mailbox> box = this->mailbox;
  std::function<void(const msgs::Image &)> onImage =
      [box](const msgs::Image &_m) { box->Keep(box->images, _m); };
  std::function<void(const msgs::Image &)> onDepth =
      [box](const msgs::Image &_m) { box->Keep(box->depths, _m); };
  std::function<void(const msgs::CameraInfo &)> onInfo =
      [box](const msgs::CameraInfo &_m)
      {
        const std::lock_guard<std::mutex> lock(box->mutex);
        box->info = _m;
      };
  this->node.Subscribe(this->base + "/image", onImage);
  this->node.Subscribe(this->base + "/depth_image", onDepth);
  this->node.Subscribe(this->base + "/camera_info", onInfo);
}

//////////////////////////////////////////////////
void UnderwaterCameraPrivate::Run()
{
  auto &box = *this->mailbox;
  while (true)
  {
    msgs::Image img, dep;
    uc::Water water;
    std::optional<msgs::CameraInfo> info;
    {
      std::unique_lock<std::mutex> lock(box.mutex);
      // The newest stamp with both frames, and water for it
      std::optional<Duration> stamp;
      box.cv.wait(lock, [&]
      {
        if (box.stop)
          return true;
        stamp.reset();
        for (auto it = box.images.rbegin(); it != box.images.rend(); ++it)
        {
          if (box.depths.count(it->first))
          {
            stamp = it->first;
            break;
          }
        }
        return stamp && box.info && !box.waters.empty();
      });
      if (box.stop)
        return;
      img = std::move(box.images[*stamp]);
      dep = std::move(box.depths[*stamp]);
      box.images.erase(box.images.begin(), box.images.upper_bound(*stamp));
      box.depths.erase(box.depths.begin(), box.depths.upper_bound(*stamp));
      // The water of the frame's own step, or the closest before it
      water = box.waters.front().second;
      for (const auto &[time, w] : box.waters)
      {
        if (time > *stamp)
          break;
        water = w;
      }
      info = box.info;
    }

    // The camera's rays, again when its intrinsics change
    uc::Intrinsics k;
    k.width = info->width();
    k.height = info->height();
    if (info->intrinsics().k_size() >= 9)
    {
      k.fx = info->intrinsics().k(0);
      k.cx = info->intrinsics().k(2);
      k.fy = info->intrinsics().k(4);
      k.cy = info->intrinsics().k(5);
    }
    if (!this->rays || this->rays->Camera().width != k.width ||
        this->rays->Camera().height != k.height ||
        this->rays->Camera().fx != k.fx || this->rays->Camera().fy != k.fy ||
        this->rays->Camera().cx != k.cx || this->rays->Camera().cy != k.cy)
    {
      this->rays.emplace(k);
    }
    this->info = info;

    msgs::Image out;
    if (!this->Make(img, dep, water, out))
      continue;
    this->imagePub.Publish(out);
    msgs::CameraInfo outInfo = *info;
    *outInfo.mutable_header() = img.header();
    this->infoPub.Publish(outInfo);
  }
}

//////////////////////////////////////////////////
bool UnderwaterCameraPrivate::Make(const msgs::Image &_img,
    const msgs::Image &_dep, const uc::Water &_water, msgs::Image &_out)
{
  const auto &k = this->rays->Camera();
  const std::size_t n = static_cast<std::size_t>(k.width) * k.height;
  std::string problem;
  if (_img.pixel_format_type() != msgs::PixelFormatType::RGB_INT8)
    problem = "a colour image that is not RGB, 8 bits";
  else if (_dep.pixel_format_type() != msgs::PixelFormatType::R_FLOAT32)
    problem = "a depth image that is not float";
  else if (_img.width() != k.width || _img.height() != k.height ||
           _dep.width() != k.width || _dep.height() != k.height)
    problem = "frames of another size than the camera's";
  else if (k.height > 0 && (_img.step() < k.width * 3 ||
           _img.data().size() < static_cast<std::size_t>(_img.step()) *
           (k.height - 1) + k.width * 3))
    problem = "a colour image shorter than its size";
  else if (_dep.data().size() < n * sizeof(float))
    problem = "a depth image shorter than its size";
  if (!problem.empty())
  {
    if (!this->warnedFrame)
    {
      gzwarn << "UnderwaterCamera: " << problem << " (" << _img.width()
             << "x" << _img.height() << ", the camera is " << k.width << "x"
             << k.height << "); such frames are skipped\n";
      this->warnedFrame = true;
    }
    return false;
  }

  std::vector<std::uint8_t> rgb(n * 3);
  for (std::uint32_t v = 0; v < k.height; ++v)
  {
    std::memcpy(rgb.data() + static_cast<std::size_t>(v) * k.width * 3,
                _img.data().data() + static_cast<std::size_t>(v) * _img.step(),
                k.width * 3);
  }
  std::vector<float> z(n);
  std::memcpy(z.data(), _dep.data().data(), n * sizeof(float));

  std::vector<std::uint8_t> pixels;
  uc::MakeUnderwater(rgb, z, *this->rays, _water, this->response, this->rng,
                     pixels);

  *_out.mutable_header() = _img.header();
  _out.set_width(k.width);
  _out.set_height(k.height);
  _out.set_step(k.width * 3);
  _out.set_pixel_format_type(msgs::PixelFormatType::RGB_INT8);
  _out.set_data(pixels.data(), pixels.size());
  return true;
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
    const auto maxGain = ParseGain(
        _sdf->Get<std::string>("max_gain", "16").first);
    if (!maxGain || *maxGain < 1.0)
    {
      gzerr << "UnderwaterCamera: <max_gain> is a number of 1 or more; "
            << "using 16\n";
    }
    d.response.gain = maxGain && *maxGain >= 1.0 ? *maxGain : 16.0;
  }
  else
  {
    const auto gain = ParseGain(exposure);
    if (!gain)
    {
      gzerr << "UnderwaterCamera: <exposure> is a positive gain or 'auto', "
            << "not '" << exposure << "'; using 1\n";
    }
    d.response.gain = gain.value_or(1.0);
  }
  const std::string wb =
      _sdf->Get<std::string>("white_balance", "none").first;
  if (wb == "grey_world")
    d.response.whiteBalance = uc::Response::WhiteBalance::kGreyWorld;
  else if (wb != "none")
    gzerr << "UnderwaterCamera: unknown <white_balance> '" << wb
          << "'; using none\n";
  for (const auto &[tag, value] :
       {std::make_pair("shot_noise", &d.response.shotNoise),
        std::make_pair("read_noise", &d.response.readNoise)})
  {
    const double v = _sdf->Get<double>(tag, 0.0).first;
    if (!std::isfinite(v) || v < 0.0)
    {
      gzerr << "UnderwaterCamera: <" << tag << "> must not be negative; "
            << "using 0\n";
    }
    *value = std::isfinite(v) && v > 0.0 ? v : 0.0;
  }
  // Each camera its own noise, and the same every run
  const unsigned int seed = _sdf->Get<unsigned int>("seed", 1u).first;
  const std::string name = scopedName(_entity, _ecm);
  std::seed_seq seq{seed, static_cast<unsigned int>(
      std::hash<std::string>{}(name) & 0xffffffffu)};
  d.rng.seed(seq);

  std::string base = "/" + topic;
  while (base.size() > 1 && base[1] == '/')
    base.erase(0, 1);
  const std::string out = _sdf->Get<std::string>("output_topic",
                                                 base + "/underwater").first;
  d.imagePub = d.node.Advertise<msgs::Image>(out + "/image");
  d.infoPub = d.node.Advertise<msgs::CameraInfo>(out + "/camera_info");
  d.base = base;
  d.sensor = _entity;
  d.active = true;
  d.worker = std::thread(&UnderwaterCameraPrivate::Run, &d);
  gzmsg << "UnderwaterCamera: " << base << " -> " << out << "/image\n";
}

//////////////////////////////////////////////////
void UnderwaterCamera::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  auto &d = *this->dataPtr;
  if (!d.active)
    return;
  // Only while someone listens to the output: the camera is then not
  // rendered for us alone.
  d.Listen(d.imagePub.HasConnections() || d.infoPub.HasConnections());

  d.sampler.Sync(_ecm);
  // The renderer already draws the water into every camera's image: do not
  // apply it twice.
  const bool drawn = nullptr != _ecm.Component<components::UnderwaterRendering>(
      worldEntity(_ecm));
  const auto pose = worldPose(d.sensor, _ecm);
  uc::Water water;
  if (d.sampler.Valid())
  {
    const double depth = d.sampler.Depth(pose.Pos(), _info.simTime);
    const auto &optics = d.sampler.Data();
    water.attenuation = optics.attenuation;
    water.backscatter = optics.backscatter;
    water.diffuseAttenuation = optics.diffuseAttenuation;
    water.veil = d.sampler.Veil(depth);
    water.cameraDepth = depth;
  }
  // Without water optics, the camera is in clear water: only its response
  water.cameraRotation = pose.Rot();

  if (drawn)
  {
    // The water is in the image already: only the camera's response
    water.cameraDepth = 0.0;
    if (!d.warnedDrawn)
    {
      gzmsg << "UnderwaterCamera: the renderer draws the water "
            << "(gz_underwater_rendering); only the camera's response is "
            << "applied\n";
      d.warnedDrawn = true;
    }
  }

  auto &box = *d.mailbox;
  {
    const std::lock_guard<std::mutex> lock(box.mutex);
    // A reset goes back in time: the history after it is gone
    while (!box.waters.empty() && box.waters.back().first >= _info.simTime)
      box.waters.pop_back();
    box.waters.emplace_back(_info.simTime, water);
    while (box.waters.size() > 1 &&
           box.waters.front().first < _info.simTime - kWaterHistory)
    {
      box.waters.pop_front();
    }
  }
  box.cv.notify_one();
}

GZ_ADD_PLUGIN(UnderwaterCamera,
              System,
              UnderwaterCamera::ISystemConfigure,
              UnderwaterCamera::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(UnderwaterCamera, "gz::sim::maritime::UnderwaterCamera")
