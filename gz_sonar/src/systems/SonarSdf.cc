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
#include "SonarSdf.hh"

#include <cstddef>
#include <string>

namespace gz::sim::maritime
{
namespace sonar = gz::maritime::sonar;

namespace
{
/// \brief A child's value, if it is there.
template<typename T>
std::optional<T> Child(const sdf::ElementPtr &_e, const std::string &_name)
{
  if (!_e || !_e->HasElement(_name))
    return std::nullopt;
  return _e->FindElement(_name)->Get<T>();
}

/// \brief A count that must be positive.
bool Count(const sdf::ElementPtr &_e, const std::string &_name,
  std::size_t &_out, std::string &_error)
{
  if (const auto v = Child<int>(_e, _name))
  {
    if (*v <= 0)
    {
      _error = "<" + _name + "> must be positive";
      return false;
    }
    _out = static_cast<std::size_t>(*v);
  }
  return true;
}
}  // namespace

bool IsSonarType(const std::string &_type)
{
  return _type == kImagingSonar || _type == kSidescanSonar ||
    _type == kProfilingSonar;
}

std::optional<sonar::SonarConfig> LoadSonarConfig(const std::string &_type,
  const sdf::ElementPtr &_block, bool &_labels, std::string &_error)
{
  sonar::SonarConfig c;
  if (_type == kImagingSonar)
    c = sonar::EchoImaging();
  else if (_type == kSidescanSonar)
    c = sonar::OmniscanSidescan();
  else if (_type == kProfilingSonar)
    c = sonar::SurveyorProfiling();
  else
  {
    _error = "unknown sonar type [" + _type + "]";
    return std::nullopt;
  }
  _labels = false;
  if (!_block)
    return c;

  if (auto v = Child<double>(_block, "frequency"))
    c.frequency = *v;
  if (auto v = Child<double>(_block, "horizontal_fov"))
    c.horizontalFov = *v;
  if (auto v = Child<double>(_block, "vertical_fov"))
    c.verticalFov = *v;
  if (!Count(_block, "beams", c.beams, _error) ||
      !Count(_block, "samples", c.samples, _error) ||
      !Count(_block, "rays_per_beam", c.raysPerBeam, _error) ||
      !Count(_block, "elevation_rays", c.elevationRays, _error) ||
      !Count(_block, "max_soundings", c.maxSoundings, _error))
  {
    return std::nullopt;
  }
  if (auto v = Child<double>(_block, "beam_width"))
    c.beamWidth = *v;
  if (auto v = Child<double>(_block, "sidelobe_level"))
    c.sidelobeLevelDb = *v;
  if (_block->HasElement("range"))
  {
    const auto range = _block->FindElement("range");
    if (auto v = Child<double>(range, "min"))
      c.rangeMin = *v;
    if (auto v = Child<double>(range, "max"))
      c.rangeMax = *v;
  }
  // Flat spellings too, for configs that cannot nest (a vehicle's parts
  // list passes its sonar settings as one level of key: value).
  if (auto v = Child<double>(_block, "range_min"))
    c.rangeMin = *v;
  if (auto v = Child<double>(_block, "range_max"))
    c.rangeMax = *v;
  if (auto v = Child<double>(_block, "pulse_length"))
    c.pulseLength = *v;
  if (auto v = Child<double>(_block, "sound_speed"))
    c.soundSpeed = *v;
  if (auto v = Child<std::string>(_block, "gain"))
  {
    if (*v == "auto")
      c.gainDb.reset();
    else
    {
      try
      {
        c.gainDb = std::stod(*v);
      }
      catch (...)
      {
        _error = "<gain> must be auto or a number of dB, not [" + *v + "]";
        return std::nullopt;
      }
    }
  }
  if (auto v = Child<double>(_block, "gain_time_constant"))
    c.gainTimeConstant = *v;
  if (auto v = Child<std::string>(_block, "tvg"))
  {
    if (*v == "none")
      c.tvg = sonar::Tvg::kNone;
    else if (*v == "spreading")
      c.tvg = sonar::Tvg::kSpreading;
    else if (*v == "spreading_absorption")
      c.tvg = sonar::Tvg::kSpreadingAbsorption;
    else
    {
      _error = "<tvg> must be none, spreading or spreading_absorption";
      return std::nullopt;
    }
  }
  if (auto v = Child<double>(_block, "tvg_spreading"))
    c.tvgSpreading = *v;
  if (auto v = Child<double>(_block, "noise_floor"))
    c.noiseFloorDb = *v;
  if (auto v = Child<double>(_block, "dynamic_range"))
    c.dynamicRangeDb = *v;
  if (auto v = Child<bool>(_block, "speckle"))
    c.speckle = *v;
  if (auto v = Child<bool>(_block, "labels"))
    _labels = *v;
  if (auto v = Child<std::string>(_block, "readout"))
  {
    if (*v == "beamformed")
      c.readout = sonar::ProfilingReadout::kBeamformed;
    else if (*v == "angle_of_arrival")
      c.readout = sonar::ProfilingReadout::kAngleOfArrival;
    else
    {
      _error = "<readout> must be beamformed or angle_of_arrival";
      return std::nullopt;
    }
  }
  if (auto v = Child<double>(_block, "detection_threshold"))
    c.detectionThresholdDb = *v;
  if (auto v = Child<double>(_block, "range_noise"))
    c.rangeNoise = *v;
  if (auto v = Child<double>(_block, "angle_noise"))
    c.angleNoise = *v;

  if (c.rangeMax <= c.rangeMin || c.rangeMin < 0.0)
  {
    _error = "<range> needs 0 <= min < max";
    return std::nullopt;
  }
  if (c.horizontalFov <= 0.0 || c.verticalFov <= 0.0 || c.beamWidth <= 0.0)
  {
    _error = "fields of view and beam width must be positive";
    return std::nullopt;
  }
  return c;
}

AcousticMarks ReadAcousticMarks(const sdf::ElementPtr &_element)
{
  AcousticMarks m;
  m.lambertDb = Child<double>(_element, "gz:acoustic_lambert");
  m.specularDb = Child<double>(_element, "gz:acoustic_specular");
  m.specularWidth = Child<double>(_element, "gz:acoustic_specular_width");
  m.transparent = _element && _element->HasElement("gz:acoustic_transparent");
  return m;
}

AcousticMarks ReadAcousticValues(const sdf::ElementPtr &_element)
{
  AcousticMarks m;
  m.lambertDb = Child<double>(_element, "lambert");
  m.specularDb = Child<double>(_element, "specular");
  m.specularWidth = Child<double>(_element, "specular_width");
  return m;
}

void ApplyMarks(sonar::AcousticProperties &_properties,
  const AcousticMarks &_marks)
{
  if (_marks.lambertDb)
    _properties.lambertDb = *_marks.lambertDb;
  if (_marks.specularDb)
    _properties.specularDb = *_marks.specularDb;
  if (_marks.specularWidth)
    _properties.specularWidth = *_marks.specularWidth;
}
}  // namespace gz::sim::maritime
