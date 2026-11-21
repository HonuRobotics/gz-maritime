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

#include <cmath>
#include <cstddef>
#include <set>
#include <string>

#include <gz/common/Console.hh>

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
  if (auto v = Child<double>(_block, "elevation_span"))
  {
    if (*v <= 0.0)
    {
      _error = "<elevation_span> must be positive";
      return std::nullopt;
    }
    c.elevationSpan = *v;
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
  // The rays reach elevationSpan half-widths of the vertical field of view,
  // and the cameras that sample them cannot look past 90 degrees.
  if (0.5 * c.verticalFov * c.elevationSpan >= 85.0 * M_PI / 180.0)
  {
    _error = "<vertical_fov> times <elevation_span> must stay under 170 degrees";
    return std::nullopt;
  }
  if (c.elevationSpan <= 0.0)
  {
    _error = "<elevation_span> must be positive";
    return std::nullopt;
  }
  struct Check { double value; bool ok; const char *message; };
  const Check checks[] = {
    {c.frequency, c.frequency > 0.0, "<frequency> must be positive"},
    {c.pulseLength, c.pulseLength > 0.0, "<pulse_length> must be positive"},
    {c.soundSpeed.value_or(1.0), c.soundSpeed.value_or(1.0) > 0.0,
     "<sound_speed> must be positive"},
    {c.dynamicRangeDb, c.dynamicRangeDb > 0.0, "<dynamic_range> must be positive"},
    {c.gainTimeConstant, c.gainTimeConstant >= 0.0,
     "<gain_time_constant> must not be negative"},
    {c.tvgSpreading, c.tvgSpreading >= 0.0, "<tvg_spreading> must not be negative"},
    {c.sidelobeLevelDb, c.sidelobeLevelDb < 0.0, "<sidelobe_level> must be negative"},
    {c.rangeNoise.value_or(0.0), c.rangeNoise.value_or(0.0) >= 0.0,
     "<range_noise> must not be negative"},
    {c.angleNoise, c.angleNoise >= 0.0, "<angle_noise> must not be negative"},
  };
  for (const auto &k : checks)
  {
    if (!k.ok || !std::isfinite(k.value))
    {
      _error = k.message;
      return std::nullopt;
    }
  }
  // Unknown elements are most likely misspellings; say so rather than
  // ignore them.
  static const std::set<std::string> known = {
    "angle_noise", "beam_width", "beams", "detection_threshold",
    "dynamic_range", "elevation_rays", "elevation_span", "frequency", "gain",
    "gain_time_constant", "horizontal_fov", "labels", "max_soundings",
    "noise_floor", "pulse_length", "range", "range_max", "range_min",
    "range_noise", "rays_per_beam", "readout", "samples", "sidelobe_level",
    "sound_speed", "speckle", "tvg", "tvg_spreading", "vertical_fov",
    "water_column"};
  for (auto e = _block->GetFirstElement(); e; e = e->GetNextElement())
  {
    if (!known.count(e->GetName()))
    {
      gzwarn << "sonar: unknown element <" << e->GetName()
             << "> in <gz:sonar>, ignored\n";
    }
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
  {
    // A zero width would divide by zero in the specular lobe.
    if (*_marks.specularWidth > 0.0)
      _properties.specularWidth = *_marks.specularWidth;
    else
      gzwarn << "sonar: a specular width must be positive; ignoring "
             << *_marks.specularWidth << "\n";
  }
}
}  // namespace gz::sim::maritime
