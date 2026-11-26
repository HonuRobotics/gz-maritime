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
/// \brief A child's text, trimmed, if the child is there.
std::optional<std::string> Text(const sdf::ElementPtr &_e,
  const std::string &_name)
{
  if (!_e || !_e->HasElement(_name))
    return std::nullopt;
  std::string t = _e->FindElement(_name)->Get<std::string>();
  const auto first = t.find_first_not_of(" \t\n\r");
  const auto last = t.find_last_not_of(" \t\n\r");
  return first == std::string::npos ? std::string() :
    t.substr(first, last - first + 1);
}

/// \brief Parse a whole string as a finite number.
std::optional<double> ParseNumber(const std::string &_text)
{
  try
  {
    std::size_t used = 0;
    const double v = std::stod(_text, &used);
    if (used == _text.size() && std::isfinite(v))
      return v;
  }
  catch (...)
  {
  }
  return std::nullopt;
}

/// \brief Parse a whole string as true or false (true, false, 1 or 0).
std::optional<bool> ParseFlag(const std::string &_text)
{
  if (_text == "true" || _text == "1")
    return true;
  if (_text == "false" || _text == "0")
    return false;
  return std::nullopt;
}

/// \brief Reads <gz:sonar>'s children strictly: a value that does not parse
/// whole, or is not finite, refuses the sensor with a message naming it,
/// rather than reading as 0 or a prefix of itself.
class Reader
{
  public: Reader(const sdf::ElementPtr &_block, std::string &_error)
    : block(_block), error(_error) {}

  /// \brief A finite number, if the child is there.
  public: bool Number(const std::string &_name, double &_out)
  {
    return this->Number(this->block, _name, _out);
  }

  public: bool Number(const sdf::ElementPtr &_e, const std::string &_name,
    double &_out)
  {
    const auto t = Text(_e, _name);
    if (!t)
      return true;
    const auto v = ParseNumber(*t);
    if (!v)
      return this->Fail("<" + _name + "> must be a finite number, not [" + *t + "]");
    _out = *v;
    return true;
  }

  public: bool Number(const std::string &_name, std::optional<double> &_out)
  {
    double v = 0.0;
    const bool had = Text(this->block, _name).has_value();
    if (!this->Number(_name, v))
      return false;
    if (had)
      _out = v;
    return true;
  }

  /// \brief A whole number from 1 to _max, if the child is there.
  public: bool Count(const std::string &_name, std::size_t _max,
    std::size_t &_out)
  {
    const auto t = Text(this->block, _name);
    if (!t)
      return true;
    const bool digits = !t->empty() &&
      t->find_first_not_of("0123456789") == std::string::npos;
    const auto v = digits && t->size() < 10 ? std::stoul(*t) : 0ul;
    if (v < 1 || v > _max)
    {
      return this->Fail("<" + _name + "> must be a whole number from 1 to " +
        std::to_string(_max) + ", not [" + *t + "]");
    }
    _out = v;
    return true;
  }

  /// \brief true or false, if the child is there.
  public: bool Flag(const std::string &_name, bool &_out)
  {
    const auto t = Text(this->block, _name);
    if (!t)
      return true;
    const auto v = ParseFlag(*t);
    if (!v)
      return this->Fail("<" + _name + "> must be true or false, not [" + *t + "]");
    _out = *v;
    return true;
  }

  private: bool Fail(const std::string &_message)
  {
    this->error = _message;
    return false;
  }

  private: sdf::ElementPtr block;
  private: std::string &error;
};
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

  Reader r(_block, _error);
  const sdf::ElementPtr range =
    _block->HasElement("range") ? _block->FindElement("range") : nullptr;
  // Upper bounds keep the cameras and buffers a sonar asks for sane.
  const bool ok =
    r.Number("frequency", c.frequency) &&
    r.Number("horizontal_fov", c.horizontalFov) &&
    r.Number("vertical_fov", c.verticalFov) &&
    r.Count("beams", 2048, c.beams) &&
    r.Count("samples", 20000, c.samples) &&
    r.Count("rays_per_beam", 32, c.raysPerBeam) &&
    r.Count("elevation_rays", 2048, c.elevationRays) &&
    r.Count("max_soundings", 1024, c.maxSoundings) &&
    r.Number("elevation_span", c.elevationSpan) &&
    r.Number("beam_width", c.beamWidth) &&
    r.Number("sidelobe_level", c.sidelobeLevelDb) &&
    r.Number(range, "min", c.rangeMin) &&
    r.Number(range, "max", c.rangeMax) &&
    // Flat spellings too, for configs that cannot nest (a vehicle's parts
    // list passes its sonar settings as one level of key: value).
    r.Number("range_min", c.rangeMin) &&
    r.Number("range_max", c.rangeMax) &&
    r.Number("pulse_length", c.pulseLength) &&
    r.Number("sound_speed", c.soundSpeed) &&
    r.Number("gain_time_constant", c.gainTimeConstant) &&
    r.Number("tvg_spreading", c.tvgSpreading) &&
    r.Number("noise_floor", c.noiseFloorDb) &&
    r.Number("dynamic_range", c.dynamicRangeDb) &&
    r.Flag("speckle", c.speckle) &&
    r.Flag("labels", _labels) &&
    r.Number("detection_threshold", c.detectionThresholdDb) &&
    r.Number("range_noise", c.rangeNoise) &&
    r.Number("angle_noise", c.angleNoise);
  if (!ok)
    return std::nullopt;

  if (auto v = Text(_block, "gain"))
  {
    if (*v == "auto")
      c.gainDb.reset();
    else if (auto g = ParseNumber(*v))
      c.gainDb = *g;
    else
    {
      _error = "<gain> must be auto or a number of dB, not [" + *v + "]";
      return std::nullopt;
    }
  }
  if (auto v = Text(_block, "tvg"))
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
  if (auto v = Text(_block, "detection"))
  {
    if (*v == "beamformed")
      c.detection = sonar::ProfilingDetection::kBeamformed;
    else if (*v == "angle_of_arrival")
      c.detection = sonar::ProfilingDetection::kAngleOfArrival;
    else
    {
      _error = "<detection> must be beamformed or angle_of_arrival";
      return std::nullopt;
    }
  }

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
    "angle_noise", "beam_width", "beams", "detection", "detection_threshold",
    "dynamic_range", "elevation_rays", "elevation_span", "frequency", "gain",
    "gain_time_constant", "horizontal_fov", "labels", "max_soundings",
    "noise_floor", "pulse_length", "range", "range_max", "range_min",
    "range_noise", "rays_per_beam", "samples", "sidelobe_level",
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

namespace
{
/// \brief One acoustic value, strictly: a value that does not parse whole
/// as a finite number is reported and left unset.
std::optional<double> MarkValue(const sdf::ElementPtr &_e,
  const std::string &_name)
{
  const auto t = Text(_e, _name);
  if (!t)
    return std::nullopt;
  const auto v = ParseNumber(*t);
  if (!v)
  {
    gzerr << "sonar: <" << _name << "> must be a finite number, not [" << *t
          << "]; ignored\n";
  }
  return v;
}
}  // namespace

/// \brief An attribute's value, if the element has it.
std::optional<std::string> Attribute(const sdf::ElementPtr &_e,
  const std::string &_name)
{
  if (!_e || !_e->HasAttribute(_name))
    return std::nullopt;
  return _e->GetAttribute(_name)->GetAsString();
}

/// \brief One acoustic mark, an attribute, strictly: a value that does not
/// parse whole as a finite number is reported and left unset.
std::optional<double> MarkAttribute(const sdf::ElementPtr &_e,
  const std::string &_name)
{
  const auto t = Attribute(_e, _name);
  if (!t)
    return std::nullopt;
  const auto v = ParseNumber(*t);
  if (!v)
  {
    gzerr << "sonar: " << _name << " must be a finite number, not [" << *t
          << "]; ignored\n";
  }
  return v;
}

AcousticMarks ReadAcousticMarks(const sdf::ElementPtr &_element)
{
  AcousticMarks m;
  // Attributes of the <visual> or <model>, as gz:buoyancy is of a collision;
  // namespaced, so SDFormat keeps them without knowing them.
  m.lambertDb = MarkAttribute(_element, "gz:acoustic_lambert");
  m.specularDb = MarkAttribute(_element, "gz:acoustic_specular");
  m.specularWidth = MarkAttribute(_element, "gz:acoustic_specular_width");
  if (auto t = Attribute(_element, "gz:acoustic_transparent"))
  {
    if (auto f = ParseFlag(*t))
      m.transparent = *f;
    else
    {
      gzerr << "sonar: gz:acoustic_transparent must be true or false, not ["
            << *t << "]; taken as false\n";
    }
  }
  // The older spelling, child elements, is no longer read: say so.
  for (const char *name : {"gz:acoustic_lambert", "gz:acoustic_specular",
                           "gz:acoustic_specular_width",
                           "gz:acoustic_transparent"})
  {
    if (_element && _element->HasElement(name))
    {
      gzwarn << "sonar: <" << name << "> as a child element is not read; "
             << "write it as an attribute of the <" << _element->GetName()
             << ">, " << name << "=\"...\"\n";
    }
  }
  return m;
}

AcousticMarks ReadAcousticValues(const sdf::ElementPtr &_element)
{
  AcousticMarks m;
  m.lambertDb = MarkValue(_element, "lambert");
  m.specularDb = MarkValue(_element, "specular");
  m.specularWidth = MarkValue(_element, "specular_width");
  return m;
}

bool ReadWater(const sdf::ElementPtr &_plugin, sonar::Water &_water,
  std::uint64_t &_seed, std::string &_error)
{
  Reader r(_plugin, _error);
  if (!r.Number("temperature", _water.temperature) ||
      !r.Number("salinity", _water.salinity) ||
      !r.Number("sound_speed", _water.soundSpeed))
  {
    return false;
  }
  // Ranges in which the absorption equations hold and water is water.
  if (_water.temperature < -2.0 || _water.temperature > 40.0)
  {
    _error = "<temperature> must be from -2 to 40 degrees C";
    return false;
  }
  if (_water.salinity < 0.0 || _water.salinity > 45.0)
  {
    _error = "<salinity> must be from 0 to 45 PSU";
    return false;
  }
  if (_water.soundSpeed < 1300.0 || _water.soundSpeed > 1700.0)
  {
    _error = "<sound_speed> must be from 1300 to 1700 m/s";
    return false;
  }
  if (auto t = Text(_plugin, "seed"))
  {
    const bool digits = !t->empty() &&
      t->find_first_not_of("0123456789") == std::string::npos && t->size() < 20;
    if (!digits)
    {
      _error = "<seed> must be a whole number, not [" + *t + "]";
      return false;
    }
    _seed = std::stoull(*t);
  }
  return true;
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
