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

// Renders sonar data from synthetic scenes with the gz_sonar engine alone,
// no Gazebo, for the contributor walkthrough "Render a ping without
// Gazebo". Writes .npy arrays and a .json of axes to an output directory;
// draw_ping.py turns them into pictures.
//
//   gz_sonar_render_ping imaging A|B|C <out_dir> [sidelobe=dB]
//                        [dynamic_range=dB] [name=suffix]
//   gz_sonar_render_ping sidescan <out_dir>
//   gz_sonar_render_ping profiling <out_dir>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>

#include "gz/maritime/sonar/Config.hh"
#include "gz/maritime/sonar/Engine.hh"
#include "gz/maritime/sonar/Rays.hh"
#include "gz/maritime/sonar/Readout.hh"
#include "gz/maritime/sonar/SyntheticScene.hh"
#include "gz/maritime/sonar/Water.hh"

namespace sonar = gz::maritime::sonar;
using gz::math::Pose3d;

namespace
{
constexpr double kDeg = M_PI / 180.0;

//////////////////////////////////////////////////
/// \brief Write a 2D array as a NumPy .npy file (format 1.0).
/// \param[in] _path The file.
/// \param[in] _data The array, row-major.
/// \param[in] _rows The number of rows.
/// \param[in] _cols The number of columns.
/// \param[in] _dtype The NumPy type, such as "<f4".
template<typename T>
void WriteNpy(const std::string &_path, const std::vector<T> &_data,
  std::size_t _rows, std::size_t _cols, const char *_dtype)
{
  std::ostringstream h;
  h << "{'descr': '" << _dtype << "', 'fortran_order': False, 'shape': ("
    << _rows << ", " << _cols << "), }";
  std::string header = h.str();
  // Pad so the data starts on a 64-byte boundary, ending in a newline.
  const std::size_t base = 10 + header.size() + 1;
  header.append((64 - base % 64) % 64, ' ');
  header.push_back('\n');
  std::ofstream f(_path, std::ios::binary);
  f.write("\x93NUMPY\x01\x00", 8);
  const auto len = static_cast<std::uint16_t>(header.size());
  f.put(static_cast<char>(len & 0xff));
  f.put(static_cast<char>(len >> 8));
  f << header;
  f.write(reinterpret_cast<const char *>(_data.data()),
    static_cast<std::streamsize>(_data.size() * sizeof(T)));
}

//////////////////////////////////////////////////
void WriteJsonArray(std::ofstream &_f, const char *_name,
  const std::vector<double> &_v, bool _last = false)
{
  _f << "  \"" << _name << "\": [";
  for (std::size_t i = 0; i < _v.size(); ++i)
    _f << (i ? ", " : "") << _v[i];
  _f << "]" << (_last ? "\n" : ",\n");
}

//////////////////////////////////////////////////
sonar::AcousticProperties Lambert(double _db)
{
  sonar::AcousticProperties p;
  p.lambertDb = _db;
  return p;
}

//////////////////////////////////////////////////
sonar::Water HarborWater()
{
  sonar::Water w;
  w.temperature = 29.0;
  w.salinity = 30.0;
  return w;
}

//////////////////////////////////////////////////
/// \brief Scenes A, B and C of spike/sonar_phase1/scenes.md, and the
/// sensor pose for each.
/// \param[in] _name The scene: A, B or C.
/// \param[out] _pose The sensor's pose.
/// \return The scene.
sonar::SyntheticScene Scene(const std::string &_name, Pose3d &_pose)
{
  sonar::SyntheticScene s;
  if (_name == "A")
  {
    s.AddSphere({5, 0, -3}, 0.25, Lambert(-10.0), 1);
    s.AddPlane({0, 0, -10}, {0, 0, 1}, Lambert(-27.0), 2);
    _pose = Pose3d(0, 0, -3, 0, 0, 0);
  }
  else if (_name == "B")
  {
    s.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 2);
    s.AddBox({4.5, -0.5, 0}, {5.5, 0.5, 0.5}, Lambert(-15.0), 1);
    _pose = Pose3d(0, 0, 2, 0, 15.0 * kDeg, 0);
  }
  else
  {
    // Rising to starboard (-y) at 10 degrees.
    s.AddPlane({0, 0, 0}, {0, std::sin(10.0 * kDeg), std::cos(10.0 * kDeg)},
      Lambert(-27.0), 2);
    _pose = Pose3d(0, 0, 2, 0, 15.0 * kDeg, 0);
  }
  return s;
}

//////////////////////////////////////////////////
/// \brief A seabed with a box and a glass float on it, for the passes.
/// \return The scene.
sonar::SyntheticScene PassScene()
{
  sonar::SyntheticScene s;
  s.AddPlane({0, 0, 0}, {0, 0, 1}, Lambert(-27.0), 2);
  s.AddBox({9.5, 3.5, 0}, {10.5, 4.5, 0.5}, Lambert(-15.0), 1);
  sonar::AcousticProperties glass = Lambert(-30.0);
  glass.specularDb = -3.0;
  s.AddSphere({16, 6, 0.25}, 0.25, glass, 3);
  return s;
}

//////////////////////////////////////////////////
int Imaging(const std::string &_scene, const std::string &_out,
  const std::vector<std::string> &_options)
{
  sonar::SonarConfig c = sonar::EchoImaging();
  c.rangeMax = 10.0;
  c.samples = 512;
  // Optional key=value settings, to see what one parameter does:
  // sidelobe=<dB> (first sidelobe of the beam pattern), dynamic_range=<dB>
  // (the levels the 8-bit frame spans) and name=<suffix> (for the files).
  std::string suffix;
  for (const auto &o : _options)
  {
    const auto eq = o.find('=');
    const std::string key = o.substr(0, eq);
    const std::string value = eq == std::string::npos ? "" : o.substr(eq + 1);
    if (key == "sidelobe")
      c.sidelobeLevelDb = std::stod(value);
    else if (key == "dynamic_range")
      c.dynamicRangeDb = std::stod(value);
    else if (key == "name")
      suffix = "_" + value;
    else
    {
      std::cerr << "unknown setting [" << o << "]\n";
      return 1;
    }
  }
  Pose3d pose;
  const auto scene = Scene(_scene, pose);
  sonar::Engine engine(c, HarborWater());
  auto hits = scene.Cast(pose, sonar::RayFan(c), 1.5 * c.rangeMax);
  sonar::AssignFootprints(hits, sonar::RayFanElevationCount(c));
  const auto b = engine.Ping(hits, 0);
  const auto frame = sonar::ReadImaging(
    sonar::ReadLevels(b, c, engine.Absorption(), 0), c);

  const std::string stem = _out + "/gz_sonar_" + _scene + suffix;
  std::vector<float> lin(b.data.begin(), b.data.end());
  WriteNpy(stem + ".npy", lin, b.beams, b.samples, "<f4");
  WriteNpy(stem + "_u8.npy", frame.data, frame.beams, frame.samples, "|u1");
  std::ofstream j(stem + ".json");
  j << "{\n  \"kind\": \"imaging\",\n";
  WriteJsonArray(j, "ranges", b.ranges);
  WriteJsonArray(j, "azimuths", b.azimuths, true);
  j << "}\n";
  std::cout << "imaging scene " << _scene << ": " << stem << ".npy\n";
  return 0;
}

//////////////////////////////////////////////////
int Sidescan(const std::string &_out)
{
  sonar::SonarConfig c = sonar::OmniscanSidescan();
  c.rangeMax = 20.0;
  c.samples = 400;
  // Automatic gain, to show it smoothed across pings.
  c.gainDb.reset();
  const auto scene = PassScene();
  sonar::Engine engine(c, HarborWater());
  const auto rays = sonar::RayFan(c);

  // The boat 3 m up runs along x at y = 0; the transducer looks to port,
  // 20 degrees down. One ping every 0.1 m.
  const std::size_t pings = 250;
  std::vector<float> waterfall;
  std::vector<double> along;
  std::vector<double> tvg;
  // At 1 m/s a ping every 0.1 m is a ping every 0.1 s; automatic gain is
  // smoothed across them.
  sonar::GainTracker gain;
  for (std::size_t i = 0; i < pings; ++i)
  {
    const double x = 0.1 * static_cast<double>(i);
    const Pose3d pose(x, 0, 3, 0, 20.0 * kDeg, 90.0 * kDeg);
    auto hits = scene.Cast(pose, rays, 1.5 * c.rangeMax);
    sonar::AssignFootprints(hits, sonar::RayFanElevationCount(c));
    const auto b = engine.Ping(hits, i);
    const auto line = sonar::ReadSidescan(
      sonar::ReadLevels(b, c, engine.Absorption(), i, &gain, 0.1));
    waterfall.insert(waterfall.end(), line.db.begin(), line.db.end());
    along.push_back(x);
    tvg = line.tvgDb;
  }
  const std::string stem = _out + "/gz_sonar_sidescan";
  WriteNpy(stem + ".npy", waterfall, pings, c.samples, "<f4");
  std::ofstream j(stem + ".json");
  j << "{\n  \"kind\": \"sidescan\",\n";
  WriteJsonArray(j, "ranges", engine.Ping({}, 0).ranges);
  WriteJsonArray(j, "along_track", along);
  WriteJsonArray(j, "tvg_db", tvg, true);
  j << "}\n";
  std::cout << "sidescan pass: " << stem << ".npy\n";
  return 0;
}

//////////////////////////////////////////////////
int Profiling(const std::string &_out)
{
  sonar::SonarConfig c = sonar::SurveyorProfiling();
  c.rangeMax = 30.0;
  const auto scene = PassScene();
  sonar::Engine engine(c, HarborWater());
  const auto rays = sonar::RayFan(c);

  // The boat 3 m up runs along x at y = 4, over the box; the profiler
  // looks straight down with its fan across the track.
  std::ofstream csv(_out + "/gz_sonar_profiling.csv");
  csv << "ping,along_track,angle,range,travel_time,x,y,z,flag\n";
  const std::size_t pings = 250;
  for (std::size_t i = 0; i < pings; ++i)
  {
    const double x = 0.1 * static_cast<double>(i);
    const Pose3d pose(x, 4, 3, 0, 90.0 * kDeg, 0);
    auto hits = scene.Cast(pose, rays, 1.5 * c.rangeMax);
    sonar::AssignFootprints(hits, sonar::RayFanElevationCount(c));
    const auto b = engine.Ping(hits, i);
    const auto soundings = sonar::ReadProfiling(
      sonar::ReadLevels(b, c, engine.Absorption(), i), c, 1500.0, i);
    for (const auto &s : soundings)
    {
      // Georeference with the true pose: the fan is the sensor's x-y
      // plane, so a sounding at angle a is along (cos a, sin a, 0).
      const gz::math::Vector3d p = pose.Pos() + pose.Rot().RotateVector(
        {s.range * std::cos(s.angle), s.range * std::sin(s.angle), 0.0});
      csv << i << "," << x << "," << s.angle << "," << s.range << ","
          << s.travelTime << "," << p.X() << "," << p.Y() << "," << p.Z()
          << "," << static_cast<int>(s.flag) << "\n";
    }
  }
  std::cout << "profiling pass: " << _out << "/gz_sonar_profiling.csv\n";
  return 0;
}
}  // namespace

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  const std::string usage =
    "usage: gz_sonar_render_ping imaging A|B|C <out_dir> [sidelobe=dB] "
    "[dynamic_range=dB] [name=suffix]\n"
    "       gz_sonar_render_ping sidescan <out_dir>\n"
    "       gz_sonar_render_ping profiling <out_dir>\n";
  if (_argc < 3)
  {
    std::cerr << usage;
    return 1;
  }
  const std::string kind = _argv[1];
  if (kind == "imaging" && _argc >= 4)
    return Imaging(_argv[2], _argv[3],
      std::vector<std::string>(_argv + 4, _argv + _argc));
  if (kind == "sidescan")
    return Sidescan(_argv[2]);
  if (kind == "profiling")
    return Profiling(_argv[2]);
  std::cerr << usage;
  return 1;
}
