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
#include "gz/sim/water_optics/Jerlov.hh"

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace gz::sim::water_optics
{
// The Jerlov water types' inherent optical properties, at 625 nm (red),
// 525 nm (green) and 450 nm (blue).
//
// Source: Solonenko, M. G. and Mobley, C. D. (2015), "Inherent optical
// properties of Jerlov water types", Applied Optics 54(17), 5392-5401,
// doi:10.1364/AO.54.005392, Tables 4 to 8: total absorption a and total
// scattering b (water included), and the diffuse attenuation K_d those a
// and b reproduce. Taken from the transcription of those tables in
// https://github.com/tishibashi-cpu/jerlov (Apache-2.0),
// jerlov/data/solonenko2015_iop.csv, which checks the transcription against
// the paper's own equations (its DATA.md); not yet checked against the
// printed tables by us.
//
// One exception, 5C at 625 nm: the printed row of Table 7 there is a
// corrupted copy of the 300-400 nm rows (DATA.md section 1). a and b there
// are the substitute of Williamson, C. A. and Hollins, R. C. (2022),
// Applied Optics 61, 9951, doi:10.1364/AO.470464 (supplementary
// spreadsheet), and K_d is Jerlov's own, Marine Optics (1976), Table XXVII.
namespace
{
const std::vector<JerlovType> kTypes = {
  //  name  absorption a (R, G, B)       scattering b (R, G, B)
  //        diffuse attenuation K_d (R, G, B)
  {"I",   {0.295, 0.046, 0.018},     {0.00104, 0.00205, 0.00381},
          {0.302, 0.049, 0.022}},
  {"IA",  {0.295, 0.0468, 0.0221},   {0.00256, 0.00402, 0.00631},
          {0.303, 0.0503, 0.0264}},
  {"IB",  {0.296, 0.0469, 0.0235},   {0.0468, 0.0565, 0.068},
          {0.310, 0.0572, 0.0342}},
  {"II",  {0.296, 0.0469, 0.0241},   {0.288, 0.387, 0.504},
          {0.337, 0.0845, 0.062}},
  {"III", {0.297, 0.0507, 0.0388},   {0.788, 1.06, 1.38},
          {0.390, 0.129, 0.124}},
  {"1C",  {0.304, 0.068, 0.105},     {0.293, 0.395, 0.514},
          {0.395, 0.121, 0.179}},
  {"3C",  {0.307, 0.078, 0.154},     {0.855, 1.15, 1.50},
          {0.456, 0.187, 0.319}},
  {"5C",  {0.327, 0.127, 0.297},     {1.070, 1.44, 1.87},
          {0.48, 0.277, 0.535}},
  {"7C",  {0.367, 0.233, 0.542},     {1.89, 2.54, 3.30},
          {0.600, 0.470, 0.924}},
  {"9C",  {0.436, 0.430, 0.943},     {2.51, 3.38, 4.39},
          {0.763, 0.826, 1.56}},
};

std::string Upper(std::string _s)
{
  std::transform(_s.begin(), _s.end(), _s.begin(),
                 [](unsigned char _c) { return std::toupper(_c); });
  return _s;
}
}  // namespace

//////////////////////////////////////////////////
std::optional<JerlovType> Jerlov(const std::string &_name)
{
  const std::string name = Upper(_name);
  for (const auto &type : kTypes)
  {
    if (type.name == name)
      return type;
  }
  return std::nullopt;
}

//////////////////////////////////////////////////
const std::vector<JerlovType> &JerlovTypes()
{
  return kTypes;
}

//////////////////////////////////////////////////
Rgb JerlovWavelengths()
{
  return Rgb(625.0, 525.0, 450.0);
}
}  // namespace gz::sim::water_optics
