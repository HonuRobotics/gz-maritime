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
#include "gz/sim/ocean_current/OceanCurrentModel.hh"

#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <gz/math/Angle.hh>

#include "StandardModel.hh"

namespace gz::sim::ocean_current
{
namespace
{
/// \brief The registry, with the model built in already in it. A function
/// local static, so it exists before any caller, whatever the load order.
struct Registry
{
  std::mutex mutex;
  std::map<std::string, OceanCurrentModelFactory> factories{
    {"standard", [] { return std::make_unique<StandardModel>(); }},
  };
};

Registry &Instance()
{
  static Registry registry;
  return registry;
}
}  // namespace

//////////////////////////////////////////////////
void RegisterOceanCurrentModelFactory(const std::string &_name,
                                      OceanCurrentModelFactory _factory)
{
  auto &r = Instance();
  const std::lock_guard<std::mutex> lock(r.mutex);
  r.factories[_name] = std::move(_factory);
}

//////////////////////////////////////////////////
std::unique_ptr<IOceanCurrentModel> CreateOceanCurrentModel(
    const std::string &_name, const OceanCurrentParameters &_params)
{
  OceanCurrentModelFactory factory;
  {
    auto &r = Instance();
    const std::lock_guard<std::mutex> lock(r.mutex);
    const auto it = r.factories.find(_name);
    if (it == r.factories.end() || !it->second)
      return nullptr;
    factory = it->second;
  }
  auto model = factory();
  if (model)
    model->SetParameters(_params);
  return model;
}

//////////////////////////////////////////////////
math::Vector3d SetVector(double _speed, double _directionDeg)
{
  // A current sets towards its direction: 90, clockwise from north, is east.
  const double a = GZ_DTOR(_directionDeg);
  return {_speed * std::sin(a), _speed * std::cos(a), 0.0};
}
}  // namespace gz::sim::ocean_current
