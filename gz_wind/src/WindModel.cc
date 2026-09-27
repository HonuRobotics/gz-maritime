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
#include "gz/sim/wind/WindModel.hh"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "StandardModel.hh"

namespace gz::sim::wind
{
namespace
{
/// \brief The registry, with the built in models already in it. A function
/// local static, so it exists before any caller, whatever the load order.
struct Registry
{
  std::mutex mutex;
  std::map<std::string, WindModelFactory> factories{
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
void RegisterWindModelFactory(const std::string &_name,
                              WindModelFactory _factory)
{
  auto &r = Instance();
  const std::lock_guard<std::mutex> lock(r.mutex);
  r.factories[_name] = std::move(_factory);
}

//////////////////////////////////////////////////
std::unique_ptr<IWindModel> CreateWindModel(const std::string &_name,
                                            const WindParameters &_params)
{
  WindModelFactory factory;
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
}  // namespace gz::sim::wind
