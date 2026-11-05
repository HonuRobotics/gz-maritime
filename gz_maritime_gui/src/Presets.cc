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
#include "gz/sim/maritime_gui/Presets.hh"

#include <tinyxml2.h>

#include <gz/common/Console.hh>

namespace gz::sim::maritime_gui
{
//////////////////////////////////////////////////
std::vector<Preset> DefaultPresets()
{
  std::vector<Preset> p(4);
  p[0].name = "Calm";
  p[0].current = FieldTarget{0.0, 0.0};
  p[0].wind = FieldTarget{0.0, 270.0};
  p[0].gusts = Gusts{0.0, 0.0};
  p[0].dwell = 12.0;

  p[1].name = "Tidal ebb";
  p[1].current = FieldTarget{0.8, 225.0};
  p[1].wind = FieldTarget{3.0, 200.0};
  p[1].gusts = Gusts{0.5, 5.0};
  p[1].dwell = 25.0;

  p[2].name = "Fresh breeze";
  p[2].current = FieldTarget{0.3, 90.0};
  p[2].wind = FieldTarget{10.0, 270.0};
  p[2].gusts = Gusts{2.0, 10.0};
  p[2].dwell = 25.0;

  p[3].name = "Squall";
  p[3].current = FieldTarget{0.5, 135.0};
  p[3].wind = FieldTarget{18.0, 300.0};
  p[3].gusts = Gusts{5.0, 25.0};
  p[3].dwell = 20.0;
  return p;
}

//////////////////////////////////////////////////
std::vector<Preset> ParsePresets(const tinyxml2::XMLElement *_elem)
{
  std::vector<Preset> out;
  if (!_elem)
    return out;
  for (auto *e = _elem->FirstChildElement("preset"); e;
       e = e->NextSiblingElement("preset"))
  {
    Preset p;
    if (const char *name = e->Attribute("name"))
      p.name = name;
    e->QueryDoubleAttribute("dwell", &p.dwell);
    if (const auto *c = e->FirstChildElement("current"))
    {
      FieldTarget t;
      c->QueryDoubleAttribute("speed", &t.speed);
      c->QueryDoubleAttribute("direction", &t.direction);
      p.current = t;
    }
    if (const auto *w = e->FirstChildElement("wind"))
    {
      FieldTarget t;
      w->QueryDoubleAttribute("speed", &t.speed);
      w->QueryDoubleAttribute("direction", &t.direction);
      p.wind = t;
      if (w->Attribute("speed_gust") || w->Attribute("direction_gust"))
      {
        Gusts g;
        w->QueryDoubleAttribute("speed_gust", &g.speed);
        w->QueryDoubleAttribute("direction_gust", &g.direction);
        p.gusts = g;
      }
    }
    if (p.name.empty() || (!p.current && !p.wind))
    {
      gzwarn << "ScenarioPanel: a <preset> needs a name and a <current> or a "
             << "<wind>; skipped\n";
      continue;
    }
    if (!(p.dwell > 0.0))
      p.dwell = 20.0;
    out.push_back(p);
  }
  return out;
}

//////////////////////////////////////////////////
void Demo::Start()
{
  this->running = true;
  this->index.reset();
}

//////////////////////////////////////////////////
void Demo::Stop()
{
  this->running = false;
  this->index.reset();
}

//////////////////////////////////////////////////
bool Demo::Running() const
{
  return this->running;
}

//////////////////////////////////////////////////
std::optional<std::size_t> Demo::Current() const
{
  return this->running ? this->index : std::nullopt;
}

//////////////////////////////////////////////////
std::optional<std::size_t> Demo::Update(const std::vector<Preset> &_presets,
    const std::chrono::steady_clock::duration &_simTime)
{
  if (!this->running || _presets.empty())
    return std::nullopt;
  if (!this->index || _simTime < this->since)
  {
    // Just started, or the world was reset: the first preset.
    this->index = 0u;
    this->since = _simTime;
    return this->index;
  }
  const double dwell = _presets[*this->index % _presets.size()].dwell;
  if (std::chrono::duration<double>(_simTime - this->since).count() < dwell)
    return std::nullopt;
  this->index = (*this->index + 1u) % _presets.size();
  this->since = _simTime;
  return this->index;
}
}  // namespace gz::sim::maritime_gui
