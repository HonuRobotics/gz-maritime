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
#ifndef GZ_MARITIME_ACOUSTICRECEIVER_HH_
#define GZ_MARITIME_ACOUSTICRECEIVER_HH_

#include <array>
#include <chrono>
#include <mutex>

#include <gz/msgs/param.pb.h>

#include <gz/sensors/Noise.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Sensor.hh>

namespace gz::sim::maritime
{
  /// \brief What a receiver can be tuned to while the world runs, and the
  /// rules it hears by.
  struct ReceiverSettings
  {
    /// \brief Hz, the centre of the band it listens to; 0 hears every
    /// frequency.
    public: double frequency{0.0};

    /// \brief Hz, the full width of the band.
    public: double bandwidth{1000.0};

    /// \brief Whether a reading carries the slant range.
    public: bool range{false};

    /// \brief Chance that a reading's bearing is anything at all, a stand in
    /// for multipath.
    public: double outlierProbability{0.0};

    /// \brief Fraction of a pinger's reach where pings start to drop; 1 is
    /// no dropout.
    public: double dropoutStart{1.0};
  };

  /// \brief One ping, as a receiver reports it.
  struct ReceiverReading
  {
    /// \brief rad, counter clockwise from the sensor's x.
    public: double bearing{0.0};

    /// \brief rad, positive up.
    public: double elevation{0.0};

    /// \brief m, straight line.
    public: double range{0.0};

    /// \brief Hz of the ping.
    public: double frequency{0.0};

    /// \brief dB relative to 1 m.
    public: double amplitude{0.0};

    /// \brief False when another ping in band arrived on top of it.
    public: bool valid{true};

    /// \brief When the pinger sent it.
    public: std::chrono::steady_clock::duration emitted{0};

    /// \brief When it reached the receiver.
    public: std::chrono::steady_clock::duration arrived{0};
  };

  /// \brief An acoustic receiver: a gz-sensors custom sensor,
  /// `<sensor type="custom" gz:type="acoustic_receiver">`, that reports
  /// every ping it hears as a gz.msgs.Param on its `<topic>`, with its
  /// `<frame_id>` in the header.
  ///
  /// The acoustics system decides what it hears and hands each ping over
  /// with Publish; this class owns the tuning, the noise and the message.
  /// It has no rate of its own: one reading per ping heard, so its
  /// `<update_rate>` is ignored.
  ///
  /// The message's keys: `bearing` (rad, counter clockwise from the sensor's
  /// x), `elevation` (rad, positive up), `frequency` (Hz), `amplitude` (dB
  /// relative to 1 m, -20 log10 of the distance), `valid` (boolean),
  /// `emit_time` (s of simulation time the pinger sent it) and, with
  /// `<range>` on, `range` (m). The header stamp is when it arrived.
  ///
  /// Its `<gz:acoustic_receiver>` block:
  ///
  /// * `<frequency>`: Hz, the centre of its band; 0 (default) hears all.
  /// * `<bandwidth>`: Hz, the band's full width, default 1000.
  /// * `<range>`: report the slant range too, default false; a passive
  ///   receiver hears a direction, not a distance.
  /// * `<bearing_noise>`, `<elevation_noise>`, `<range_noise>`: each a
  ///   standard gz-sensors `<noise>` block. The bearing is wrapped into
  ///   (-pi, pi], the elevation clamped to [-pi/2, pi/2] and the range kept
  ///   positive.
  /// * `<outlier_probability>`: chance a reading's bearing is uniformly
  ///   random, default 0.
  /// * `<dropout_start>`: fraction of a pinger's reach where its pings start
  ///   to drop, the chance rising in a straight line to certain at the
  ///   reach; 1 (default) is no dropout.
  ///
  /// It is retuned on `<topic>/set`, a gz.msgs.Param with `frequency` and
  /// `bandwidth`.
  class AcousticReceiver : public sensors::Sensor
  {
    /// \brief The three measured channels, for noise.
    public: enum class Channel { kBearing = 0, kElevation = 1, kRange = 2 };

    // Documentation inherited.
    public: bool Load(const sdf::Sensor &_sdf) override;

    /// \brief The base class's rate check, unused.
    public: using sensors::Sensor::Update;

    /// \brief Does nothing: the receiver publishes when a ping arrives.
    /// \param[in] _now Unused.
    /// \return True.
    public: bool Update(const std::chrono::steady_clock::duration &_now)
        override;

    // Documentation inherited.
    public: bool HasConnections() const override;

    /// \brief The settings as they are now.
    /// \return A copy.
    public: ReceiverSettings Settings() const;

    /// \brief Apply a channel's noise.
    /// \param[in] _channel The channel.
    /// \param[in] _value The true value.
    /// \return The value with noise, unchanged without a noise block.
    public: double ApplyNoise(Channel _channel, double _value);

    /// \brief Publish one reading.
    /// \param[in] _reading The reading.
    /// \return True if published.
    public: bool Publish(const ReceiverReading &_reading);

    /// \brief Handler of the set topic, on a transport thread.
    /// \param[in] _msg Parameter names and values.
    private: void OnSet(const msgs::Param &_msg);

    /// \brief The settings.
    private: ReceiverSettings settings;

    /// \brief Guards the settings across the transport and ECM threads.
    private: mutable std::mutex mutex;

    /// \brief Noise per channel; null without noise.
    private: std::array<sensors::NoisePtr, 3> noise;

    /// \brief Transport node.
    private: transport::Node node;

    /// \brief Publisher of the readings.
    private: transport::Node::Publisher pub;
  };
}

#endif
