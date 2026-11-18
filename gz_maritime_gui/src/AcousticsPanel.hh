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
#ifndef GZ_SIM_MARITIME_GUI_ACOUSTICSPANEL_HH_
#define GZ_SIM_MARITIME_GUI_ACOUSTICSPANEL_HH_

#include <memory>

#include <QString>
#include <QVariantList>
#include <QVariantMap>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz::sim::maritime_gui
{
  class AcousticsPanelPrivate;

  /// \brief See the acoustic pingers and what the receivers hear, and
  /// switch the pingers.
  ///
  /// A GUI system with a panel. It reads the pingers from the acoustics
  /// system's list on `/world/<world>/acoustics/pingers` and, newer, from
  /// the AcousticPinger components in the GUI's ECM, and lists them with
  /// their frequency, period, offset and reach, each with a switch that sends
  /// `active` on the pinger's `set` topic. It hears the receivers the
  /// acoustics system lists on `/world/<world>/acoustics/receivers`, and each
  /// one's readings, and lists them with their band and last bearing, each
  /// retuned on its `set` topic. It writes nothing into the GUI's ECM.
  ///
  /// It draws, each drawing switched on its own, in a colour per frequency
  /// (25 kHz orange to 40 kHz violet), through MarkerManager:
  ///
  /// * the reach: the circle where each pinger's reach meets the plane drawn
  ///   on, grey and dashed when it is off;
  /// * the pings: a ring per ping, where its sound meets that plane,
  ///   spreading at the speed of sound slowed down so the eye can follow it,
  ///   fading out at the reach; only pings that were sent, and at most a
  ///   few rings per pinger;
  /// * the readings: a ray from each receiver along the bearing and
  ///   elevation of its last reading, from the pose it had when the reading
  ///   came, a while after it came; red when the reading was invalid.
  ///
  /// The reach and the pings are drawn on a horizontal plane: the water, as
  /// a boat sees them; a depth under it; or the depth of a receiver,
  /// following it, the slice a vehicle under the water listens in; or in 3D,
  /// the reach and each ping as wireframe spheres cut at the water. Under the
  /// water they are behind the drawn sea, seen from an underwater camera or
  /// with the water hidden.
  ///
  /// The reach and the rays are sent when they change, paused or not.
  ///
  /// A plugin added from the menu after the world is up has in the ECM
  /// only the pingers that changed since; the list fills in the rest. It
  /// never asks the world's state service.
  ///
  /// Configuration, all optional:
  ///
  /// * `<reach>`, `<pings>`, `<readings>`: draw each, default false.
  /// * `<speed_of_sound>`: m/s, the world's, default 1500.
  /// * `<slow_down>`: how many times slower than sound the rings spread,
  ///   default 50.
  /// * `<max_ring_radius>`: m, the farthest a ring is drawn for a pinger
  ///   with no reach limit, default 200.
  /// * `<ray_length>`: m, default 30.
  /// * `<ray_lifetime>`: s of simulation time a reading's ray stays,
  ///   default 1.5.
  /// * `<plane>`: what the reach and the pings are drawn on, `water`
  ///   (default), `depth` or `receiver`; or `3d`, as wireframe spheres cut
  ///   at the water.
  /// * `<plane_depth>`: m under the water, for `depth`, default 5.
  /// * `<plane_receiver>`: the topic of the receiver whose depth `receiver`
  ///   follows; default the first receiver listed.
  /// * `<water_level>`: world z of the water, default 0.
  /// * `<marker_service>`: MarkerManager's service, default `/marker`.
  class AcousticsPanel : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief The pingers, each a map: name, frequency (kHz), period,
    /// offset, reach, active, colour.
    Q_PROPERTY(QVariantList pingers READ Pingers NOTIFY PingersChanged)

    /// \brief The receivers, each a map: name, topic, frequency (kHz),
    /// bandwidth (kHz), colour.
    Q_PROPERTY(QVariantList receivers READ Receivers NOTIFY ReceiversChanged)

    /// \brief The last reading of each receiver, as text, by topic.
    Q_PROPERTY(QVariantMap readings READ Readings NOTIFY ReadingsChanged)

    /// \brief Draw the reach.
    Q_PROPERTY(bool showReach READ ShowReach WRITE SetShowReach
               NOTIFY ShowReachChanged)

    /// \brief Draw the pings.
    Q_PROPERTY(bool showPings READ ShowPings WRITE SetShowPings
               NOTIFY ShowPingsChanged)

    /// \brief Draw the readings.
    Q_PROPERTY(bool showReadings READ ShowReadings WRITE SetShowReadings
               NOTIFY ShowReadingsChanged)

    /// \brief How many times slower than sound the rings spread. A signal
    /// of its own, as each drawing property has: one shared by all would
    /// put the old value back in the field while a new one is typed and a
    /// check box ticked.
    Q_PROPERTY(double slowDown READ SlowDown NOTIFY SlowDownChanged)

    /// \brief What the reach and the pings are drawn on: water, depth,
    /// receiver or 3d.
    Q_PROPERTY(QString plane READ CurrentPlane NOTIFY PlaneChanged)

    /// \brief m under the water, for the depth plane.
    /// A signal of its own, as each drawing property has: one shared with
    /// the plane would put the old depth back in the field while a new one
    /// is typed and another option changed.
    Q_PROPERTY(double planeDepth READ PlaneDepth NOTIFY PlaneDepthChanged)

    /// \brief The topic of the receiver the receiver plane follows.
    Q_PROPERTY(QString planeReceiver READ PlaneReceiver
               NOTIFY PlaneReceiverChanged)

    /// \brief The world z drawn at now, as text.
    Q_PROPERTY(QString planeText READ PlaneText NOTIFY StatusChanged)

    /// \brief A short sentence on what is happening.
    Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)

    /// \brief Constructor.
    public: AcousticsPanel();

    /// \brief Destructor.
    public: ~AcousticsPanel() override;

    // Documentation inherited.
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited.
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override;

    /// \brief Switch a pinger on or off.
    /// \param[in] _name The pinger.
    /// \param[in] _active On.
    public: Q_INVOKABLE void SetActive(const QString &_name, bool _active);

    /// \brief Retune a receiver.
    /// \param[in] _topic The receiver's topic.
    /// \param[in] _kHz Its band's centre in kHz, as typed, a decimal point
    /// or a comma; 0 hears every frequency.
    /// \return False if the text is not a number, or negative.
    public: Q_INVOKABLE bool SetReceiverFrequency(const QString &_topic,
                                                  const QString &_kHz);

    /// \brief Set how many times slower than sound the rings spread.
    /// \param[in] _text A number of at least 1, as typed.
    /// \return False if it is not one.
    public: Q_INVOKABLE bool SetSlowDown(const QString &_text);

    /// \brief Draw the reach and the pings on another plane.
    /// \param[in] _plane water, depth, receiver or 3d.
    public: Q_INVOKABLE void SetPlane(const QString &_plane);

    /// \brief Set the depth of the depth plane.
    /// \param[in] _text m under the water, not negative, as typed.
    /// \return False if it is not one.
    public: Q_INVOKABLE bool SetPlaneDepth(const QString &_text);

    /// \brief Follow another receiver's depth.
    /// \param[in] _topic The receiver's topic.
    public: Q_INVOKABLE void SetPlaneReceiver(const QString &_topic);

    /// \brief What the reach and the pings are drawn on.
    /// \return water, depth, receiver or 3d.
    public: QString CurrentPlane() const;

    /// \brief The depth of the depth plane.
    /// \return m under the water.
    public: double PlaneDepth() const;

    /// \brief The receiver the receiver plane follows.
    /// \return Its topic, empty for the first listed.
    public: QString PlaneReceiver() const;

    /// \brief Where the plane is now.
    /// \return A short text.
    public: QString PlaneText() const;

    /// \brief The pingers.
    /// \return The list.
    public: QVariantList Pingers() const;

    /// \brief The receivers.
    /// \return The list.
    public: QVariantList Receivers() const;

    /// \brief The last readings.
    /// \return Text by topic.
    public: QVariantMap Readings() const;

    /// \brief Whether the reach is drawn.
    /// \return True if it is.
    public: bool ShowReach() const;

    /// \brief Draw the reach or not.
    /// \param[in] _show Draw it.
    public: void SetShowReach(bool _show);

    /// \brief Whether the pings are drawn.
    /// \return True if they are.
    public: bool ShowPings() const;

    /// \brief Draw the pings or not.
    /// \param[in] _show Draw them.
    public: void SetShowPings(bool _show);

    /// \brief Whether the readings are drawn.
    /// \return True if they are.
    public: bool ShowReadings() const;

    /// \brief Draw the readings or not.
    /// \param[in] _show Draw them.
    public: void SetShowReadings(bool _show);

    /// \brief How many times slower than sound the rings spread.
    /// \return The factor.
    public: double SlowDown() const;

    /// \brief What is happening.
    /// \return A short sentence.
    public: QString Status() const;

    /// \brief The pingers changed.
    signals: void PingersChanged();

    /// \brief The receivers changed.
    signals: void ReceiversChanged();

    /// \brief A reading came.
    signals: void ReadingsChanged();

    /// \brief The reach is drawn, or no longer.
    signals: void ShowReachChanged();

    /// \brief The pings are drawn, or no longer.
    signals: void ShowPingsChanged();

    /// \brief The readings are drawn, or no longer.
    signals: void ShowReadingsChanged();

    /// \brief The rings spread at another speed.
    signals: void SlowDownChanged();

    /// \brief The reach and the pings are drawn on another plane, or in 3D.
    signals: void PlaneChanged();

    /// \brief The depth of the depth plane changed.
    signals: void PlaneDepthChanged();

    /// \brief The receiver plane follows another receiver.
    signals: void PlaneReceiverChanged();

    /// \brief The status changed.
    signals: void StatusChanged();

    /// \brief Private data pointer.
    private: std::unique_ptr<AcousticsPanelPrivate> dataPtr;
  };
}

#endif  // GZ_SIM_MARITIME_GUI_ACOUSTICSPANEL_HH_
