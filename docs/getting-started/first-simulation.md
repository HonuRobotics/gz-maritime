# First simulation

Start the ocean, put the tutorial boat on it, and drive it. Run every command
in a terminal where you sourced `~/maritime_ws/install/setup.bash`.

## 1. The ocean on its own

```bash
ros2 launch kai_bringup simulation.launch.xml
```

```{figure} images/open-water.jpg
:alt: A calm, rippled ocean stretching to the horizon under a pale sky, with no vehicles

The `open_water` world at its default, calm sea state.
```

Gazebo opens the `open_water` world: a calm sea, a sky and nothing else. No
vehicle is part of this world; vehicles are added to it. Stop it with
Ctrl+C before the next step.

## 2. Put a boat on it

```bash
ros2 launch tutorial_usv_gazebo sim.launch.xml
```

```{figure} images/tutorial-usv.jpg
:alt: The tutorial USV, a twin-hull boat with a grey deck, floating on the ocean

The tutorial USV floating at its waterline.
```

The **tutorial USV** is a 1 m catamaran made for these docs. The world file
doesn't mention it: the boat's own model marks the shapes that float it
([how that works](../how-to/own-vehicle.md#how-a-vehicle-floats)).

Check it from a second terminal:

```bash
gz model -m tutorial_usv -p          # z close to 0: floating at the waterline
ros2 topic list | grep tutorial_usv  # thrust commands and sensors
```

## 3. Drive it

Each propeller takes a thrust command in newtons. Send both at once:

```bash
ros2 topic pub -t 5 -r 5 /tutorial_usv/motor_port/thrust std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -t 5 -r 5 /tutorial_usv/motor_stbd/thrust std_msgs/msg/Float64 "{data: 5.0}" &
wait
```

The boat moves forward. Send `0.0` to both to stop, or two different values
to turn.

```{warning}
Thrust commands **latch**: a propeller keeps its last command until it gets a
new one. Always start both commands together, as above. If one arrives well
before the other, the boat spins before it moves forward.
```

## 4. Look at it in RViz

```bash
ros2 launch tutorial_usv_gazebo rviz.launch.xml
```

```{figure} images/tutorial-usv-rviz.jpg
:alt: RViz showing the tutorial USV as an orange twin-hull model over a grid, with the Displays panel listing a RobotModel whose description topic is /tutorial_usv/robot_description and whose TF Prefix is tutorial_usv

The tutorial USV in RViz, on the config the launch generates for it.
```

RViz shows the boat and its frames. In a simulation every frame carries the
boat's name (`tutorial_usv/base_link`) and the description is published in
the boat's namespace, so the launch starts RViz on a config pointed at that
instance, with simulation time. For another boat, pass its name:
`name:=boat_a`.

## 5. Add a second boat

Stop the simulation and start it with two boats:

```bash
ros2 launch tutorial_usv_gazebo two_usvs.launch.xml
```

Two identical boats, `boat_a` and `boat_b`, float 4 m apart. Each has its own
topics, so this drives only one of them:

```bash
ros2 topic pub -t 5 -r 5 /boat_a/motor_port/thrust std_msgs/msg/Float64 "{data: 5.0}" &
ros2 topic pub -t 5 -r 5 /boat_a/motor_stbd/thrust std_msgs/msg/Float64 "{data: 5.0}" &
wait
```

A third one can join a running simulation:

```bash
ros2 launch tutorial_usv_gazebo spawn.launch.xml name:=boat_c y:=-6 use_composition:=false
```

## 6. Make waves

The sea state can change while the simulation runs, from 0 (glassy) to 9:

```bash
gz service -s /world/default/wave/set_parameters --reqtype gz.msgs.Param \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'params {key: "sea_state" value {type: INT32 int_value: 3}}'
```

The waves pass through the hulls instead of lifting the boats. That is
expected for now: the boats float on the flat water level, and the moving
surface is only drawn. See
[Waves and physics today](../how-to/own-vehicle.md#waves-and-physics-today).

## Next

- [The tutorial USV](../vehicles/tutorial-usv.md): its topics, frames and
  files.
- [Bring your own vehicle](../how-to/own-vehicle.md): build a vehicle like it.
