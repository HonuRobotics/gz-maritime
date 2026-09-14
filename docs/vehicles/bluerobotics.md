# BlueBoat and BlueROV2

```{figure} images/blueboat.jpg
:alt: The BlueBoat, a dark blue twin-hull surface vehicle with an antenna mast, floating on the ocean

The BlueBoat on the open water world.
```

The Blue Robotics vehicles from
[bluerobotics_models](https://github.com/HonuRobotics/bluerobotics_models)
run on this ocean too. Build that repository in the same workspace:

```bash
cd ~/maritime_ws/src
git clone https://github.com/HonuRobotics/bluerobotics_models.git
cd ~/maritime_ws
rosdep install --from-paths src --ignore-src -y
colcon build --merge-install
source install/setup.bash
```

Each vehicle keeps its floating volume on a link of its own
(`hull_displacement` for the boat, `buoyancy_displacement` for the ROV), so
that its many other collision shapes (brackets, thrusters, sensors) don't add
buoyancy. On this ocean that link floats once its collisions are marked
`gz:buoyancy="true"`, which is the same one-attribute change the vehicle
generators are gaining under the
[multi-vehicle plan](https://github.com/HonuRobotics/gz-maritime/blob/lyrical/MULTI_VEHICLE_PLAN.md).

```{admonition} Interim
:class: note

Until the generators emit the mark and each vehicle ships its own spawn
launch, the recipe below marks the generated model by hand and spawns it
next to gz-maritime's simulation launch. It is three commands per vehicle.
```

## BlueBoat

Generate the model, mark its displacement collisions, and put it on the
ocean:

```bash
ros2 run blueboat_gazebo configure_vehicle.py \
  --config $(ros2 pkg prefix --share blueboat_description)/config/blueboat.yaml \
  --out-dir ~/blueboat
sed -i 's|<collision name="pontoon_|<collision gz:buoyancy="true" name="pontoon_|' ~/blueboat/model.sdf

ros2 launch kai_bringup simulation.launch.xml &
ros2 run ros_gz_sim create -name blueboat -file ~/blueboat/model.sdf -z 0.05
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/blueboat/ros_gz_bridge.yaml
```

The boat floats at its waterline. Its topics (`/blueboat/motor_port/thrust`,
`/blueboat/ping/range`, ...) are described in the bluerobotics_models
documentation. The generated model already declares the `gz` namespace, and
the pontoon boxes are on their own link, so the mark is all they need.

## BlueROV2

```bash
ros2 run bluerov2_gazebo configure_vehicle.py \
  --config $(ros2 pkg prefix --share bluerov2_description)/config/bluerov2.yaml \
  --out-dir ~/bluerov2
sed -i 's|<collision name="displacement">|<collision gz:buoyancy="true" name="displacement">|' ~/bluerov2/model.sdf

ros2 launch kai_bringup simulation.launch.xml &
ros2 run ros_gz_sim create -name bluerov2 -file ~/bluerov2/model.sdf -z 0.5
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/bluerov2/ros_gz_bridge.yaml
```

The ROV drops in from half a metre up. It is trimmed almost exactly neutral
(2 g of net buoyancy), so it settles under the surface and stays roughly
where it stopped.

## Without editing the model

A copy of the world can float a vehicle by name instead, the way the
vehicles' own worlds do. Add to the buoyancy plugin in your copy of
`open_water.sdf`:

```xml
<enable>blueboat::hull_displacement</enable>
```

and launch with `world:=` pointing at the copy. This works today with the
unmodified generated models, at the price of a world that knows the vehicle's
name ([Use your own world](../how-to/own-world.md#the-buoyancy-system)).
