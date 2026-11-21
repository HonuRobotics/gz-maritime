# Set the difficulty

A few parameters make a sonar image easier or harder to work with, and a seed makes any run repeatable. All of them are in the [design's parameter tables](../design/sonar.md#sdf).

## Raise the noise

```bash
sed 's/labels: true}/labels: true, noise_floor: -25}/' \
  $(ros2 pkg prefix --share bluerov2_description)/config/bluerov2_sonar.yaml > /tmp/noisy.yaml
ros2 launch bluerov2_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share gz_sonar)/worlds/sonar_first_look.sdf config_file:=/tmp/noisy.yaml
```

`noise_floor` is the noise level below full scale (dB): from −50 by default to −25 here, the weaker targets sink into it.

| Parameter | Effect | Easier | Harder |
|---|---|---|---|
| `noise_floor` | Additive noise, dB re full scale | −60 | −20 |
| `gain` | `auto`, or a fixed gain in dB | `auto` | a fixed gain too low or too high |
| `speckle` | The grain of a real sonar image | `false` | `true` |
| `range_max` | The range window, m | close to the targets | much longer |

## Replay a run

```bash
ros2 run gz_sonar capture_sonar.py /bluerov2/sonar_forward/image /tmp/run1 --name p --ping 400 --timeout 120
# stop the launch, launch it again the same way, then
ros2 run gz_sonar capture_sonar.py /bluerov2/sonar_forward/image /tmp/run2 --name p --ping 400 --timeout 120
sha256sum /tmp/run1/gz_sonar_p_u8.npy /tmp/run2/gz_sonar_p_u8.npy
```

Ping 400 (40 s in) of the same world, seed and motion is the same bytes in every run; the world's sonar system takes `<seed>` to make a different, equally repeatable, run.

## Check

With the noise raised, the tire and the far edge of the fan fade into the noise; the two checksums are equal.
