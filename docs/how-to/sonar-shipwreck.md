# Map a shipwreck

A BlueBoat with a sidescan (a Cerulean Omniscan 450 SS) and a profiling sonar (a Cerulean Surveyor 240-16) runs a lawnmower survey over a shipwreck on a flat sandy seabed 8 m down. One survey gives two maps: a sidescan mosaic, which shows the wreck and its shadows, and the profiler's soundings as a point cloud, which gives its shape. Because the simulation knows the wreck's geometry exactly, the point cloud can be scored against it, and positioning noise can be added to see what it costs. The model behind the sonars is in the [sonar simulation design](../design/sonar.md); the sonars themselves are introduced in [Profiling and sidescan sonar from the USV](sonar-survey.md).

```{figure} ../design/images/shipwreck_survey.svg
:alt: Line drawing of a survey boat on a line beside a broken wreck on the seabed: below the boat a profiler fan reaching the seabed and the wreck, a shaded sidescan swath to the side, profiler soundings as dots, and the wreck's acoustic shadow on the seabed

The survey, sketched (vertical scale ×2): on each line the profiler's fan sounds a strip under the boat, and the sidescan images a wider swath to each side, in which the wreck casts an acoustic shadow away from the track.
```

```{figure} images/sonar-shipwreck-gazebo.png
:alt: Gazebo view from under the water beside the wreck: the long low hull broken into sections on the seabed, the engine house standing up at the right, and the BlueBoat small on the surface above

The same scene in Gazebo, from a camera under the water off the wreck's port side, with the boat on the surface above it.
```

The wreck, `model://shipwreck` in `gz_sonar`, is a wooden steam barge modeled on the Monohansett in Thunder Bay National Marine Sanctuary, at its true size (48.8 by 9.1 m) and broken in three sections. It is an authored approximation, not a survey of the real site. Its meshes are its exact ground truth, and `ground_truth/features.yaml` names its features (the boiler, the engine, the mast, the anchor and others) in the model's frame.

As on the USV page, waves do not move the boat: gz-maritime's waves are drawn but not coupled to buoyancy.

Run everything inside drydock, from `~/maritime_ws`, with the workspace sourced.

## Launch

```bash
ros2 launch blueboat_gazebo sim.launch.xml \
  world:=$(ros2 pkg prefix --share gz_sonar)/worlds/sonar_shipwreck.sdf \
  config_file:=$(ros2 pkg prefix --share blueboat_description)/config/blueboat_sonar.yaml \
  x:=-45 y:=-30
```

The boat starts at the corner of the survey area, south-west of the wreck.

## Survey and record

In one terminal, record; in another, drive:

```bash
ros2 run gz_sonar record_survey.py sonar_shipwreck survey.npz 900
```

```bash
ros2 run gz_sonar survey_lawnmower.py sonar_shipwreck \
  --x0 -40 --x1 45 --y0 -30 --y1 40 --spacing 10
```

`survey_lawnmower.py` runs eight lines along x, 10 m apart, from y = −30 to 40, holding each with a heading and cross-track controller and turning at the ends. At about 1.8 m/s the survey takes about 12.5 minutes of sim time. `record_survey.py` saves every sidescan line, every ping of soundings and the boat's pose, each with its sim time; stop it with Ctrl-C once the survey is done, and it saves what it has.

```{figure} images/sonar-shipwreck-pass.webp
:alt: Animation: above, the Gazebo view of the boat crossing over the wreck; below, the port and starboard waterfalls growing as it goes, the wreck's engine house and its dark shadow scrolling down the starboard side

One line over the wreck, played at about twice its speed: the boat in Gazebo above, and its port (left) and starboard (right) waterfalls below, newest line at the top.
```

The pose here is the simulation's true pose, which a real survey does not have. Both mapping scripts take `--pose-noise` (meters) and `--heading-noise` (degrees) to perturb it, as an imperfect navigation solution would.

## The sidescan mosaic

```bash
ros2 run gz_sonar sidescan_mosaic.py survey.npz mosaic.png
```

`sidescan_mosaic.py` lays each line on the ground with the boat's pose at its time. Each sample's slant range r becomes a ground range sqrt(r² − h²), with the altitude h taken from the line's first seabed return, as a real sidescan's bottom track does, and the sample is laid perpendicular to the heading on its own side. Cells of 0.25 m keep the mean of the samples that fall in them. Lines taken in the turns are left out: laid perpendicular to a heading that is changing, they fan out across the map.

```{figure} images/sonar-shipwreck-mosaic.png
:alt: Two sidescan mosaics of the wreck side by side: on the left, with true poses, the hull's outline, its hatches and the break between sections sharp, and dark shadows beside the taller stern section; on the right, with pose noise, the same wreck blurred and smeared along the lines

The mosaic around the wreck, with true poses (left) and with 0.5 m of position noise and 2° of heading noise on every line (right).
```

With true poses, the hull's outline, its hatches and the breaks between sections are sharp, and the stern section, which stands about 2 m proud, casts dark shadows away from each line that passed it. The mosaic assumes a flat seabed, as a real one does: anything standing proud is laid down at its slant range, a little toward the track. Over the stern, where a line's first return comes from the wreck rather than the seabed, the altitude changes from line to line and the lines are laid down unevenly. The faint bands across the map are the seams where neighboring lines overlap. With noisy poses, each line is laid a little off, and the wreck blurs along the lines: the hull is still there, the hatches are not.

## The soundings as a point cloud

```bash
ros2 run gz_sonar soundings_cloud.py survey.npz cloud.ply
```

`soundings_cloud.py` turns each good sounding, a two-way travel time and an angle in the profiler's fan, into a range along that angle, and places it in the world through the profiler's mount on the boat and the boat's pose at the ping. It writes an ASCII PLY, which any point cloud tool reads, and a plan view colored by height.

```{figure} images/sonar-shipwreck-soundings.png
:alt: Plan view of the soundings around the wreck, colored by height: the open seabed dark at 8 m, the long low forward hull a little lighter, and the intact stern section and the mast standing up in green and yellow

The soundings around the wreck, colored by height: 150,240 points for the whole survey. The forward hull stands a little above the seabed, the stern section about 2 m, and the mast highest of all.
```

## Score it against the wreck

```bash
ros2 run gz_sonar compare_wreck.py cloud.ply $(ros2 pkg prefix --share gz_sonar)/models/shipwreck
```

`compare_wreck.py` reads the wreck's meshes, places them with the wreck's pose in the world (the default is the world's), and measures each sounding's distance to the nearest true surface, the wreck or the flat seabed. It scores the soundings over the wreck apart from those on open seabed:

| Positioning | Over the wreck: median, 95 % within | Open seabed: median, 95 % within |
|---|---|---|
| True poses | 0.027 m, 0.095 m | 0.027 m, 0.087 m |
| 0.5 m and 2° of noise | 0.031 m, 0.154 m | 0.027 m, 0.087 m |
| 1 m and 4° of noise | 0.033 m, 0.243 m | 0.027 m, 0.087 m |

```{figure} images/sonar-shipwreck-error.png
:alt: Histograms of the distance from each sounding over the wreck to the true surface, for true poses and two levels of positioning noise: most within 0.1 m in all three, with a longer tail as the noise grows

The distance from each sounding over the wreck to the true surface, for each level of positioning noise.
```

With true poses, the soundings lie on the true surfaces within the profiler's range and angle noise. Positioning noise moves every sounding of a ping together, which on flat seabed changes nothing, so the open seabed scores the same in all three; over the wreck it puts soundings beside the edges they came from, and the tail grows with the noise. A few soundings, 31 of the 17,489 over the wreck with true poses, lie more than 0.5 m from any surface, all around the mast. A thin pole and the seabed behind it echo at the same range, and the angle measured is likely between the two, as it is on a real angle-of-arrival profiler.

## Check

- The mosaic shows the wreck's outline and the stern section's shadows; with pose noise, the same wreck blurs.
- With true poses, 95 % of the soundings over the wreck lie within 0.1 m of the true surface.
- With positioning noise, the error over the wreck grows with the noise, and on open seabed it does not change.
