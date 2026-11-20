# Make objects visible to the sonar

Every surface echoes; how strongly is set in SDF. Nothing has to be marked for the sonar to see it, so mark only the objects a scene depends on. The rules and a sense of scale for each value are in the [design](../design/sonar.md#acoustic-properties-of-the-scene).

## Mark a surface

```xml
<visual name="glass">
  <geometry><sphere><radius>0.2</radius></sphere></geometry>
  <gz:acoustic_lambert>-30</gz:acoustic_lambert>
  <gz:acoustic_specular>-3</gz:acoustic_specular>
  <gz:acoustic_specular_width>0.07</gz:acoustic_specular_width>
</visual>
```

Lambert strength (dB) is how much a rough surface scatters back; the specular terms (dB, and width in radians) add a glint seen only head on, as glass gives. The same elements directly under `<model>` apply to all its visuals.

## Hide a surface

```xml
<model name="flag">
  <gz:acoustic_transparent/>
  ...
</model>
```

For things that are there only for the cameras: the sonar sees through them.

## Name the seabed

```xml
<plugin filename="gz-maritime-sonar-system" name="gz::sim::maritime::Sonar">
  <seabed>
    <model>seabed</model>
    <lambert>-27</lambert>
  </seabed>
  <default><lambert>-20</lambert></default>
</plugin>
```

Terrain often comes from models you do not edit, so its values are set in the world; `<default>` covers anything unmarked.

## See it

The [first-look world](sonar-first-look.md) marks its targets this way: launch it, and record a ping with its ground truth.

```bash
ros2 run gz_sonar capture_sonar.py /bluerov2/sonar_forward/image /tmp/sonar --name look --count 20 --labels
python3 -c "
import json, numpy as np
ids = np.load('/tmp/sonar/gz_sonar_look_labels.npy')
names = json.load(open('/tmp/sonar/gz_sonar_look.json'))['label_names']
print(sorted({names[i] for i in np.unique(ids) if i}))"
```

## Check

The list names the seabed and the targets in view, and never the flag, which the sonar does not see; in the fan, the glass float glints brightly only where the sonar looks at it head on, while the tire returns a weak, steady echo.
