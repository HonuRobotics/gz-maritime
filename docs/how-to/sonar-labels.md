# Generate labeled training data

For every ping, the sonar can publish which surface sent each beam and sample its echo: ground truth for training and scoring detectors. A real sonar cannot know this, so it is on its own topic, never in the sensor data.

## Turn on the labels

```xml
<gz:sonar>
  <labels>true</labels>
</gz:sonar>
```

The sensor then publishes `<topic>/labels` beside its image; in a BlueROV2 config it is `sonar: {labels: true}`, as in `bluerov2_sonar.yaml`.

## Record

With the [first-look world](sonar-first-look.md) running:

```bash
for i in 1 2 3 4 5; do
  ros2 run gz_sonar capture_sonar.py /bluerov2/sonar_forward/image /tmp/dataset \
    --name ping_$i --count $((i * 10)) --labels
done
```

Each capture writes the 8-bit frame (`_u8.npy`), the labels (`_labels.npy`, one id per beam and sample, 0 for nothing) and a `.json` with the ranges, the beam angles and the name of each id.

## Check

```bash
python3 -c "
import json, numpy as np
f = np.load('/tmp/dataset/gz_sonar_ping_1_u8.npy')
ids = np.load('/tmp/dataset/gz_sonar_ping_1_labels.npy')
names = json.load(open('/tmp/dataset/gz_sonar_ping_1.json'))['label_names']
for i in np.unique(ids):
    if i:
        print(names[i], int((ids == i).sum()), 'cells, mean', round(float(f[ids == i].mean())))"
```

Each target's cells carry its label, and the targets read brighter than the seabed around them.
