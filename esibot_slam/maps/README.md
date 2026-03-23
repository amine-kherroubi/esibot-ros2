# maps/

This directory stores SLAM maps saved by `save_map.launch.py`.

After running the mapping session, save the map with:

```bash
ros2 launch esibot_slam save_map.launch.py map_name:=esibot_map
```

This generates two files here:

| File | Description |
|---|---|
| `esibot_map.pgm` | Grayscale PGM image — white=free, black=obstacle, grey=unknown |
| `esibot_map.yaml` | Metadata for Nav2 — resolution, origin, free/occupied thresholds |

These two files are **Task 3.5 Deliverable #4** (map of a minimum 3 m × 3 m area).

They are also the input for **Task 3.6** (Nav2 autonomous navigation):

```bash
ros2 launch esibot_bringup navigation.launch.py \
    map:=$(ros2 pkg prefix esibot_slam)/share/esibot_slam/maps/esibot_map.yaml
```
