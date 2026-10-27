# Orchard static map

An occupancy map for documentation and RViz only. Planning and control do
not use it, and launching it does not start `move_base`.

- Frame: `map`, aligned with Gazebo `world`
- Bounds: x `[-11, 11]` m, y `[-17, 17]` m
- Resolution: `0.05` m/cell
- Black: greenhouse boundary and the four tree rows
- White: passages and row-end transfer corridors

Regenerate the PGM after changing the calibration:

```bash
python3 tools/generate_orchard_static_map.py maps/orchard_static.pgm
```

Show the map and the zone overlay in a running simulation:

```bash
roslaunch harbot_orchard_picker orchard_static_map.launch
```
