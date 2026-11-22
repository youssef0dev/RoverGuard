# RoverGuard architecture

## Data flow

```text
Mission target
    -> Occupancy grid
    -> A* planner
    -> Safe path points
    -> Path follower
    -> linear/angular velocity
    -> differential-drive kinematics
    -> wheel velocities
    -> PyBullet now / C++ HAL later
```

Sensor kaymchi l `SafetySupervisor` b tari9 mosta9il. Ya3ni safety t9der
tw9ef robot wakha planner/controller 3taw command dyal l9dam.

## Modules

- `config.py`: ga3 dimensions w thresholds f blassa wa7da.
- `grid_map.py`: l-ard m9soma l moraba3at free/occupied.
- `astar.py`: a9sar safe path f grid.
- `path_follower.py`: kaytbe3 path point b point.
- `controllers.py`: kay7awwel position error l linear/angular speed.
- `kinematics.py`: kay7awwel body speed l sor3a dyal kola roue.
- `odometry.py`: kay9ddar pose mn wheel angles.
- `safety.py`: CLEAR/SLOW/STOP m3a hysteresis.
- `simulation.py`: PyBullet ghir adapter, machi source dyal core logic.

## 3lach had ta9sim mohim?

Mlli nbdlo simulator b robot 7a9i9i, A*, mission, kinematics w safety logic
ma-khas-homch ytktbo mn jdid. Kanbdlo ghir layer li katqra sensors w katktb
commands l motors.

