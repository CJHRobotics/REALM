# simulation_lib

Python library for loading, representing, and tooling around REALM maze
environments defined in XML.

---

## Modules

| Module | Purpose |
|---|---|
| `environment_parser.py` | Parses a maze XML file into plain Python dicts |
| `environment.py` | Converts parsed data into typed objects (`Wall`, `Landmark`, `StartingPosition`, etc.) and produces Webots node strings |
| `start_position_generator.py` | Generates a uniform grid of valid robot start positions from a maze XML and saves them to CSV |
| `webots_torch_environment.py` | Gymnasium environment skeleton for RL agents |

---

## Maze XML Format

All maze files live under `simulation/worlds/environments/`.  The root tag
can be `<world>` or `<environment>` — the parser accepts both.

### Walls

```xml
<wall x1=" 2.61" y1=" 0.00" x2=" 1.85" y2=" 1.85"
      type="boundary" height="0.5" width="0.012"/>

<wall x1=" 0.75" y1="-1.25" x2="-0.75" y2="-1.25"
      type="obstacle" height="0.5" width="0.012"/>
```

| Attribute | Description |
|---|---|
| `x1, y1, x2, y2` | Wall endpoints in metres |
| `type` | `boundary` — forms the outer perimeter; `obstacle` — interior wall |
| `height` | Wall height in metres (default `0.3`) |
| `width` | Wall thickness in metres (default `0.012`) |

Boundary walls must connect end-to-end to form a closed convex polygon.

### Landmarks

```xml
<landmark type="panel"
          x=" 2.23" y=" 0.93" theta="-2.749"
          height="1.0" width="1.0"
          texture="../protos/world_objects/textures/april_tags/landmark_tag_0.png"
          red="1.00" green="0.00" blue="0.00"/>

<landmark type="cylinder"
          x="1.0" y="0.0" theta="0.0"
          height="0.5" radius="0.1"
          red="0.00" green="0.00" blue="1.00"/>
```

| Attribute | Description |
|---|---|
| `type` | `panel` — rectangular sign panel; `cylinder` — cylindrical post |
| `x, y` | Position in metres |
| `theta` | Yaw in radians (panel faces inward toward origin by convention) |
| `texture` | Path to image file — `panel` only |
| `radius` | Cylinder radius in metres — `cylinder` only |
| `red, green, blue` | Recognition colour (0–1) used by the Webots camera |

### Start positions

```xml
<train_start_positions>
    <pos x="-1.5" y="0.0" theta="0.0"/>
</train_start_positions>

<test_start_positions>
    <pos x="-1.5" y="0.0" theta="0.0"/>
</test_start_positions>
```

These seed positions are used at runtime.  For habituation, a full grid of
positions is generated separately — see [Start Position Generator](#start-position-generator) below.

### Goal

```xml
<goal id="0" x="0.0" y="0.0"/>
```

---

## Environment Class

```python
from realm_tools.simulation_lib.environment import Environment

env = Environment('simulation/worlds/environments/samples/octagon.xml')

# Typed wall lists
env.boundary_walls      # list[Wall]
env.obstacles           # list[Wall]

# Landmarks
env.landmarks           # list[Landmark]

# Start positions
env.training_starting_location   # list[StartingPosition]  (x, y, theta)
env.testing_start_location       # list[StartingPosition]

# Random position helpers
pos = env.get_random_training_starting_position()   # StartingPosition
pos = env.get_random_testing_starting_position()

# Goal
gx, gy = env.get_goal_location()
```

`Environment` is normally instantiated through `HamBot.load_environment()` — direct
use is for offline tooling and analysis.

---

## Start Position Generator

Generates a uniform grid of valid robot start positions from a maze XML.
Each candidate point is accepted if it is:

1. Inside the outer boundary polygon (convex hull of boundary wall endpoints).
2. At least `wall_clearance` metres from every wall segment (boundary and obstacle).

**Key parameters**

| Parameter | Description | Default |
|---|---|---|
| `spacing` | Distance between adjacent grid points (m). Smaller = denser coverage. | `0.1` |
| `wall_clearance` | Minimum distance from any wall surface (m). | `0.2` |
| `theta` | Heading assigned to every point (radians). | `0.0` |

### Command line

```bash
# Basic — output written alongside the XML as octagon_positions.csv
python -m realm_tools.simulation_lib.start_position_generator \
    simulation/worlds/environments/samples/octagon.xml

# Custom spacing and clearance with explicit output path
python -m realm_tools.simulation_lib.start_position_generator \
    simulation/worlds/environments/samples/octagon.xml \
    --spacing 0.1 \
    --clearance 0.2 \
    --output simulation/worlds/environments/samples/positions/octagon_positions.csv

# Save a coverage plot to data/data_cache/octagon_grid.png
python -m realm_tools.simulation_lib.start_position_generator \
    simulation/worlds/environments/samples/octagon.xml \
    --spacing 0.1 --clearance 0.2 --plot
```

### Python API

```python
from realm_tools.simulation_lib.start_position_generator import generate_grid

df = generate_grid(
    xml_path='simulation/worlds/environments/samples/octagon.xml',
    spacing=0.1,
    wall_clearance=0.2,
    output='simulation/worlds/environments/samples/positions/octagon_positions.csv',
    plot=True,
)
# df → pd.DataFrame with columns: x, y, theta
```

### Loading the CSV in a controller

```python
import pandas as pd

points = pd.read_csv('simulation/worlds/environments/vpce/positions/LM8_positions.csv')
for _, row in points.iterrows():
    robot.teleport_robot(x=row.x, y=row.y, theta=row.theta)
```
