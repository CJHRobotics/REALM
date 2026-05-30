# Protos

This directory contains all Webots PROTO files and their associated assets.

```
protos/
  robots/
    hambot/
      hambot.proto
      meshes/           ← .obj mesh files
  world_objects/
    objects/            ← world object PROTO files
      wall.proto
      landmark.proto
    textures/
      april_tags/       ← april tag panel images
      flags/            ← flag panel images
      wall_textures/    ← wall surface textures
```

---

## Naming Convention

All files and directories use lowercase with underscores — no capitals, no spaces.

```
good:  black_marble.jpg
good:  landmark_tag_0.png
good:  my_new_wall_texture.jpg
bad:   BlackMarble.jpg
bad:   Landmark Tag 0.png
```

---

## Adding a New Texture

Textures are image files used by panel landmarks or walls. The parser treats texture paths as plain strings — no code changes are needed to add new textures.

**1.** Add the image file to the appropriate subfolder:

| Content | Folder |
|---------|--------|
| AprilTag images | `world_objects/textures/april_tags/` |
| Flag images | `world_objects/textures/flags/` |
| Wall surface images | `world_objects/textures/wall_textures/` |
| New category | Create a new subfolder under `world_objects/textures/` |

**2.** Reference it in the environment XML using a path relative to the world file (`.wbt`):

```xml
<landmark type="panel" ... texture="../protos/world_objects/textures/april_tags/my_tag.png"/>
<wall ... texture="../protos/world_objects/textures/wall_textures/my_texture.jpg"/>
```

That's it — no parser or Python changes required.

---

## Adding a New World Object PROTO

**1.** Create the `.proto` file in `world_objects/objects/` following the naming convention. Use an existing proto as a reference — `wall.proto` for solid objects, `landmark.proto` for recognized objects.

**2.** Declare it in every `.wbt` world file that will use it. Open the world file in a text editor and add an `IMPORTABLE EXTERNPROTO` line:

```
IMPORTABLE EXTERNPROTO "../protos/world_objects/objects/my_object.proto"
```

**3.** Add a new landmark `type` to the parser and environment classes:

In `realm_tools/simulation_lib/environment_parser.py`, add the new type's attributes to `parse_landmark`:

```python
elif landmark_type == 'my_type':
    data['my_attribute'] = float(xml_landmark.get('my_attribute', default_value))
```

In `realm_tools/simulation_lib/environment.py`, add a node string method to the `Landmark` class and dispatch it from `get_webots_node_string`:

```python
def get_webots_node_string(self):
    if self.landmark_type == 'cylinder': return self._cylinder_node_string()
    elif self.landmark_type == 'panel':  return self._panel_node_string()
    elif self.landmark_type == 'my_type': return self._my_type_node_string()

def _my_type_node_string(self):
    return (
        f'DEF my_type_{self.id} MyProtoName {{ '
        f'translation {self.translation[0]:.2f} {self.translation[1]:.2f} {self.translation[2]:.2f} '
        f'... }}'
    )
```

**4.** Use it in an environment XML:

```xml
<landmark type="my_type" x="0.0" y="0.0" my_attribute="1.0"/>
```

---

## Adding a New Robot PROTO

**1.** Create a new directory under `robots/` for the robot:

```
robots/
  hambot/
  my_robot/
    my_robot.proto
    meshes/
```

**2.** Declare it in the world file:

```
EXTERNPROTO "../protos/robots/my_robot/my_robot.proto"
```

**3.** Follow the same three-tier inheritance pattern — create a base class in `realm_tools/robot_lib/` that wraps the Webots Supervisor API.
