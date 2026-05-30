# Environments

This directory contains XML environment files that define the simulation world — walls, landmarks, starting positions, and goals. The environment is loaded at runtime by the `Environment` class in `realm_tools/simulation_lib/environment.py`.

See `samples/` for working examples.

---

## XML Schema

All environment files must have an `<environment>` root tag.

```xml
<?xml version="1.0" encoding="utf-8"?>
<environment>
    <!-- walls, landmarks, positions, goals -->
</environment>
```

---

## Walls

Walls are defined by two endpoints. The parser computes the center, length, and rotation automatically.

```xml
<wall x1="-2.0" y1="1.0" x2="2.0" y2="1.0"
      type="boundary"
      height="0.5"
      width="0.012"
      texture="../protos/world_objects/textures/wall_textures/white_marble.png"/>
```

| Attribute | Required | Default | Description |
|-----------|----------|---------|-------------|
| `x1`, `y1` | yes | — | First endpoint (meters) |
| `x2`, `y2` | yes | — | Second endpoint (meters) |
| `type` | yes | — | `boundary` or `obstacle` |
| `height` | no | `0.3` | Wall height in meters |
| `width` | no | `0.012` | Wall thickness in meters |
| `texture` | no | proto default | Path to texture image, relative to the world file |

**`boundary`** walls define the outer enclosure. **`obstacle`** walls are internal.

If `texture` is omitted the `wall.proto` default (black marble) is used.

---

## Landmarks

All landmarks share a unified `<landmark>` tag. The `type` attribute controls which Webots proto is spawned.

### Cylinder

A colored vertical cylinder — uses the custom `landmark.proto`.

```xml
<landmark type="cylinder"
          x="0.0" y="1.0"
          height="1.5" radius="0.25"
          red="1.0" green="0.0" blue="0.0"/>
```

| Attribute | Required | Default | Description |
|-----------|----------|---------|-------------|
| `x`, `y` | yes | — | Center position (meters) |
| `height` | no | `1.0` | Cylinder height in meters |
| `radius` | no | `0.25` | Cylinder radius in meters |
| `red`, `green`, `blue` | no | `1.0` each | RGB color (0–1) |

### Panel

A flat rectangular panel — uses the Webots `RectangularPanel` proto. Supports any image texture.

```xml
<landmark type="panel"
          x="0.0" y="1.0" theta="-1.571"
          height="1.0" width="1.0"
          texture="../protos/world_objects/textures/april_tags/april_tag_0.png"
          red="1.0" green="0.0" blue="0.0"/>
```

| Attribute | Required | Default | Description |
|-----------|----------|---------|-------------|
| `x`, `y` | yes | — | Center position (meters) |
| `theta` | no | `0.0` | Rotation angle in radians |
| `height` | no | `1.0` | Panel height in meters |
| `width` | no | `1.0` | Panel width in meters |
| `texture` | yes | — | Path to image, relative to the world file |
| `red`, `green`, `blue` | no | `1.0` each | RGB tint (0–1) |

Texture path is passed directly to Webots — the parser does not validate it. Any image file in `protos/world_objects/textures/` can be used; add new images to the appropriate subfolder and reference them by path.

---

## Start Positions

Define one or more robot start poses for training and testing separately. Theta is in radians.

```xml
<train_start_positions>
    <pos x="-1.5" y="0.0" theta="0.0"/>
    <pos x=" 1.5" y="0.0" theta="3.141"/>
</train_start_positions>

<test_start_positions>
    <pos x="0.0" y="-1.5" theta="1.571"/>
</test_start_positions>
```

---

## Goal

```xml
<goal id="0" x="0.0" y="0.0"/>
```

| Attribute | Required | Default | Description |
|-----------|----------|---------|-------------|
| `id` | no | `0` | Goal identifier |
| `x`, `y` | yes | — | Goal position (meters) |

---

## Minimal Example

```xml
<?xml version="1.0" encoding="utf-8"?>
<environment>

    <wall x1="-2.0" y1=" 2.0" x2=" 2.0" y2=" 2.0" type="boundary"/>
    <wall x1=" 2.0" y1=" 2.0" x2=" 2.0" y2="-2.0" type="boundary"/>
    <wall x1=" 2.0" y1="-2.0" x2="-2.0" y2="-2.0" type="boundary"/>
    <wall x1="-2.0" y1="-2.0" x2="-2.0" y2=" 2.0" type="boundary"/>

    <landmark type="panel" x="0.0" y="2.0" theta="-1.571" height="1.0" width="1.0"
              texture="../protos/world_objects/textures/april_tags/april_tag_0.png"/>

    <train_start_positions>
        <pos x="-1.0" y="0.0" theta="0.0"/>
    </train_start_positions>

    <test_start_positions>
        <pos x="-1.0" y="0.0" theta="0.0"/>
    </test_start_positions>

    <goal id="0" x="1.0" y="0.0"/>

</environment>
```
