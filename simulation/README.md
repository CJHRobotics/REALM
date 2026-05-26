# Webots Simulation Overview

This document explains the core Webots concepts used in REALM. Each section covers a key component of the simulator, what it does, and links to the official Webots documentation for deeper reference.

For a full reference, see the [Webots Reference Manual](https://cyberbotics.com/doc/reference/index).

---

## What is Webots?

[Webots](https://cyberbotics.com/doc/guide/introduction-to-webots) is an open-source robot simulator developed by Cyberbotics. It provides a full physics engine, a 3D environment, and a rich library of sensor and actuator models. Robots are defined using PROTO files (a VRML-based format) and controlled via Python, C, C++, Java, or MATLAB.

REALM uses Webots R2025a with Python controllers.

---

## World Files

A Webots **world** (`.wbt`) is the scene file that defines everything in the simulation — the robot, environment geometry, lighting, physics settings, and which controller runs on each robot.

World files for REALM are located in `simulation/worlds/`. Open them directly in Webots to launch a simulation.

**Reference:** [Webots World Files](https://cyberbotics.com/doc/guide/tutorial-1-your-first-simulation-in-webots)

---

## PROTO Files

A **PROTO** defines a reusable robot or object node in Webots. The HamBot is defined in `simulation/protos/HamBot/HamBot.proto`. It specifies the robot's geometry, appearance, sensors, actuators, and physics in a hierarchical node structure.

**Reference:** [PROTO Nodes](https://cyberbotics.com/doc/guide/tutorial-7-your-first-proto)

---

## Robot

The `Robot` node is the base node for any controllable agent in Webots. It holds all devices (sensors, actuators) as children and links to a controller script that runs the robot's logic each timestep.

In REALM, `HamBot` is built on the `Robot` node (via `Supervisor`) and is defined in `HamBot.proto`.

**Reference:** [Robot Node](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot)

---

## Supervisor

A **Supervisor** is a special type of `Robot` that has elevated privileges — it can read and modify the simulation world at runtime (move nodes, reset physics, query any node's position, etc.). This is how REALM teleports the robot to starting positions and loads maze geometry dynamically.

```python
from controller import Supervisor
supervisor = Supervisor()

# Get a node and move it
node = supervisor.getSelf()
node.getField('translation').setSFVec3f([1.0, 0.0, 0.09])
```

**Reference:** [Supervisor Node](https://cyberbotics.com/doc/guide/tutorial-8-the-supervisor)

---

## Controller

A **controller** is the Python script that runs the robot's logic. Each controller runs in its own process and communicates with Webots via the controller API. Every controller follows the same basic structure — a setup section followed by a `while` loop that steps the simulation:

```python
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

while robot.step(timestep) != -1:
    # read sensors, compute, actuate
    pass
```

`step()` advances the simulation by one timestep and returns `-1` when the simulation ends or is reset.

Controllers are stored in `simulation/controllers/`. Each controller lives in its own subdirectory with a matching Python file and a `runtime.ini` that points to the correct Python interpreter.

**Reference:** [Controller Programming](https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers)

---

## Timestep

The **timestep** is the duration of one simulation step in milliseconds. It is set in the world file and retrieved in the controller via `robot.getBasicTimeStep()`. All sensor enables and `step()` calls use this value.

In REALM the timestep is accessed as `robot.timestep`.

**Reference:** [Robot.getBasicTimeStep()](https://cyberbotics.com/doc/guide/the-user-interface)

---

## Motors

The HamBot uses two **RotationalMotor** devices — one per wheel. Motors are retrieved by name and set to velocity control mode by passing `float('inf')` as the position target.

```python
motor = robot.getDevice('left motor')
motor.setPosition(float('inf'))   # velocity control mode
motor.setVelocity(5.0)            # rad/sec
```

| Property | Value |
|----------|-------|
| Max velocity | 20 rad/s |
| Max torque | 50 N·m |

**Reference:** [RotationalMotor](https://cyberbotics.com/doc/reference/motor)

---

## Encoders (Position Sensors)

Each motor has a **PositionSensor** (wheel encoder) that measures cumulative wheel rotation in radians. To compute distance traveled, multiply the encoder reading by the wheel radius:

```python
encoder = robot.getDevice('left wheel encoder')
encoder.enable(timestep)

distance = encoder.getValue() * wheel_radius  # meters
```

| Property | Value |
|----------|-------|
| Wheel radius | 0.045 m |
| Axel length | 0.205 m |

**Reference:** [PositionSensor](https://cyberbotics.com/doc/reference/positionsensor)

---

## IMU (Inertial Measurement Unit)

The HamBot uses a **InertialUnit** device that returns the robot's roll, pitch, and yaw in radians. Yaw is used to derive the robot's compass heading.

```python
imu = robot.getDevice('imu')
imu.enable(timestep)

roll, pitch, yaw = imu.getRollPitchYaw()
```

Heading in degrees (0–360, East = 0°):
```python
import math
bearing = math.degrees(yaw)
if bearing < 0:
    bearing += 360
```

**Hardware equivalent:** [Adafruit BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)

**Reference:** [InertialUnit](https://cyberbotics.com/doc/reference/inertialunit)

---

## Camera

The HamBot has a front-facing **Camera** with object recognition enabled. It captures `224×224` RGB images and can identify objects by their recognition color.

```python
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

image = camera.getImageArray()         # 224x224 RGB pixel array
objects = camera.getRecognitionObjects()  # list of recognized objects
```

| Property | Value |
|----------|-------|
| Resolution | 224 × 224 |
| Field of view | 45° (0.785 rad) |

**Reference:** [Camera](https://cyberbotics.com/doc/reference/camera)

---

## LiDAR

The HamBot uses a **Lidar** device modeled on the Slamtec RPLidar A2. It returns a 360° horizontal range scan as a list of 360 distance values in meters (index 0 = rear, index 180 = front, index 90 = left, index 270 = right).

```python
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

ranges = lidar.getRangeImage()   # list of 360 floats (meters)
front  = ranges[180]
left   = ranges[90]
rear   = ranges[0]
right  = ranges[270]
```

| Property | Value |
|----------|-------|
| Range | 0.05 – 12.0 m |
| Resolution | 360 points / 360° |
| Noise | 0.000833 m |

**Hardware equivalent:** [Slamtec RPLidar A2](https://learn.adafruit.com/slamtec-rplidar-on-pi)

**Reference:** [Lidar](https://cyberbotics.com/doc/reference/lidar)

---

## GPS

The HamBot includes a **GPS** device that returns the robot's absolute position in the simulation world frame.

```python
gps = robot.getDevice('gps')
gps.enable(timestep)

x, y, z = gps.getValues()
```

**Reference:** [GPS](https://cyberbotics.com/doc/reference/gps)

---

## Display

The HamBot has a **Display** device (`800×800`) used to render images inside the Webots GUI — REALM uses it to show maze maps and visualizations.

Images are loaded as raw pixel data and pasted to the display each timestep:

```python
from controller import Display

display = robot.getDevice('Robot Display')
ir = display.imageNew(data, Display.RGB, width, height)
display.imagePaste(ir, 0, 0, False)
display.imageDelete(ir)
```

**Reference:** [Display](https://cyberbotics.com/doc/reference/display)

---

## Physics & Bounding Objects

Every solid node in Webots that participates in physics needs a `boundingObject` (collision geometry) and a `Physics` node (mass/density). The HamBot body uses a `Box`, wheels use `Cylinder` shapes, and caster wheels are `Sphere` shapes.

**Reference:** [Physics](https://cyberbotics.com/doc/reference/physics)

---

## PyCharm Setup for Running Controllers with `<extern>`

When a Webots robot's controller is set to `<extern>`, Webots waits for an external process to connect — meaning you launch the controller manually from PyCharm rather than having Webots start it. This is the recommended workflow for development and debugging.

For full details see the [Cyberbotics PyCharm guide](https://www.cyberbotics.com/doc/guide/using-pycharm-with-webots?version=R2019b-rev1).

### 1. Set the Python Interpreter

Open **PyCharm → Settings → Project → Python Interpreter** and set it to:

```
~/REALM/realm_core_venv/bin/python3
```

### 2. Add the Webots Controller Library as a Content Root

PyCharm needs to know where the Webots `controller` Python package lives so it can resolve imports like `from controller import Supervisor`.

1. Go to **PyCharm → Settings → Project → Project Structure**
2. Click **Add Content Root**
3. Add the following path:
```
/Applications/Webots.app/Contents/lib/controller/python
```

This allows PyCharm to index the Webots API and provide autocomplete for all Webots classes.

### 3. Set Environment Variables

Each run configuration needs the following environment variables so the Webots controller library can find `libController.dylib` at runtime.

Go to **Run → Edit Configurations → Environment Variables** and add:

```
WEBOTS_HOME=/Applications/Webots.app
DYLD_LIBRARY_PATH=/Applications/Webots.app/Contents/lib/controller
PYTHONUNBUFFERED=1
```

In PyCharm's environment variable text field these are semicolon-separated:

```
WEBOTS_HOME=/Applications/Webots.app;DYLD_LIBRARY_PATH=/Applications/Webots.app/Contents/lib/controller;PYTHONUNBUFFERED=1
```

### 4. Set the Working Directory

Each run configuration's **Working Directory** must be set to that controller's own directory, for example:

```
~/REALM/simulation/controllers/Calibration
```

Each controller calls `os.chdir("../../..")` at the top to navigate back to the repo root for imports — this only works correctly if the working directory starts at the controller's folder.

### 5. Run the Controller

1. Open the world file in Webots and ensure the HamBot controller is set to `<extern>`
2. Start the simulation in Webots — it will pause waiting for the external controller
3. Run your controller script from PyCharm
4. Webots will connect and the simulation will begin

---

## Useful Webots Links

| Resource                        | Link |
|---------------------------------|------|
| Webots User Guide & Reference Manual            | [cyberbotics.com/doc/guide](https://cyberbotics.com/doc/guide/index) |
| Controller Programming (Python) | [Controller Guide](https://cyberbotics.com/doc/guide/controller-programming?tab-language=python) |
| PyCharm + Webots Setup          | [Cyberbotics PyCharm Guide](https://www.cyberbotics.com/doc/guide/using-pycharm-with-webots?version=R2019b-rev1) |
| PROTO Reference                 | [PROTO Nodes](https://cyberbotics.com/doc/reference/proto) |
| Webots GitHub                   | [github.com/cyberbotics/webots](https://github.com/cyberbotics/webots) |
