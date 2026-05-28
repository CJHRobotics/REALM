# REALM

**REALM** (Robotic Environment for Autonomous Learning and Mapping) is a template repository for building Webots-based robot simulation experiments centered around reinforcement learning research. The goal is to provide a clean, minimal starting point that can be forked and extended for new papers and experiments without having to rebuild the simulation infrastructure from scratch.

The repo provides:
- A pre-configured HamBot robot with sensors (camera, LiDAR, IMU, GPS, encoders) ready to use in Webots
- A Gymnasium-compatible environment skeleton for RL training with Stable-Baselines3 or PyTorch
- Environment/environment loading and management tools
- Two isolated Python environments — one for simulation/training, one for data analysis
- A calibration controller for manually driving the robot with keyboard controls

---

## Requirements

### 1. Python 3.11

REALM requires Python 3.11 specifically.

**macOS:**
```shell
brew install python@3.11
```

**Linux:**
```shell
sudo apt-get install python3.11
```

**Windows:**
Install Python 3.11 from the [Microsoft Store](https://apps.microsoft.com/store/detail/python-311/9NRWMJP3717K) to avoid PATH issues.

### 2. Webots R2025a

Download and install from the [Cyberbotics website](https://cyberbotics.com/#download).

> **Linux users:** Do not install Webots via Snap. Use the `.deb` package or tarball instead.

### 3. Git

**Windows:** [git-scm.com](https://git-scm.com/download/win)

**Linux:** `sudo apt-get install git`

**macOS:** `brew install git`

---

## Setup

### 1. Clone the repository

```shell
git clone <your-repo-url>
cd REALM
```

### 2. Run the install script

```shell
# Core environment only (Webots + RL training)
python setup/realm_install.py

# Core + analysis environment (adds Jupyter, statsmodels, seaborn, etc.)
python setup/realm_install.py --data_analysis
```

This will:
- Find Python 3.11 on your system
- Create `realm_core_venv` (and optionally `realm_analysis_venv`)
- Install all dependencies
- Add the project root to the venv's Python path
- Generate `runtime.ini` files in all Webots controller directories

To remove the environments:
```shell
python setup/realm_install.py --uninstall
python setup/realm_install.py --uninstall --data_analysis
```

### 3. Activate the environment

**macOS/Linux:**
```shell
source realm_core_venv/bin/activate
```

**Windows:**
```shell
realm_core_venv\Scripts\activate
```

For analysis work:
```shell
source realm_analysis_venv/bin/activate   # macOS/Linux
realm_analysis_venv\Scripts\activate      # Windows
```

> If you add a new controller under `simulation/controllers/`, re-run `python setup/add_runtime_ini.py` to generate its `runtime.ini`.

---

## Project Structure

```
REALM/
├── setup/
│   ├── realm_install.py          # Install / uninstall script
│   ├── add_runtime_ini.py        # Generates Webots runtime.ini files
│   ├── requirements_webots.txt   # Core venv dependencies
│   └── requirements_analysis.txt # Analysis venv dependencies
│
├── realm_tools/
│   ├── robot_lib/
│   │   ├── hambot.py             # Base robot class (sensors, motors, supervisor)
│   │   ├── my_robot.py           # User extension template (inherits HamBot)
│   │   └── robot_tools.py        # Shared robot utility functions
│   ├── simulation_lib/
│   │   ├── environment.py        # Environment class and environment objects
│   │   ├── maze_parser.py        # XML maze file parser
│   │   └── webots_torch_environment.py  # Gymnasium environment skeleton
│   └── image_lib/
│       ├── feature_extractor.py  # CNN feature extraction
│       └── image_feature_lib.py  # Image processing utilities
│
├── simulation/
│   ├── controllers/
│   │   ├── example/              # Example Webots controller
│   │   └── calibration/          # Keyboard-driven calibration controller
│   ├── protos/                   # HamBot and world object Webots protos
│   └── worlds/                   # Webots world files and maze XMLs
│
├── data/
│   └── DataCache/                # Temp files used by the display system
│
└── docs/                         # Figures and documentation assets
```

---

## Extending for Your Project

The intended workflow when forking this repo for a new paper:

1. **Robot logic** — subclass `HamBot` in `my_robot.py` and add your experiment-specific methods (action sets, observation processing, etc.)
2. **Environment** — fill in the `WebotsEnv` skeleton in `webots_torch_environment.py` with your observation space, reward function, and episode logic
3. **Controllers** — add new Webots controllers under `simulation/controllers/` then re-run `add_runtime_ini.py`
4. **Personal files** — your personal robot subclass (e.g. `yourname_robot.py`) can be gitignored so the template stays clean for others

---

## Calibration Controller

A keyboard-driven controller is provided for manually testing robot behaviour in Webots:

| Key | Action |
|---|---|
| Arrow Up | Forward |
| Arrow Down | Backward |
| Arrow Left | Turn left |
| Arrow Right | Turn right |
| Any other key | Stop |

Open `simulation/worlds/calibration.wbt` in Webots to use it.

---

## Additional Documentation

- [HamBot Reference](realm_tools/README.md)
- [Simulation & Webots Guide](simulation/README.md)
- [Controller Setup Guide](simulation/controllers/README.md)
- [Webots Controller Guide](https://cyberbotics.com/doc/guide/controller-programming)
- [Gymnasium API Reference](https://gymnasium.farama.org/api/env/)
- [Stable-Baselines3 Docs](https://stable-baselines3.readthedocs.io/)
