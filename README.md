# 🏎️ Autonomous Lap Optimizer

> A reinforcement learning environment and autonomous driving framework built on [TORCS](http://torcs.sourceforge.net/) — designed to train, benchmark, and evolve self-driving agents that optimize lap times through iterative learning.

[![Python](https://img.shields.io/badge/Python-3.11%2B-blue?logo=python)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2014.04%20%7C%2016.04-orange?logo=ubuntu)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Simulator](https://img.shields.io/badge/Simulator-TORCS-red)](http://torcs.sourceforge.net/)

---

## 📖 Overview

**Autonomous Lap Optimizer** extends the [Gym-TORCS](https://github.com/ugo-nama-kun/gym_torcs) OpenAI-Gym-like interface with a full pipeline for autonomous racing research:

- **Modular driving clients** with configurable control parameters (steering gain, throttle, gear shifting, traction control)
- **Versioned agent progression** — from a reactive baseline (`v1`) to corner-aware, track-memory-assisted agents (`v2`, `v2_track_memory`)
- **Race telemetry logging** with per-step CSV export and lap detection
- **Track profiling** that builds a geometric speed map from verified clean laps
- **Multi-car parallel racing** support via a standalone socket client
- **Reinforcement learning harness** with OpenAI Gym-style `reset / step / end` interface

---

## 🗂️ Repository Structure

```
Autonomous-Lap-Optimizer/
│
├── gym_torcs.py                  # Core Gym-TORCS environment (reset/step/end)
├── example_experiment.py         # RL training loop example
├── sample_agent.py               # Random reference agent (steering baseline)
│
├── torcs_jm_par.py               # Multi-car parallel racing client (main runner)
│
├── snakeoil3_gym.py              # Original snakeoil TORCS client (preserved)
├── snakeoil3_v1.py               # V1: Windows-safe client + CSV telemetry logging
├── snakeoil3_v2.py               # V2: Corner-aware speed + prospective braking
├── snakeoil3_v2_track_memory.py  # V2+: Feed-forward track memory layer
├── snakeoil3_jm2.py              # Alternate snakeoil variant
├── jmcncarai.py                  # Additional agent variant
│
├── build_track_profile.py        # Builds speed profile from verified clean lap
├── race_logging.py               # Per-step telemetry logger with lap detection
├── create_results_csvs.py        # Initialises results/ CSV files with correct headers
├── test_race_logging.py          # Unit tests for race_logging module
│
├── autostart.sh                  # Auto-launches and configures TORCS GUI
├── practice.xml                  # TORCS practice race configuration
│
├── results/                      # Output directory: lap_data.csv, track_data.csv
├── vtorcs-RL-color/              # Modified TORCS build with RL + colour support
└── .gitignore
```

---

## 🧠 Agent Evolution Chain

| Version | File | Key Capability |
|---|---|---|
| **Baseline** | `snakeoil3_gym.py` | Original unmodified snakeoil client |
| **V1** | `snakeoil3_v1.py` | Windows-safe connection + CSV telemetry logging |
| **V2** | `snakeoil3_v2.py` | Symmetric corner speed reduction + prospective braking via `track_9` sensor |
| **V2+Memory** | `snakeoil3_v2_track_memory.py` | Feed-forward track profile look-ahead; falls back to V2 if profile unavailable |
| **Parallel** | `torcs_jm_par.py` | Multi-car socket client with full CLI parameter control |

---

## ⚙️ Requirements

| Dependency | Notes |
|---|---|
| Ubuntu 14.04 LTS / 16.04 LTS | TORCS runs on Linux; Windows can run `v1`/`v2` clients only |
| Python 3.11+ | Standard library only for `v1`/`v2`; `gym_torcs.py` requires OpenAI Gym + numpy |
| [xautomation](http://linux.die.net/man/7/xautomation) | For automated GUI interaction in `autostart.sh` |
| [OpenAI Gym](https://github.com/openai/gym) | Required for `gym_torcs.py` environment wrapper |
| numpy | Required for `gym_torcs.py` and `sample_agent.py` |
| vtorcs-RL-color | Modified TORCS binary — see `vtorcs-RL-color/` for installation |

---

## 🚀 Quick Start

### 1. Install vtorcs-RL-color

Follow the build instructions inside the `vtorcs-RL-color/` directory.

### 2. Initialise the race in TORCS GUI

```bash
sudo torcs
```

Navigate: **Race → Practice → Configure Race** to select a track, then **Race → Practice → New Race**.  
TORCS will hold a blue screen — this means the server is ready.

### 3. Initialise result files

```bash
python create_results_csvs.py
```

This creates `results/lap_data.csv` and `results/track_data.csv` with correct headers.

### 4a. Run the standalone driving client (recommended starting point)

```bash
# Reactive baseline
python snakeoil3_v1.py --port 3001 --track road1 --car unknown

# Corner-aware + prospective braking
python snakeoil3_v2.py --port 3001 --track road1 --car unknown

# With track memory (requires a profile — see below)
python snakeoil3_v2_track_memory.py --port 3001 --track road1 --car unknown
```

### 4b. Run the multi-car parallel client

```bash
python torcs_jm_par.py --host localhost --port 3001 --episodes 5 --steps 100000
```

All configurable options:

| Option | Default | Description |
|---|---|---|
| `--host` / `-H` | `localhost` | TORCS server host |
| `--port` / `-p` | `3001` | TORCS server port |
| `--episodes` / `-e` | `1` | Number of learning episodes |
| `--steps` / `-m` | `100000` | Max simulation steps per episode |
| `--track` / `-t` | `unknown` | Track label for logging |
| `--stage` / `-s` | `3` | Race stage: 0=warm-up, 1=qualifying, 2=race, 3=unknown |
| `--parameters` / `-f` | — | JSON blob: `target_speed`, `steer_gain`, `centering_gain`, `brake_threshold`, `gear_speeds[6]`, `enable_traction_control` |
| `--debug` / `-d` | off | Print full telemetry each step |

### 4c. Run the RL experiment

```bash
python example_experiment.py
```

---

## 🔬 RL Environment API

```python
from gym_torcs import TorcsEnv

# Steering only (1D continuous action), with vision input
env = TorcsEnv(vision=True, throttle=False)

# Steering + throttle (2D continuous action), no vision
env = TorcsEnv(vision=False, throttle=True)

# Reset — use relaunch=True periodically to avoid TORCS memory leak
ob = env.reset(relaunch=True)

from sample_agent import Agent
agent = Agent(1)  # 1 = steering only
action = agent.act(ob, reward=0, done=False, vision_on=False)

ob, reward, done, _ = env.step(action)

env.end()  # shuts down TORCS
```

### Observation Space

| Sensor | Shape | Range | Description |
|---|---|---|---|
| `focus` | (5,) | [0, 1] | Focus sensors |
| `speedX` | scalar | raw | Longitudinal speed (km/h) |
| `speedY` | scalar | raw | Lateral speed (km/h) |
| `speedZ` | scalar | raw | Vertical speed (km/h) |
| `opponents` | (36,) | [0, 1] | Distance to nearby opponents |
| `rpm` | scalar | raw | Engine RPM |
| `track` | (19,) | [0, 1] | Track edge distances at 19 angles |
| `wheelSpinVel` | (4,) | raw | Wheel spin velocity |
| `vision` *(optional)* | (4096, 3) | [0, 255] | 64×64 RGB image |

> Track sensors are normalised to [0, 1]. Out-of-road readings return −1/200.

---

## 📊 Race Logging & Results

All telemetry is written to `results/` automatically during a run:

| File | Content |
|---|---|
| `results/lap_data.csv` | One row per lap: `run_id`, `lap_time_seconds`, `damage_delta`, `status`, etc. |
| `results/track_data.csv` | One row per ~5 Hz sample: position, speed, sensors, lap context |

Lap detection triggers when `curLapTime` resets and the car has advanced at least the minimum lap distance — robust against restarts, reconnects, and reverse crossings.

---

## 🗺️ Track Profiling

Once you have at least one verified clean lap (zero damage, `status=completed`) in `results/`:

```bash
python build_track_profile.py
```

This produces `results/track_profile.csv` — a 50-metre-binned conservative speed map of the track.  
`snakeoil3_v2_track_memory.py` loads this profile at startup and uses it as a feed-forward speed hint, silently falling back to V2 reactive behaviour if the profile is missing or invalid.

---

## 🔧 Configuration: Control Parameters

Edit the top of `torcs_jm_par.py` to change driving behaviour:

```python
TARGET_SPEED         = 100   # km/h — cruise speed on straights
STEER_GAIN           = 30    # steering aggressiveness
CENTERING_GAIN       = 0.20  # track-centre correction strength
BRAKE_THRESHOLD      = 0.9   # angle at which braking activates (lower = earlier)
GEAR_SPEEDS          = [0, 20, 40, 80, 100, 180]  # upshift thresholds (km/h)
ENABLE_TRACTION_CONTROL = True
```

Or pass them at runtime:

```bash
python torcs_jm_par.py --parameters '{"target_speed":120,"steer_gain":25,"enable_traction_control":false}'
```

---

## 🔊 Sensor Noise (Optional)

To inject noise into low-dimensional sensors, edit lines 33–35 of `gym_torcs.py`:

```python
os.system('torcs -nofuel -nodamage -nolaptime -vision -noisy &')
os.system('torcs -nofuel -nolaptime -noisy &')
```

---

## 🧪 Running Tests

```bash
python -m pytest test_race_logging.py -v
```

---

## 📚 References & Acknowledgements

| Resource | Link |
|---|---|
| Original Gym-TORCS | [ugo-nama-kun/gym_torcs](https://github.com/ugo-nama-kun/gym_torcs) |
| vtorcs (modified TORCS) | [giuse/vtorcs](https://github.com/giuse/vtorcs) |
| Python Snakeoil Client | [xed.ch/project/snakeoil](http://xed.ch/project/snakeoil/index.html) |
| TORCS AI Research Paper | [Loiacono et al., arXiv:1304.1672](https://arxiv.org/pdf/1304.1672.pdf) |
| DDPG + Keras on TORCS | [Ben Lau's Blog](https://yanpanlau.github.io/2016/10/11/Torcs-Keras.html) |

> Original Gym-TORCS developed during the Spring 2016 internship at Preferred Networks.  
> This project is licensed under the [MIT License](LICENSE).
