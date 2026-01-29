
# Hexapod Control and Simulation Project

This repository provides Python scripts to control and simulate a hexapod robot using Dynamixel servos. The project includes GUI sliders for real-time control and PyBullet for simulation visualization.

---

## Project Structure

```
hexapod_project/
├── hexapod_only.py            # Control hexapod with Dynamixel servo using Tkinter sliders
├── hexapod_simulation.py      # Real-time hexapod control with PyBullet simulation and GUI sliders
├── simulation.py              # Hexapod simulation in PyBullet with GUI sliders
├── lib/
│   ├── hexapod_constant.py    # Defines hexapod constants like body size and leg dimensions
│   └── servo.py               # Functions to control Dynamixel servos
└── models/
    └── hexapod.urdf           # Hexapod model for PyBullet
```

---

## Features

- **Hexapod Control:** Control the hexapod using sliders for real-time adjustments.
- **Dynamixel Servo Integration:** Dynamically set servo positions for precise movement.
- **Simulation:** Visualize hexapod movement in the PyBullet simulation environment.
- **Flexible Components:** Modular design with configuration constants in `lib/hexapod_constant.py`.

---

## Setup Instructions

### Prerequisites

- Python 3.x
- [PyBullet](https://pypi.org/project/pybullet/) for physics simulation
- Dynamixel SDK (for servo control)

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/hexapod_project.git
   cd hexapod_project
   ```

2. Install the required Python packages:

   ```bash
   pip install pybullet numpy
   ```

---

## Usage

### Hexapod Simulation Only

```bash
python simulation.py
```

Simulate the hexapod in PyBullet without controlling actual servos.

### Hexapod Control with Simulation

```bash
python hexapod_simulation.py
```

Control the hexapod using GUI sliders with real-time PyBullet simulation.

### Hexapod Control with Servo Only

```bash
python hexapod_only.py
```

Use Tkinter sliders to control the hexapod with Dynamixel servos.

---

## Directory Details

- **lib/hexapod_constant.py:** Contains hexapod dimensions like body size and leg lengths.
- **lib/servo.py:** Provides functions to write servo positions to Dynamixel servos.
- **models/hexapod.urdf:** Defines the 3D hexapod model for simulation in PyBullet.

---

# Hexapod Demo

[![Watch the Demo on YouTube](https://img.youtube.com/vi/8T388ddCNac/hqdefault.jpg)](https://www.youtube.com/watch?v=8T388ddCNac)

Click the image above to watch the hexapod robot demo on YouTube.

## Contributing

Contributions are welcome! Please fork this repository and submit a pull request.

---

## License

This project is licensed under the [MIT License](LICENSE).
