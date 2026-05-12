# VexSim 2.0

VexSim 2.0 is a Python simulator for VEX-style robot driving, match loading, scoring, parking, and field interaction. It uses PyBullet for physics, Pygame for the app loop/UI, and an OpenGL renderer when available.

## Features

- 3D robot and field simulation with PyBullet physics.
- GPU-rendered match view with transparent loaders and 3D triballs.
- Configurable robot size, mass, drivetrain, intake, and controller bindings.
- Match scoring, leaderboard, team profiles, loaders, goals, and parking zones.
- Fallback renderer for systems without OpenGL support.

## Setup

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python main.py
```

On macOS/Linux:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python main.py
```

## Contributing

Anyone is welcome to fork the repository and open pull requests. Changes should go through review and must pass CI before they are merged into `main`.

See [CONTRIBUTING.md](CONTRIBUTING.md) for the workflow and quality expectations.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
