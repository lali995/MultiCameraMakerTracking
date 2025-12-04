# Multi-Camera Maker Tracking

A visualization tool for multi-camera tracking systems. Displays camera positions and viewing directions in 3D space based on sensor configuration files.

## Features

- Reads camera extrinsic parameters from `sensor_config.json` files
- Interactive 3D visualization with Plotly (zoom, pan, rotate, hover inspection)
- Matplotlib visualization option for basic viewing
- Displays camera folder names on hover

## Requirements

- Python 3.x
- numpy
- matplotlib
- plotly

## Installation

```bash
pip install numpy matplotlib plotly
```

## Usage

### Interactive visualization (default)

```bash
python main.py
```

Opens an interactive 3D view in your browser with:
- Mouse drag to rotate
- Scroll to zoom
- Hover over cameras to see coordinates and folder name
- Click and drag to pan

### Matplotlib visualization

```bash
python main.py --matplotlib
```

## Configuration

Place camera configuration folders in the `config/` directory. Each folder should be named `SZVIDS-*` and contain a `sensor_config.json` file with a `Sensor to World Extrinsic` 4x4 transformation matrix.

Example structure:
```
config/
├── SZVIDS-250515-DB77B5-8F194B-9AF224/
│   └── sensor_config.json
├── SZVIDS-250513-21892C-CAEB69-EC3063/
│   └── sensor_config.json
└── ...
```

## Coordinate System

- X-axis: Left to right
- Y-axis: Up
- Z-axis: Front to back
