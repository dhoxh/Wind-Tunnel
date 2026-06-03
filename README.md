# RustFlow — Interactive Wind Tunnel Visualization

RustFlow is a native desktop airflow visualization tool written in Rust.  
It allows users to import 3D models (GLB / glTF) and observe real-time, physically-inspired airflow behavior around arbitrary geometry.

This project focuses on interactive simulation and visual intuition rather than engineering-grade CFD accuracy.

> See **[WINDFLOW.md](WINDFLOW.md)** for a full description of the Navier–Stokes
> wind-flow simulation, how it works, and the keyboard controls.

## Quick start

```bash
cargo run --release
```

Use the **Wind Tunnel** settings panel (top-left) to import a `.glb`/`.gltf`
model (e.g. from Sketchfab), set wind speed, ride height, heading, streamline
density, and the FPS cap. The full control list is in [WINDFLOW.md](WINDFLOW.md).

On Linux, building Bevy needs `libasound2-dev`, `libudev-dev`, and
`libxkbcommon-dev`; macOS and Windows need nothing extra.

## Features

- Drag-and-drop 3D model support (GLB / glTF)
- Real-time incompressible flow simulation
- Obstacle-aware airflow wrapping and wake formation
- Streamline and dye visualization
- Adjustable wind speed, viscosity, and swirl parameters
- GPU-accelerated simulation via wgpu (Metal on macOS)

## Simulation Approach

RustFlow uses a grid-based Navier–Stokes approximation with:

- Semi-Lagrangian advection
- Pressure projection for incompressibility
- Obstacle boundary handling via voxel or slice masks
- Optional vorticity confinement for enhanced visual turbulence

The solver prioritizes numerical stability and responsiveness to enable real-time interaction.

## Platform

- Native desktop application
- Developed and tested on macOS (Apple Silicon compatible)

## Purpose

This project is designed as an interactive airflow exploration tool for educational, experimental, and visualization purposes. It demonstrates how modern Rust and GPU compute pipelines can be used to simulate fluid-like behavior around arbitrary 3D geometry.

## Future Improvements

- Full 3D voxel simulation mode
- Higher-resolution grids
- Additional visualization modes (pressure, vorticity heatmaps)
- Performance optimizations and benchmarking tools

