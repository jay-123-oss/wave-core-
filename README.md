# Wave-Core Autonomous Drone Autopilot Showcase & HIL 3D Twin

> **Deterministic, Zero-Collision 3D Volumetric Flight Engine & Real-Time HIL Twin**
> Standalone web platform and interactive 3D digital twin for the Wave-Core Sub-Millisecond Robotics Safety Arbiter.

---

## 🌟 Overview

This repository contains the standalone, fully self-contained website and interactive 3D simulation for **Wave-Core**:
- **Interactive 3D Digital Twin**: Built with Three.js for real-time volumetric radar, 48-beam spherical LiDAR mesh, and obstacle evasion playback.
- **Audited Telemetry & Reports**: Includes all 8 technical audit benchmarks, Monte Carlo stress testing reports (1,000 runs, 0 collisions), and PX4/Gazebo HIL closed-loop results.
- **Cinematic Previews & Whitepaper**: Complete downloadable technical documentation, mission test videos, and interactive system architecture diagrams.
- **Zero External Dependencies**: Self-contained with local vendor scripts, assets, CSS, and telemetry data.

---

## 🚀 Quick Start / Local Preview

Open `index.html` directly in any modern browser, or launch a local static server:

```bash
# Using Python 3
python3 -m http.server 8000

# Using Node.js / npx
npx serve .
```

Then visit:
- **Main Platform**: `http://localhost:8000/`
- **3D HIL Twin Experience**: `http://localhost:8000/hil_3d_experience.html`

---

## 📁 Repository Structure

```
.
├── index.html                 # Main Wave-Core platform showcase & interactive dashboard
├── hil_3d_experience.html     # Dedicated Fullscreen 3D HIL Telemetry Twin
├── assets/                    # Optimized image assets, HUD renderings, and demo previews
├── css/                       # Premium dark-mode design system & animations (styles.css)
├── data/                      # 8 Official Audit Reports (.txt) and mission telemetry (.csv)
├── downloads/                 # Whitepaper PDF, benchmark reports, and mission clips
├── js/                        # Core application logic, telemetry parsers, and 3D simulation
│   ├── app.js                 # UI interactions, report viewer, intake forms
│   ├── data.js                # System architecture metrics and telemetry constants
│   ├── hil_telemetry_data.js  # Embedded 6-DOF time-series flight trajectories
│   ├── simulator.js           # Three.js 3D volumetric radar and drone visualization
│   └── vendor/                # Embedded Three.js and OrbitControls libraries
└── videos/                    # HD mission flight simulations and demonstrations
```

---

## 🌐 Deploy to GitHub Pages / Vercel / Netlify

This repository is ready for immediate 1-click static deployment:
- **GitHub Pages**: Go to `Settings > Pages` -> Select `main` branch -> `/ (root)`.
- **Vercel / Netlify**: Simply link this repository with root directory `./`.

---

© 2026 Wave-Core Robotics Inc. All rights reserved.
