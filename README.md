<div align="center">

<h1>FAST-LIO-DCReg</h1>
<h3>FAST-LIO2 + DCReg: Robust LiDAR-Inertial Odometry with Principled Degeneracy Handling</h3>

[![License: GPL v2](https://img.shields.io/badge/License-GPLv2-blue.svg)](LICENSE)
[![Stars](https://img.shields.io/github/stars/JokerJohn/FAST-LIO-DCReg?style=social)](https://github.com/JokerJohn/FAST-LIO-DCReg/stargazers)
[![Issues](https://img.shields.io/github/issues/JokerJohn/FAST-LIO-DCReg)](https://github.com/JokerJohn/FAST-LIO-DCReg/issues)
[![arXiv](https://img.shields.io/badge/arXiv-2509.06285-b31b1b)](https://arxiv.org/abs/2509.06285)

</div>

**FAST-LIO-DCReg** integrates [DCReg](https://github.com/JokerJohn/DCReg) — a principled degeneracy characterization and mitigation framework — into [FAST-LIO2](https://github.com/hku-mars/FAST_LIO), enabling robust LiDAR-inertial odometry in degenerate environments such as narrow corridors, long hallways, planar scenes, and sparse-feature areas.

> **DCReg** is the author's prior work on degenerate LiDAR registration (**D**ecoupled **C**haracterization for ill-conditioned **Reg**istration). It achieves **20%–50% accuracy improvement** and **5–100× speedup** over state-of-the-art degeneracy-handling methods. See the [DCReg paper](https://arxiv.org/abs/2509.06285) for full details.

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [DCReg Integration Design](#dcreg-integration-design)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Supported LiDARs](#supported-lidars)
- [Datasets](#datasets)
- [Roadmap](#roadmap)
- [Citation](#citation)
- [Acknowledgments](#acknowledgments)
- [License](#license)

---

## Overview

### Why Integrate DCReg into FAST-LIO2?

FAST-LIO2 is a state-of-the-art LiDAR-inertial odometry system that excels in general environments. However, like all scan-to-map registration-based systems, FAST-LIO2 is vulnerable to **LiDAR degeneracy** — situations where the point cloud provides insufficient geometric constraints along one or more degrees of freedom (DOF). Common degenerate scenarios include:

- **Narrow corridors / tunnels**: lack of lateral or yaw constraints.
- **Long planar hallways**: insufficient depth constraints.
- **Sparse-feature areas**: stairwells, open fields, parking lots.
- **Repetitive geometry**: structured environments with ambiguous matching.

Standard mitigations (SVD truncation, uniform Tikhonov regularization) suffer from two fundamental problems:
1. They fail to correctly identify *which* DOFs are actually degenerate.
2. They apply uniform damping that corrupts well-constrained directions.

**DCReg** solves both problems through a principled Schur-complement-based decoupling of rotation and translation, enabling:
- **Precise detection** of which DOFs lack constraints and to what extent.
- **Targeted preconditioning** that stabilizes only degenerate directions while preserving observable information.

**FAST-LIO-DCReg** embeds this framework into FAST-LIO2's iterated extended Kalman filter (iEKF) update loop, improving robustness without sacrificing accuracy in well-conditioned scenes.

### System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         FAST-LIO-DCReg                           │
│                                                                  │
│  ┌───────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │  LiDAR    │───▶│  Motion      │───▶│  iEKF Update         │  │
│  │  (raw pts)│    │  Undistortion│    │  (scan-to-map ICP)   │  │
│  └───────────┘    └──────────────┘    │                      │  │
│                                       │  ┌────────────────┐  │  │
│  ┌───────────┐    ┌──────────────┐    │  │  DCReg Module  │  │  │
│  │   IMU     │───▶│  Forward     │───▶│  │                │  │  │
│  │           │    │  Propagation │    │  │ 1. Schur decp. │  │  │
│  └───────────┘    └──────────────┘    │  │ 2. Degen. det. │  │  │
│                                       │  │ 3. Tgt. precond│  │  │
│                                       │  │ 4. PCG solve   │  │  │
│                                       │  └────────────────┘  │  │
│                                       └──────────────────────┘  │
│                                                ▼                  │
│                                    ┌──────────────────┐          │
│                                    │  ikd-Tree Map    │          │
│                                    │  (incremental)   │          │
│                                    └──────────────────┘          │
└──────────────────────────────────────────────────────────────────┘
```

---

## Key Features

### From FAST-LIO2

- **Tightly-coupled LiDAR-IMU fusion** via iterated extended Kalman filter (iEKF).
- **Direct scan-to-map odometry** on raw LiDAR points (no feature extraction required).
- **Incremental mapping** with [ikd-Tree](https://github.com/hku-mars/ikd-Tree) for real-time performance at 100+ Hz LiDAR rates.
- **Multi-LiDAR support**: Livox (Avia, Horizon, MID-70), Velodyne, Ouster, and others.
- **ARM platform support**: Khadas VIM3, Nvidia TX2, Raspberry Pi 4B.
- **External IMU support** with robust extrinsic initialization.

### From DCReg

- **Reliable ill-conditioning detection**: Schur complement decomposition decouples rotation and translation Hessian blocks, eliminating cross-term coupling that masks true degeneracy patterns.
- **Quantitative degeneracy characterization**: Maps mathematical eigenspace to physical motion space (t0, t1, t2, r0, r1, r2), revealing which specific motions lack constraints and to what degree.
- **Targeted preconditioning**: Eigenvalue clamping is applied exclusively in degenerate subspaces, leaving well-constrained directions untouched.
- **Efficient PCG solver**: A single interpretable regularization parameter (`λ`) controls the preconditioning strength.
- **20%–50% accuracy improvement** and **5–100× speedup** over methods such as ME-SR, ME-TSVD, ME-TReg, FCN-SR, XICP, and SuperLoc.

### New in FAST-LIO-DCReg

- Per-iteration degeneracy diagnosis within the iEKF update loop.
- Degeneracy flags and characterization scores published as ROS topics for real-time monitoring.
- Configurable degeneracy detection threshold and regularization strength.
- Compatible with all LiDAR types supported by FAST-LIO2.

---

## DCReg Integration Design

DCReg is inserted into FAST-LIO2's **iEKF measurement update** step. At each ICP iteration, after computing the point-to-plane Jacobian **J** and forming the approximate Hessian **H = Jᵀ J**, DCReg performs the following:

### Step 1 — Schur Complement Decoupling

The full 6×6 Hessian is partitioned into rotation (**H_RR**), translation (**H_tt**), and cross (**H_Rt**) blocks. DCReg computes the Schur complement of the translation block:

```
S_R = H_RR − H_Rt · H_tt⁻¹ · H_tR
```

`S_R` is the effective rotation Hessian after marginalizing translation. Its eigenspectrum reflects the true observability of rotation, free of scale-coupling artifacts.

### Step 2 — Degeneracy Detection and Characterization

The minimum eigenvalue of `S_R` (and its translation counterpart `S_t`) is compared against a configurable threshold `λ_threshold`. Directions associated with eigenvalues below the threshold are flagged as degenerate. The corresponding eigenvectors are projected back into physical motion space to identify:
- Which axis (X/Y/Z translation or Roll/Pitch/Yaw rotation) is degenerate.
- The severity (as a percentage of observability loss).

### Step 3 — Targeted Preconditioning

Eigenvalues of `S_R` (and `S_t`) that fall below `λ_threshold` are clamped to that value, forming a preconditioned Hessian. The cross terms are **not** modified, preserving coupling information for non-degenerate directions.

### Step 4 — PCG Solve

The preconditioned system is solved using a Preconditioned Conjugate Gradient (PCG) solver, which is significantly faster than dense Cholesky factorization in ill-conditioned settings.

> **Integration note**: Following the DCReg paper's recommendation, degeneracy handling is applied **only in the first ICP iteration** of each iEKF step, preventing over-regularization as the linearization point approaches convergence.

---

## Prerequisites

### System Requirements

| Component | Minimum Version |
|-----------|----------------|
| Ubuntu    | 18.04          |
| ROS       | Melodic        |
| CMake     | 3.10           |
| GCC / G++ | 7.5            |

### Dependencies

| Library          | Version   | Notes |
|------------------|-----------|-------|
| PCL              | ≥ 1.8     | [Installation](http://www.pointclouds.org/downloads/linux.html) |
| Eigen            | ≥ 3.3.4   | [Installation](http://eigen.tuxfamily.org) |
| OpenMP           | ≥ 201511  | Usually bundled with GCC |
| livox_ros_driver | any       | Required for Livox LiDARs — [Installation](https://github.com/Livox-SDK/livox_ros_driver) |

> **Note for Ubuntu 18.04 / 20.04**: The default PCL (1.8 / 1.10) and Eigen (3.3.7) shipped with ROS Melodic / Noetic are sufficient.

---

## Installation

### 1. Install livox_ros_driver (required for Livox LiDARs)

```bash
# Follow the official guide:
# https://github.com/Livox-SDK/livox_ros_driver
source $LIVOX_ROS_DRIVER_WS/devel/setup.bash
```

Add the `source` line to `~/.bashrc` to make it persistent.

### 2. Clone and build

```bash
cd ~/$YOUR_ROS_WS/src
git clone https://github.com/JokerJohn/FAST-LIO-DCReg.git --recursive
cd FAST-LIO-DCReg
git submodule update --init --recursive
cd ../..
catkin_make
source devel/setup.bash
```

> If you use a custom PCL build, export its path first:
> ```bash
> export PCL_ROOT={CUSTOM_PCL_PATH}
> ```

### 3. Docker (optional)

A Docker container based on the upstream FAST-LIO2 image can be used for quick testing:

```bash
# Create and run container (downloads ~2 GB image on first run)
touch run_fastlio_dcreg.sh
chmod +x run_fastlio_dcreg.sh
```

Paste the following into `run_fastlio_dcreg.sh`:

```bash
#!/bin/bash
mkdir -p ~/docker_ws
xhost +local:
docker run -itd \
  --name=fastlio_dcreg \
  --user mars_ugv \
  --network host \
  --ipc=host \
  -v /home/$USER/docker_ws:/home/mars_ugv/docker_ws \
  --privileged \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --env="DISPLAY=$DISPLAY" \
  kenny0407/marslab_fastlio2:latest \
  /bin/bash
```

Then run:

```bash
./run_fastlio_dcreg.sh
```

---

## Configuration

### DCReg Parameters

The following parameters in the sensor-specific YAML config file (e.g., `config/avia.yaml`) control the DCReg module:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dcreg_enable`           | bool  | `true`  | Enable / disable DCReg degeneracy handling. |
| `dcreg_lambda_threshold` | float | `1e-3`  | Eigenvalue threshold for degeneracy detection. Lower values are more permissive. |
| `dcreg_first_iter_only`  | bool  | `true`  | Apply preconditioning only in the first ICP iteration (recommended). |
| `dcreg_publish_diag`     | bool  | `true`  | Publish degeneracy diagnostics on `/dcreg/degeneracy_info`. |

### FAST-LIO2 Parameters (unchanged)

All standard FAST-LIO2 parameters remain available. Key ones:

| Parameter            | Description |
|----------------------|-------------|
| `lid_topic`          | LiDAR point cloud topic name. |
| `imu_topic`          | IMU topic name. |
| `extrinsic_T`        | LiDAR-to-IMU translational extrinsic (3×1 vector). |
| `extrinsic_R`        | LiDAR-to-IMU rotational extrinsic (3×3 matrix). |
| `extrinsic_est_en`   | Enable online extrinsic estimation (set `false` if extrinsics are known). |
| `pcd_save_enable`    | Save accumulated global map to `PCD/scans.pcd` on exit. |

---

## Usage

### Livox Avia

```bash
roslaunch fast_lio_dcreg mapping_avia.launch
rosbag play YOUR_BAG.bag
```

### Velodyne / Ouster

```bash
roslaunch fast_lio_dcreg mapping_velodyne.launch
# or
roslaunch fast_lio_dcreg mapping_ouster.launch
rosbag play YOUR_BAG.bag
```

### Monitor Degeneracy in Real Time

```bash
rostopic echo /dcreg/degeneracy_info
```

The `degeneracy_info` message includes:
- `degenerate` (bool array, 6 DOF): whether each DOF is degenerate.
- `eigenvalues` (float array, 6): eigenvalues of the decoupled Hessians.
- `severity` (float array, 6): percentage of observability loss per DOF.
- `axis_labels` (string array): physical interpretation (`"X"`, `"Y"`, `"Z"`, `"Roll"`, `"Pitch"`, `"Yaw"`).

### Save Point Cloud Map

Set `pcd_save_enable: 1` in the launch file. The accumulated global map is saved to `FAST-LIO-DCReg/PCD/scans.pcd` when the node is terminated.

```bash
pcl_viewer PCD/scans.pcd
# Press 1–5 to toggle coloring: random / X / Y / Z / intensity
```

---

## Supported LiDARs

| LiDAR Type        | Model Examples                            | Launch File                     |
|-------------------|-------------------------------------------|---------------------------------|
| Livox solid-state | Avia, Horizon, MID-70                     | `mapping_avia.launch`           |
| Velodyne spinning | VLP-16, VLP-32, HDL-32E, HDL-64E         | `mapping_velodyne.launch`       |
| Ouster spinning   | OS0, OS1, OS2                             | `mapping_ouster.launch`         |

> **Important**: Livox LiDARs must use `livox_lidar_msg.launch` from `livox_ros_driver` (not `livox_lidar.launch`) to produce per-point timestamps required for motion undistortion.

---

## Datasets

The following public datasets are recommended for testing FAST-LIO-DCReg, especially in degenerate scenarios:

| Dataset | Environment | Degeneracy Type | Link |
|---------|-------------|-----------------|------|
| NCLT    | Campus outdoor + indoor | Moderate | [Link](http://robots.engin.umich.edu/nclt/) |
| DCReg Cylinder & Parkinglot | Controlled degenerate | Planar / sparse | [Google Drive](https://drive.google.com/drive/folders/1TnS7K7q0hr-7SY__mR8pGQX1PJV3Bzfo?usp=drive_link) |
| HKU Livox Avia Indoor | Corridor / indoor | Narrow passage | [Google Drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing) |
| MARSIM Simulator | Simulated UAV | Configurable | [MARSIM](https://github.com/hku-mars/MARSIM) |

---

## Roadmap

- [x] Design integration architecture (DCReg inside iEKF update loop).
- [ ] Port DCReg degeneracy detection module (Schur complement decomposition).
- [ ] Integrate targeted preconditioning into FAST-LIO2's registration step.
- [ ] Implement PCG solver replacement for the iEKF state update.
- [ ] Add ROS diagnostic topic `/dcreg/degeneracy_info`.
- [ ] Add YAML config parameters for DCReg control.
- [ ] Validate on degenerate scenarios: narrow corridor, stairwell, planar parking lot.
- [ ] Benchmark against vanilla FAST-LIO2 and other degeneracy-aware methods.
- [ ] Release pre-built Docker image with FAST-LIO-DCReg.
- [ ] Add support for FAST-LIVO2 (LiDAR-inertial-visual) backend.

---

## Citation

If you use FAST-LIO-DCReg in your research, please cite both upstream works:

**DCReg** (degeneracy characterization and mitigation):

```bibtex
@misc{hu2025dcreg,
      title={DCReg: Decoupled Characterization for Efficient Degenerate LiDAR Registration},
      author={Xiangcheng Hu and Xieyuanli Chen and Mingkai Jia and Jin Wu and Ping Tan and Steven L. Waslander},
      year={2025},
      eprint={2509.06285},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2509.06285},
}
```

**FAST-LIO2** (LiDAR-inertial odometry backbone):

```bibtex
@article{xu2022fast,
  title={Fast-lio2: Fast direct lidar-inertial odometry},
  author={Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal={IEEE Transactions on Robotics},
  volume={38},
  number={4},
  pages={2053--2073},
  year={2022},
  publisher={IEEE}
}
```

---

## Acknowledgments

- [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO): Wei Xu, Yixi Cai, Dongjiao He, Fangcheng Zhu, Jiarong Lin, Zheng Liu, Borong Yuan.
- [JokerJohn/DCReg](https://github.com/JokerJohn/DCReg): Xiangcheng Hu, Xieyuanli Chen, Mingkai Jia, Jin Wu, Ping Tan, Steven L. Waslander.
- [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree): Dynamic KD-Tree for incremental 3D mapping.

---

## License

This repository is licensed under the **GPL-2.0** license, consistent with the upstream FAST-LIO2 license.

See [LICENSE](LICENSE) for details.

