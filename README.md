# 3D SLAM with GTSAM

This project implements a 3D Simultaneous Localization and Mapping (SLAM) solution using the GTSAM library. It reads pose and edge data from a `.g2o` file, performs both batch and incremental optimization, and outputs the optimized poses.

## Features

- **Batch Optimization**: Uses Gauss-Newton optimization to solve the SLAM problem in a batch manner.
- **Incremental Optimization**: Utilizes iSAM2 for real-time, incremental updates to the SLAM solution.
- **Data Handling**: Reads and processes `.g2o` files containing 3D pose and edge information.

## Prerequisites

- **C++ Compiler**: Ensure you have a C++ compiler that supports C++11 or later.
- **GTSAM**: The Georgia Tech Smoothing and Mapping library must be installed. You can find installation instructions [here](https://gtsam.org/get_started/).
- **Eigen**: A C++ template library for linear algebra. GTSAM depends on Eigen, which is typically included with GTSAM.

## Building the Project

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/3d-slam-gtsam.git
   cd 3d-slam-gtsam
