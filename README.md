# SLAM with GTSAM

This project implements both 2D and 3D Simultaneous Localization and Mapping (SLAM) solutions using the GTSAM library. It reads pose and edge data from `.g2o` files, performs both batch and incremental optimization, and outputs the optimized poses.

## Features

- **2D SLAM**: 
  - Batch Optimization: Uses Gauss-Newton optimization for batch processing.
  - Incremental Optimization: Utilizes iSAM2 for real-time, incremental updates.
  
- **3D SLAM**:
  - Batch Optimization: Solves the SLAM problem using Gauss-Newton optimization.
  - Incremental Optimization: Uses iSAM2 for incremental updates to the SLAM solution.

- **Data Handling**: Reads and processes `.g2o` files containing pose and edge information for both 2D and 3D scenarios.

## Prerequisites

- **C++ Compiler**: Ensure you have a C++ compiler that supports C++11 or later.
- **GTSAM**: The Georgia Tech Smoothing and Mapping library must be installed. You can find installation instructions [here](https://gtsam.org/get_started/).
- **Eigen**: A C++ template library for linear algebra. GTSAM depends on Eigen, which is typically included with GTSAM.

## Building the Project

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/slam-gtsam.git
   cd slam-gtsam
