vim:ft=markdown

# Gemini Code Assistant Context

This document provides a comprehensive overview of the Photo-SLAM project, designed to be used as a context file for the Gemini Code Assistant.

## Project Overview

Photo-SLAM is a real-time Simultaneous Localization and Photorealistic Mapping (SLAM) system that works with Monocular, Stereo, and RGB-D cameras. It is a complex C++ project that integrates ORB-SLAM3 for robust and real-time camera pose estimation and its own novel Gaussian Splatting-based mapping module for creating photorealistic 3D maps.

The core of the project is divided into two main parts:
1.  **ORB-SLAM3**: A feature-based SLAM library used for tracking and localization. It is included as a submodule.
2.  **Gaussian Mapper**: The main mapping component of this project. It takes the keyframes and map points from ORB-SLAM3 and uses them to train a Gaussian Splatting model, resulting in a dense, photorealistic 3D representation of the environment. This component is implemented using C++ and CUDA, with LibTorch for the underlying deep learning framework.

The project also includes a viewer built with ImGui for visualizing the map and camera trajectory in real-time.

### Key Technologies

*   **Languages**: C++, CUDA
*   **Core Libraries**:
    *   **LibTorch**: For tensor operations and neural network components required by the Gaussian Splatting model.
    *   **OpenCV**: For image processing and computer vision tasks.
    *   **Eigen**: For linear algebra operations.
*   **Build System**: CMake
*   **Dependencies**: Boost, jsoncpp, OpenGL, GLFW, GLEW.

## Building and Running

The project is built using CMake. A convenience script, `build.sh`, is provided to compile the entire project, including the ORB-SLAM3 submodule and its dependencies.

### Building the Project

To build the project, run the following commands from the root directory:

```bash
chmod +x ./build.sh
./build.sh
```

This script will first compile the dependencies of ORB-SLAM3 (DBoW2, g2o, Sophus), then ORB-SLAM3 itself, and finally the Photo-SLAM libraries and executables. The final executables are placed in the `bin/` directory and the libraries in the `lib/` directory.

### Running an Example

The project provides several executables in the `bin/` directory for running the system on different datasets. For example, to run the system on the Replica dataset with an RGB-D camera, you can use the following command:

```bash
./bin/replica_rgbd \
    ./ORB-SLAM3/Vocabulary/ORBvoc.txt \
    ./cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
    ./cfg/gaussian_mapper/RGB-D/Replica/replica_rgbd.yaml \
    PATH_TO_Replica/office0 \
    PATH_TO_SAVE_RESULTS
```

The `scripts/` directory contains shell scripts that automate the process of running the system on various benchmark datasets.

## Development Conventions

*   **Code Style**: The code follows a modern C++ style, with extensive use of smart pointers (`std::shared_ptr`, `std::unique_ptr`) for memory management. The code is organized into classes and namespaces to maintain a clear structure.
*   **Configuration**: The system is highly configurable through YAML files located in the `cfg/` directory. There are separate configuration files for ORB-SLAM3 and the Gaussian Mapper.
*   **Modularity**: The project is highly modular. The SLAM and mapping components are decoupled and communicate through a well-defined interface. The `GaussianMapper` runs in a separate thread and processes the output from the `ORB_SLAM3::System`. This allows for a high degree of parallelism and makes the system more efficient.
*   **CUDA for Acceleration**: Computationally intensive parts of the Gaussian Splatting and rendering pipeline are implemented in CUDA to achieve real-time performance. These are found in files with `.cu` extensions (e.g., `src/rasterize_points.cu`).
*   **Viewer**: The optional viewer is built using ImGui and provides a way to visualize the system's output in real-time. The viewer code is located in the `viewer/` directory.
