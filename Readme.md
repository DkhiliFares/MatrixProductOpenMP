
# Matrix Multiply Service (ROS 2 Jazzy)

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![OpenMP](https://img.shields.io/badge/Parallel-OpenMP-orange)

A ROS 2 package that implements a matrix multiplication service using OpenMP for parallel computation. The service takes two matrices as input, performs their multiplication, and returns the result. Designed for ROS 2 Jazzy, it supports matrices of arbitrary sizes (e.g., 100x100).

## Features

- Custom ROS 2 service for matrix multiplication
- Parallelized computation using OpenMP for improved performance
- Supports arbitrary matrix sizes with dimension validation
- Example client for testing with 100x100 matrices

## Prerequisites

- ROS 2 Jazzy installed on Ubuntu 24.04 (or compatible)
- OpenMP library (`libomp-dev`)
- CMake and a C++ compiler (g++ or clang)
- Configured ROS 2 workspace (e.g., `~/ros2_ws`)

### Install dependencies:
```bash
sudo apt install libomp-dev
```

## Installation

1. Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/matrix_multiply_service.git
```
(Replace `your-username` with your GitHub username)

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select matrix_multiply_service
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Launch the matrix multiplication server:
```bash
ros2 run matrix_multiply_service matrix_multiply_server
```

2. In a separate terminal, run the client to test with 100x100 matrices:
```bash
ros2 run matrix_multiply_service matrix_multiply_client
```

The client will:
- Generate two 100x100 matrices with random values
- Send them to the server
- Print the result along with the execution time

## Project Structure

```
matrix_multiply_service/
├── srv/
│   └── MatrixMultiply.srv          # Service definition
├── src/
│   ├── matrix_multiply_server.cpp  # Server node with OpenMP
│   └── matrix_multiply_client.cpp  # Test client
├── CMakeLists.txt                  # Build configuration
├── package.xml                     # Package metadata
└── README.md
```

## Performance

- Uses OpenMP to parallelize matrix multiplication
- For 100x100 matrices, execution time depends on your CPU:
  - Typically a few milliseconds to tens of milliseconds with multiple threads
- To adjust performance: Modify `omp_set_num_threads(n)` in `matrix_multiply_server.cpp`

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Service not found | Ensure server is running and workspace is sourced |
| Compilation errors | Verify `libomp-dev` is installed and ROS 2 dependencies are met |
| Long execution time | Check OpenMP support (`g++ --version`) and adjust thread count |

