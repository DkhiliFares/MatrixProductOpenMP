Matrix Multiply Service (ROS 2 Jazzy)
This is a ROS 2 package that implements a matrix multiplication service using OpenMP for parallel computation. The service takes two matrices as input, performs their multiplication, and returns the result. It is designed to work with ROS 2 Jazzy and supports matrices of arbitrary sizes (e.g., 100x100).
Features

Custom ROS 2 service for matrix multiplication.
Parallelized computation using OpenMP for improved performance.
Supports arbitrary matrix sizes with dimension validation.
Example client for testing with 100x100 matrices.

Prerequisites

ROS 2 Jazzy installed on Ubuntu 24.04 (or compatible).
OpenMP library (libomp-dev).
CMake and a C++ compiler (e.g., g++ or clang).
A configured ROS 2 workspace (e.g., ~/ros2_ws).

Install dependencies:
sudo apt install libomp-dev

Installation

Clone the repository into your ROS 2 workspace:
cd ~/ros2_ws/src
git clone https://github.com/your-username/matrix_multiply_service.git

Replace your-username with your GitHub username.

Build the package:
cd ~/ros2_ws
colcon build --packages-select matrix_multiply_service


Source the workspace:
source ~/ros2_ws/install/setup.bash



Usage

Launch the matrix multiplication server:
ros2 run matrix_multiply_service matrix_multiply_server


In a separate terminal, run the client to test with 100x100 matrices:
ros2 run matrix_multiply_service matrix_multiply_client

The client generates two 100x100 matrices with random values, sends them to the server, and prints the result along with the execution time.


Project Structure

srv/MatrixMultiply.srv: Service definition for matrix multiplication.
src/matrix_multiply_server.cpp: Server node implementing the multiplication with OpenMP.
src/matrix_multiply_client.cpp: Client node for testing with 100x100 matrices.
CMakeLists.txt and package.xml: Build configuration files.

Performance

The service uses OpenMP to parallelize matrix multiplication, improving performance for large matrices.
For 100x100 matrices, execution time depends on your CPU (typically a few milliseconds to tens of milliseconds with multiple threads).
To adjust the number of OpenMP threads, modify omp_set_num_threads(n) in matrix_multiply_server.cpp.

Troubleshooting

Service not found: Ensure the server is running and the workspace is sourced.
Compilation errors: Verify that libomp-dev is installed and ROS 2 dependencies are met.
Long execution time: Check OpenMP support (g++ --version) and adjust thread count.

License
This project is licensed under the MIT License. See the LICENSE file for details.
Contact
For issues or contributions, please open an issue or pull request on the GitHub repository.
