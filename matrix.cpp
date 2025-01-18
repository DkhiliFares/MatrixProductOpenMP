#include <iostream>
#include <vector>
#include <cstdlib>
#include <omp.h>
#include <ctime>

using namespace std;

// Function to generate a square matrix with random values
void generateMatrix(int N, vector<vector<double>> &matrix) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            matrix[i][j] = (double)rand() / RAND_MAX;
        }
    }
}

// Sequential matrix multiplication
void matrixMultiplySequential(int N, const vector<vector<double>> &A, const vector<vector<double>> &B, vector<vector<double>> &C) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            C[i][j] = 0;
            for (int k = 0; k < N; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Parallel matrix multiplication (outer loop parallelized)
void matrixMultiplyParallelOuter(int N, const vector<vector<double>> &A, const vector<vector<double>> &B, vector<vector<double>> &C) {
    #pragma omp parallel for
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            C[i][j] = 0;
            for (int k = 0; k < N; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Parallel matrix multiplication (middle loop parallelized)
void matrixMultiplyParallelMiddle(int N, const vector<vector<double>> &A, const vector<vector<double>> &B, vector<vector<double>> &C) {
    for (int i = 0; i < N; i++) {
        #pragma omp parallel for
        for (int j = 0; j < N; j++) {
            C[i][j] = 0;
            for (int k = 0; k < N; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Parallel matrix multiplication (outer and middle loops parallelized)
void matrixMultiplyParallelOuterMiddle(int N, const vector<vector<double>> &A, const vector<vector<double>> &B, vector<vector<double>> &C) {
    #pragma omp parallel for collapse(2)
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            C[i][j] = 0;
            for (int k = 0; k < N; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

int main() {
    int num_threads = omp_get_max_threads(); // Nombre de threads souhaité
    cout<<"nombre max de threads egale a "<<num_threads;
    omp_set_num_threads(num_threads);
    srand(time(NULL));
    int N = 1024;  // Default matrix size

    // Dynamically allocate matrices
    vector<vector<double>> A(N, vector<double>(N));
    vector<vector<double>> B(N, vector<double>(N));
    vector<vector<double>> C(N, vector<double>(N));

    generateMatrix(N, A);
    generateMatrix(N, B);

    // Parallel outer loop
    double startp, endp;
    startp = omp_get_wtime();
    matrixMultiplyParallelOuter(N, A, B, C);
    endp = omp_get_wtime();
    cout << "Parallel outer loop execution time: " << (endp - startp) << " seconds" << endl;

    // Parallel middle loop
    startp = omp_get_wtime();
    matrixMultiplyParallelMiddle(N, A, B, C);
    endp = omp_get_wtime();
    cout << "Parallel middle loop execution time: " << (endp - startp) << " seconds" << endl;

    // Parallel outer and middle loops
    startp = omp_get_wtime();
    matrixMultiplyParallelOuterMiddle(N, A, B, C);
    endp = omp_get_wtime();
    cout << "Parallel outer and middle loops execution time: " << (endp - startp) << " seconds" << endl;

    // Sequential execution
    clock_t start, end;
    start = clock();
    matrixMultiplySequential(N, A, B, C);
    end = clock();
    double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
    cout << "Sequential execution time: " << cpu_time_used << " seconds" << endl;

    return 0;
}
