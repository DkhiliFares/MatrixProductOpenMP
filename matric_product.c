#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include <time.h>

// Function to generate a square matrix with random values
void generateMatrix(int N, double matrix[N][N]) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            matrix[i][j] = (double)rand() / RAND_MAX;
        }
    }
}

// Sequential matrix multiplication
void matrixMultiplySequential(int N, double A[N][N], double B[N][N], double C[N][N]) {
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
void matrixMultiplyParallelOuter(int N, double A[N][N], double B[N][N], double C[N][N]) {
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
void matrixMultiplyParallelMiddle(int N, double A[N][N], double B[N][N], double C[N][N]) {
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
void matrixMultiplyParallelOuterMiddle(int N, double A[N][N], double B[N][N], double C[N][N]) {
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
    srand(time(NULL));
    int N = 512;  // Default matrix size

    // Dynamically allocate matrices
    double (*A)[N] = malloc(sizeof(double[N][N]));
    double (*B)[N] = malloc(sizeof(double[N][N]));
    double (*C)[N] = malloc(sizeof(double[N][N]));

    generateMatrix(N, A);
    generateMatrix(N, B);

    // Sequential execution
    double start = omp_get_wtime();
    matrixMultiplySequential(N, A, B, C);
    double end = omp_get_wtime();
    printf("Sequential execution time: %f seconds\n", end - start);

    // Parallel outer loop
    start = omp_get_wtime();
    matrixMultiplyParallelOuter(N, A, B, C);
    end = omp_get_wtime();
    printf("Parallel outer loop execution time: %f seconds\n", end - start);

    // Parallel middle loop
    start = omp_get_wtime();
    matrixMultiplyParallelMiddle(N, A, B, C);
    end = omp_get_wtime();
    printf("Parallel middle loop execution time: %f seconds\n", end - start);

    // Parallel outer and middle loops
    start = omp_get_wtime();
    matrixMultiplyParallelOuterMiddle(N, A, B, C);
    end = omp_get_wtime();
    printf("Parallel outer and middle loops execution time: %f seconds\n", end - start);

    // Free dynamically allocated memory
    free(A);
    free(B);
    free(C);

    return 0;
}
