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
    int num_threads = omp_get_max_threads(); // Nombre de threads souhaité
    printf("nombre max de thread egale: %d",num_threads);
    omp_set_num_threads(num_threads);
    srand(time(NULL));
    int N = 1024;  // Default matrix size

    // Dynamically allocate matrices
    double (*A)[N] = malloc(sizeof(double[N][N]));
    double (*B)[N] = malloc(sizeof(double[N][N]));
    double (*C)[N] = malloc(sizeof(double[N][N]));

    generateMatrix(N, A);
    generateMatrix(N, B);

  
    // Parallel outer loop
    double startp, endp;
    startp = omp_get_wtime();
    matrixMultiplyParallelOuter(N, A, B, C);
    endp = omp_get_wtime();
    printf("Parallel outer loop execution time: %f seconds\n", endp - startp);

    // Parallel middle loop
    startp = omp_get_wtime();
    matrixMultiplyParallelMiddle(N, A, B, C);
    endp = omp_get_wtime();
    printf("Parallel middle loop execution time: %f seconds\n", endp - startp);

    // Parallel outer and middle loops
    startp = omp_get_wtime();
    matrixMultiplyParallelOuterMiddle(N, A, B, C);
    endp = omp_get_wtime();
    printf("Parallel outer and middle loops execution time: %f seconds\n", endp - startp);
  // Sequential execution
    clock_t start,end;
    start=clock();
    matrixMultiplySequential(N, A, B, C);
    end=clock();
    double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("Sequential execution time: %f seconds\n", cpu_time_used );
    
    // Free dynamically allocated memory
    free(A);
    free(B);
    free(C);

    return 0;
}
