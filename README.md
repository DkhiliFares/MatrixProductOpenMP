# 🔢 Parallel Matrix Multiplication with OpenMP

This C++ project demonstrates different approaches to matrix multiplication using **OpenMP** for parallel computing. It compares the performance of sequential and parallel implementations across various loop configurations.

---

## 🚀 Features

- Matrix generation with random floating-point values
- **Sequential matrix multiplication**
- **Parallel multiplication** using OpenMP:
  - Outer loop parallelization
  - Middle loop parallelization
  - Outer and middle loops combined (`collapse(2)`)

---

## 🛠️ Technologies

- C++
- OpenMP
- Standard Template Library (`vector`)
- Random number generation
- Performance benchmarking (`omp_get_wtime`, `clock`)

---

## 📈 Performance Evaluation

The program prints the execution time for:

- ✅ Sequential execution
- 🧵 Parallel (outer loop)
- 🧵 Parallel (middle loop)
- 🧵 Parallel (outer + middle loop with collapse)

This allows easy comparison of efficiency gains from different parallel strategies.

---

## ⚙️ How to Compile

Make sure OpenMP is enabled in your compiler:

```bash
g++ -fopenmp -O2 matrix_multiplication.cpp -o matrix_multiplication
```

Then run:

```bash
./matrix_multiplication
```

---

## 📌 Example Output

```
nombre max de threads egale a 8
Parallel outer loop execution time: 3.51 seconds
Parallel middle loop execution time: 3.78 seconds
Parallel outer and middle loops execution time: 2.89 seconds
Sequential execution time: 15.62 seconds
```

> ⏱️ Actual results depend on system specs and matrix size.

---

## 📚 Notes

- Default matrix size: `1024 x 1024`
- Modify the `N` variable in `main()` to test with different sizes.
- Matrix multiplication is computationally intensive: larger sizes benefit more from parallelization.

---

## 🧠 Author

**Fares Dkhili**  
🔗 [GitHub](https://github.com/DkhiliFares) | 📫 dkhili.fares@gmail.com

---

> *Optimizing performance through parallelism — one loop at a time.*
```


