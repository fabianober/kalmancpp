# Simple Kalman Filter in C++

A basic implementation of a 1D Kalman filter for educational purposes, demonstrating the core concepts of optimal estimation under uncertainty.

## 📋 Overview

This project implements a simple scalar Kalman filter that can track a single value over time by optimally combining noisy measurements with predictions. It's designed to be as simple as possible while still demonstrating the fundamental principles of Kalman filtering.

## 🎯 What is a Kalman Filter?

A Kalman filter is an optimal estimator that:

- **Predicts** where a system will be next based on a model
- **Updates** that prediction using noisy measurements
- **Balances trust** between predictions and measurements based on their uncertainties
- **Minimizes estimation error** over time

Think of it as a smart way to filter noise from sensor data while accounting for how confident you are in your predictions vs measurements.

## 📁 Project Structure

```
kalmancpp/
├── main.cpp           # Demo program showing filter in action
├── kalman.h           # Kalman filter class declaration
├── kalman.cpp         # Kalman filter implementation
├── utils.h            # Utility functions header
├── utils.cpp          # Utility functions implementation
├── .vscode/
│   └── tasks.json     # VS Code build/run tasks
└── README.md          # This file
```

## 🛠️ Building and Running

### Prerequisites

- C++ compiler with C++17 support (g++, clang++)
- VS Code (optional, but configured)

### Command Line Build

```bash
g++ -std=c++17 -Wall -Wextra -g -o program main.cpp utils.cpp kalman.cpp
./program
```

### VS Code Build

- Press `Cmd+Shift+B` (macOS) or `Ctrl+Shift+B` (Windows/Linux) to build
- Use `Cmd+Shift+P` → "Tasks: Run Task" → "Run C++ Program" to run

## 🚀 Demo Scenario

The program demonstrates a Kalman filter tracking a constant value (5.0) using noisy sensor measurements:

- **True Value**: 5.0 (what we're trying to estimate)
- **Initial Guess**: 0.0 (deliberately wrong to show convergence)
- **Noisy Measurements**: Values around 5.0 with random noise
- **Goal**: Filter the noise to get a better estimate

### Expected Output

```
Kalman Filter Demo
==================
True value: 5.0
Measurement -> Filtered Estimate
4.9 -> 4.45856
5.1 -> 4.78086
4.8 -> 4.78806
5.2 -> 4.9209
4.95 -> 4.92954
5.05 -> 4.96377
4.85 -> 4.9322
5.15 -> 4.99189
Final estimate: 4.99189
Final uncertainty: 0.0274074
```

## 🧮 How It Works

### The Filter Parameters

| Parameter                 | Value | Meaning                              |
| ------------------------- | ----- | ------------------------------------ |
| **Initial Estimate**      | 0.0   | Our first guess (deliberately wrong) |
| **Initial Error**         | 1.0   | How uncertain we are about our guess |
| **Process Noise (Q)**     | 0.01  | How much the true value might change |
| **Measurement Noise (R)** | 0.1   | How noisy our sensor readings are    |

### The Two-Step Cycle

1. **PREDICT**: Project current estimate forward

   - Estimate stays the same (constant model)
   - Uncertainty increases by process noise

2. **UPDATE**: Correct prediction with measurement
   - Calculate Kalman gain (trust ratio)
   - Blend prediction with measurement
   - Reduce uncertainty (gain confidence)

### Key Equations

```cpp
// Kalman Gain (trust ratio)
K = P / (P + R)

// State Update (blend prediction and measurement)
x = x + K * (measurement - x)

// Uncertainty Update (gain confidence)
P = (1 - K) * P
```

## 📊 What to Observe

1. **Convergence**: The estimate moves from 0.0 toward 5.0
2. **Noise Filtering**: Filtered values are closer to truth than raw measurements
3. **Uncertainty Reduction**: Final uncertainty (0.027) << initial uncertainty (1.0)
4. **Optimal Blending**: Early measurements have more influence than later ones

## 🎛️ Tuning Parameters

### Process Noise (Q)

- **Higher Q**: More responsive to changes, less smooth
- **Lower Q**: More stable, slower to adapt
- **Use when**: The true value might change over time

### Measurement Noise (R)

- **Higher R**: Trust measurements less, smoother output
- **Lower R**: Trust measurements more, more responsive
- **Use when**: You know how noisy your sensor is

## 🔬 Educational Value

This implementation is designed for learning:

- **Extensively commented** code explaining every step
- **Simple scalar math** (no matrices needed)
- **Clear variable names** matching textbook notation
- **Real-world scenario** that's easy to understand
- **Observable results** showing filter performance

## 🚀 Extensions

This basic filter could be extended to:

- **Multi-dimensional states** (position + velocity)
- **Different motion models** (constant velocity, acceleration)
- **Control inputs** (known forces/commands)
- **Non-linear systems** (Extended/Unscented Kalman Filters)

## 📚 Further Reading

- [Kalman Filter Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)
- "Kalman and Bayesian Filters in Python" by Roger Labbe
- "Optimal State Estimation" by Dan Simon

## 🤝 Contributing

This is an educational project. Feel free to:

- Add more examples
- Improve documentation
- Create additional demos
- Suggest parameter tuning exercises

## 📄 License

This project is for educational use. Feel free to use and modify as needed.

---

_Built with ❤️ for learning Kalman filtering fundamentals_
