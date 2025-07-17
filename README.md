# Kalman Filter C++ Implementation with MATLAB Interface

A high-performance C++ implementation of a Simple Kalman Filter with a complete MATLAB interface using MEX files.

## Project Structure

```
kalmancpp/
├── src/                    # C++ source code
│   ├── kalman.h           # Kalman filter header
│   ├── kalman.cpp         # Kalman filter implementation
│   ├── utils.h            # Utility functions header
│   └── utils.cpp          # Utility functions implementation
├── matlab/                 # MATLAB interface
│   ├── kalman_mex.cpp     # MEX interface code
│   ├── KalmanFilter.m     # MATLAB wrapper class
│   ├── build_mex.m        # MEX compilation script
│   └── test_kalman_interface.m  # Test script
├── examples/               # Example code and demos
│   ├── main.cpp           # C++ standalone example
│   └── kalman_demo.m      # MATLAB demo script
├── docs/                   # Documentation
│   └── MATLAB_README.md   # Detailed MATLAB usage guide
└── README.md              # This file
```

## Quick Start

### C++ Standalone Version

1. **Build the C++ project:**

   ```bash
   g++ -std=c++17 -Wall -Wextra -g -o program src/kalman.cpp src/utils.cpp examples/main.cpp
   ```

2. **Run the example:**
   ```bash
   ./program
   ```

### MATLAB Interface

1. **Open MATLAB and navigate to the project root directory**

2. **Build the MEX interface:**

   ```matlab
   cd matlab
   build_mex()
   ```

3. **Run the demo:**

   ```matlab
   cd ../examples
   kalman_demo()
   ```

4. **Or use the object-oriented interface:**
   ```matlab
   addpath('matlab')
   filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);
   filter.step(5.0);  % Process a measurement
   estimate = filter.getState();
   ```

## Features

### C++ Implementation

- **High Performance**: Optimized C++ implementation
- **Simple Interface**: Easy-to-use class-based API
- **Well Documented**: Extensive comments explaining the theory
- **Memory Safe**: Proper resource management

### MATLAB Interface

- **MEX Integration**: High-performance C++ backend with MATLAB frontend
- **Object-Oriented**: Clean MATLAB class wrapper
- **Batch Processing**: Efficient processing of multiple measurements
- **Visualization**: Built-in plotting capabilities
- **Error Handling**: Comprehensive error checking and reporting

## Usage Examples

### Basic C++ Usage

```cpp
#include "src/kalman.h"

// Create filter: initial_estimate, initial_error, process_noise, measurement_noise
SimpleKalmanFilter filter(0.0, 1.0, 0.01, 0.1);

// Process measurements
std::vector<double> measurements = {4.9, 5.1, 4.8, 5.2};
for (double measurement : measurements) {
    filter.predict();
    filter.update(measurement);
    std::cout << "Estimate: " << filter.getState() << std::endl;
}
```

### Basic MATLAB Usage

```matlab
% Add MATLAB directory to path
addpath('matlab')

% Create filter
filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);

% Process measurements
measurements = [4.9, 5.1, 4.8, 5.2];
[estimates, uncertainties] = filter.process(measurements);

% Visualize results
filter.plotResults(measurements, 'TrueValue', 5.0);
```

## Dependencies

### C++ Requirements

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- Standard library only (no external dependencies)

### MATLAB Requirements

- MATLAB R2017b or later
- MEX compiler configured (`mex -setup cpp`)
- For macOS: Xcode Command Line Tools
- For Windows: Visual Studio or MinGW
- For Linux: GCC with C++17 support

## Documentation

- **[MATLAB Interface Guide](docs/MATLAB_README.md)** - Comprehensive MATLAB usage documentation
- **[C++ API Documentation](src/kalman.h)** - Detailed C++ interface documentation
- **[Examples](examples/)** - Working examples in both C++ and MATLAB

## Testing

### Test C++ Implementation

```bash
# Build and run C++ tests
g++ -std=c++17 -Wall -Wextra -g -o program src/kalman.cpp src/utils.cpp examples/main.cpp
./program
```

### Test MATLAB Interface

```matlab
cd matlab
test_kalman_interface()
```

## Theory

This implements a simple 1D Kalman filter for tracking a scalar value. The filter operates in two steps:

1. **Predict**: Project current state estimate forward in time
2. **Update**: Correct prediction using new measurement

Key parameters:

- **Process Noise (Q)**: How much the true value changes over time
- **Measurement Noise (R)**: How noisy the sensor measurements are
- **Initial Error (P)**: Initial uncertainty in the estimate

The filter automatically balances trust between predictions and measurements based on their relative uncertainties.

## License

This project is provided for educational and research purposes.

## Contributing

Contributions are welcome! Please feel free to submit issues, suggestions, or pull requests.
