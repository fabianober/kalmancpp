# MATLAB Interface for C++ Kalman Filter

This project provides a MATLAB interface to a C++ implementation of a Simple Kalman Filter using MEX files. This allows MATLAB users to leverage the performance of C++ while working in their familiar MATLAB environment.

## Features

- **High Performance**: C++ implementation for computational efficiency
- **MATLAB Integration**: Native MATLAB interface using MEX files
- **Object-Oriented**: MATLAB wrapper class for easy usage
- **Batch Processing**: Process multiple measurements efficiently
- **Visualization**: Built-in plotting capabilities for analysis
- **Memory Safe**: Automatic cleanup of C++ resources

## Files Overview

### Core Implementation

- `kalman.h` / `kalman.cpp` - C++ Kalman filter implementation
- `kalman_mex.cpp` - MEX interface between MATLAB and C++

### MATLAB Interface

- `KalmanFilter.m` - Object-oriented MATLAB wrapper class
- `build_mex.m` - Script to compile the MEX file
- `kalman_demo.m` - Comprehensive demonstration script

### Utilities

- `utils.h` / `utils.cpp` - Utility functions
- `main.cpp` - C++ standalone demo (optional)

## Quick Start

### 1. Compile the MEX File

Open MATLAB and navigate to this directory, then run:

```matlab
build_mex()
```

This will compile the C++ code into a MEX file that MATLAB can call.

### 2. Run the Demo

```matlab
kalman_demo()
```

This will run a comprehensive demonstration showing various features of the Kalman filter.

### 3. Basic Usage

```matlab
% Create a Kalman filter
filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);

% Process measurements one by one
measurements = [4.9, 5.1, 4.8, 5.2, 4.95];
for i = 1:length(measurements)
    filter.step(measurements(i));  % predict + update
    fprintf('Estimate: %.4f\n', filter.getState());
end

% Or process all measurements at once
[states, uncertainties] = filter.process(measurements);

% Visualize results
filter.plotResults(measurements, 'TrueValue', 5.0);
```

## Detailed Usage

### Constructor Parameters

```matlab
filter = KalmanFilter(initial_estimate, initial_error, process_noise, measurement_noise)
```

- **initial_estimate**: Your first guess of the true value
- **initial_error**: How uncertain you are about your initial guess
- **process_noise**: How much you expect the true value to change over time
- **measurement_noise**: How noisy/unreliable your sensor measurements are

### Methods

#### Basic Operations

- `predict()` - Perform prediction step
- `update(measurement)` - Perform measurement update
- `step(measurement)` - Combined predict + update
- `getState()` - Get current state estimate
- `getErrorCovariance()` - Get current uncertainty

#### Advanced Operations

- `process(measurements)` - Batch process multiple measurements
- `reset(initial_estimate, initial_error)` - Reset filter
- `getCurrentState()` - Get complete state structure
- `plotResults(measurements, ...)` - Visualize performance

### MEX Interface (Low-Level)

If you prefer to use the MEX interface directly:

```matlab
% Create filter
filter_id = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);

% Use filter
kalman_mex('predict', filter_id);
kalman_mex('update', filter_id, 5.0);
state = kalman_mex('getState', filter_id);

% Clean up
kalman_mex('delete', filter_id);
```

## Examples

### Example 1: Noise Filtering

```matlab
% Generate noisy measurements around true value of 10.0
true_value = 10.0;
noise_std = 0.5;
measurements = true_value + noise_std * randn(100, 1);

% Create and use filter
filter = KalmanFilter(0.0, 2.0, 0.01, noise_std^2);
[filtered_states, uncertainties] = filter.process(measurements);

% Compare results
fprintf('True value: %.2f\n', true_value);
fprintf('Mean measurement: %.2f (std: %.2f)\n', mean(measurements), std(measurements));
fprintf('Final filtered estimate: %.2f\n', filtered_states(end));
fprintf('Final uncertainty: %.4f\n', uncertainties(end));
```

### Example 2: Real-Time Processing

```matlab
% Simulate real-time sensor data
filter = KalmanFilter(0.0, 1.0, 0.001, 0.1);

figure;
h_meas = plot(NaN, NaN, 'b.', 'MarkerSize', 10);
hold on;
h_filt = plot(NaN, NaN, 'r-', 'LineWidth', 2);
legend('Measurements', 'Filtered', 'Location', 'best');

for i = 1:100
    % Simulate new measurement
    measurement = 5.0 + 0.1 * randn();

    % Update filter
    filter.step(measurement);
    state = filter.getState();

    % Update plot
    h_meas.XData(end+1) = i;
    h_meas.YData(end+1) = measurement;
    h_filt.XData(end+1) = i;
    h_filt.YData(end+1) = state;

    drawnow;
    pause(0.1);
end
```

### Example 3: Parameter Tuning

```matlab
% Test different noise parameters
true_value = 7.5;
measurements = true_value + 0.2 * randn(50, 1);

% Different process noise values
process_noises = [0.001, 0.01, 0.1];
colors = {'b', 'r', 'g'};

figure;
hold on;
plot(measurements, 'k.', 'MarkerSize', 8, 'DisplayName', 'Measurements');

for i = 1:length(process_noises)
    filter = KalmanFilter(0.0, 1.0, process_noises(i), 0.04);
    [states, ~] = filter.process(measurements);
    plot(states, colors{i}, 'LineWidth', 2, ...
         'DisplayName', sprintf('Q = %.3f', process_noises(i)));
end

yline(true_value, 'k--', 'LineWidth', 2, 'DisplayName', 'True Value');
legend('Location', 'best');
title('Effect of Process Noise Parameter');
```

## Compilation Requirements

### MATLAB Requirements

- MATLAB R2017b or later (for C++17 support)
- MATLAB Coder (for MEX compilation)

### Compiler Requirements

#### Windows

- Visual Studio 2017 or later, OR
- MinGW-w64 compiler

#### macOS

- Xcode Command Line Tools: `xcode-select --install`
- macOS 10.14 or later

#### Linux

- GCC 7.0 or later with C++17 support
- Build essentials: `sudo apt-get install build-essential` (Ubuntu/Debian)

### Setup Compiler in MATLAB

If you encounter compilation issues, configure your compiler:

```matlab
mex -setup cpp
```

## Troubleshooting

### Common Issues

1. **MEX file not found**

   - Run `build_mex()` to compile the MEX file
   - Check that compilation completed without errors

2. **Compiler not found**

   - Install appropriate compiler for your platform
   - Run `mex -setup cpp` in MATLAB

3. **C++17 not supported**

   - Update your compiler
   - Use MATLAB R2017b or later

4. **Memory issues**
   - The MATLAB wrapper automatically manages memory
   - Use `clear classes` if experiencing issues

### Performance Tips

1. **Use batch processing** for large datasets:

   ```matlab
   [states, errors] = filter.process(measurements);  % Fast
   ```

   Instead of:

   ```matlab
   for i = 1:length(measurements)  % Slower
       filter.step(measurements(i));
   end
   ```

2. **Reuse filter objects** instead of creating new ones

3. **Choose appropriate parameters** based on your system characteristics

## Theory

The Kalman filter implemented here is a 1D (scalar) filter that operates in two steps:

1. **Predict**: Project the current state forward in time

   - State prediction: `x_pred = x_current` (constant model)
   - Covariance prediction: `P_pred = P_current + Q`

2. **Update**: Correct the prediction using a measurement
   - Kalman gain: `K = P_pred / (P_pred + R)`
   - State update: `x_new = x_pred + K * (measurement - x_pred)`
   - Covariance update: `P_new = (1 - K) * P_pred`

Where:

- `Q` is the process noise (how much the system changes)
- `R` is the measurement noise (how noisy sensors are)
- `P` is the error covariance (uncertainty in estimate)

## License

This project is provided as-is for educational and research purposes.

## Contributing

Feel free to submit issues, suggestions, or improvements to enhance the MATLAB interface or extend functionality.
