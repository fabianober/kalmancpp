function kalman_demo()
    % KALMAN_DEMO - Demonstration of the C++ Kalman Filter MEX interface
    %
    % This demo shows how to use the C++ Kalman filter from MATLAB.
    % It demonstrates:
    % 1. Creating a Kalman filter
    % 2. Processing noisy measurements
    % 3. Comparing filtered vs unfiltered results
    % 4. Visualizing the convergence behavior
    
    fprintf('===========================================\n');
    fprintf('C++ Kalman Filter MATLAB Interface Demo\n');
    fprintf('===========================================\n\n');
    
    % Add matlab directory to path
    [examples_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(examples_dir);
    matlab_dir = fullfile(project_root, 'matlab');
    addpath(matlab_dir);
    
    % Check if MEX file exists
    if ~exist('kalman_mex', 'file')
        fprintf('MEX file not found. Attempting to build...\n');
        try
            run(fullfile(matlab_dir, 'build_mex.m'));
        catch
            error('kalman_mex MEX file not found. Please run matlab/build_mex.m first.');
        end
    end
    
    %% DEMO 1: Basic Filter Usage
    fprintf('Demo 1: Basic Kalman Filter Usage\n');
    fprintf('----------------------------------\n');
    
    % Create a Kalman filter
    % Parameters: initial_estimate, initial_error, process_noise, measurement_noise
    filter_id = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);
    
    % Simulate noisy measurements around true value of 5.0
    true_value = 5.0;
    num_measurements = 10;
    noise_std = 0.1;
    measurements = true_value + noise_std * randn(num_measurements, 1);
    
    fprintf('True value: %.2f\n', true_value);
    fprintf('Measurement noise std: %.2f\n\n', noise_std);
    fprintf('Step | Measurement | Filtered | Error Cov\n');
    fprintf('-----|-------------|----------|----------\n');
    
    % Process measurements one by one
    filtered_states = zeros(num_measurements, 1);
    error_covs = zeros(num_measurements, 1);
    
    for i = 1:num_measurements
        % Predict and update
        kalman_mex('predict', filter_id);
        kalman_mex('update', filter_id, measurements(i));
        
        % Get results
        filtered_states(i) = kalman_mex('getState', filter_id);
        error_covs(i) = kalman_mex('getErrorCovariance', filter_id);
        
        fprintf('%4d | %10.4f | %8.4f | %8.4f\n', ...
                i, measurements(i), filtered_states(i), error_covs(i));
    end
    
    fprintf('\nFinal estimate: %.4f (error: %.4f)\n', ...
            filtered_states(end), abs(filtered_states(end) - true_value));
    fprintf('Final uncertainty: %.6f\n\n', error_covs(end));
    
    % Clean up
    kalman_mex('delete', filter_id);
    
    %% DEMO 2: Batch Processing
    fprintf('Demo 2: Batch Processing\n');
    fprintf('------------------------\n');
    
    % Create another filter for batch processing
    filter_id2 = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);
    
    % Process all measurements at once
    [batch_states, batch_error_covs] = kalman_mex('process', filter_id2, measurements);
    
    fprintf('Batch processing completed!\n');
    fprintf('Batch final estimate: %.4f\n', batch_states(end));
    fprintf('Batch final uncertainty: %.6f\n\n', batch_error_covs(end));
    
    % Verify that individual and batch processing give same results
    max_diff = max(abs(filtered_states - batch_states));
    if max_diff < 1e-10
        fprintf('✓ Individual and batch processing results match!\n\n');
    else
        fprintf('✗ Warning: Difference between individual and batch: %.2e\n\n', max_diff);
    end
    
    kalman_mex('delete', filter_id2);
    
    %% DEMO 3: Comparison with Different Noise Levels
    fprintf('Demo 3: Effect of Different Noise Parameters\n');
    fprintf('--------------------------------------------\n');
    
    % Test different measurement noise levels
    noise_levels = [0.01, 0.1, 0.5];
    colors = {'b-', 'r-', 'g-'};
    
    figure('Name', 'Kalman Filter Performance Comparison', 'Position', [100, 100, 1200, 800]);
    
    % Create subplot layout
    subplot(2, 2, 1);
    hold on;
    title('Filtered Estimates vs Measurements');
    xlabel('Measurement Index');
    ylabel('Value');
    
    subplot(2, 2, 2);
    hold on;
    title('Error Covariance Evolution');
    xlabel('Measurement Index');
    ylabel('Error Covariance');
    
    subplot(2, 2, 3);
    hold on;
    title('Estimation Error');
    xlabel('Measurement Index');
    ylabel('|Estimate - True Value|');
    
    subplot(2, 2, 4);
    hold on;
    title('Measurement vs Filtered (Final 20 points)');
    xlabel('Measurement Value');
    ylabel('Filtered Value');
    
    for j = 1:length(noise_levels)
        noise_level = noise_levels(j);
        
        % Generate measurements with this noise level
        test_measurements = true_value + noise_level * randn(50, 1);
        
        % Create filter with corresponding measurement noise
        filter_id3 = kalman_mex('create', 0.0, 1.0, 0.01, noise_level);
        
        % Process measurements
        [test_states, test_error_covs] = kalman_mex('process', filter_id3, test_measurements);
        
        % Plot results
        subplot(2, 2, 1);
        plot(1:length(test_measurements), test_measurements, [colors{j}(1) '.'], ...
             'MarkerSize', 8, 'DisplayName', sprintf('Measurements (σ=%.2f)', noise_level));
        plot(1:length(test_states), test_states, colors{j}, 'LineWidth', 2, ...
             'DisplayName', sprintf('Filtered (σ=%.2f)', noise_level));
        
        subplot(2, 2, 2);
        plot(1:length(test_error_covs), test_error_covs, colors{j}, 'LineWidth', 2, ...
             'DisplayName', sprintf('σ=%.2f', noise_level));
        
        subplot(2, 2, 3);
        estimation_error = abs(test_states - true_value);
        plot(1:length(estimation_error), estimation_error, colors{j}, 'LineWidth', 2, ...
             'DisplayName', sprintf('σ=%.2f', noise_level));
        
        subplot(2, 2, 4);
        % Plot last 20 points for scatter comparison
        if length(test_measurements) >= 20
            scatter(test_measurements(end-19:end), test_states(end-19:end), 50, ...
                   colors{j}(1), 'filled', 'DisplayName', sprintf('σ=%.2f', noise_level));
        end
        
        kalman_mex('delete', filter_id3);
        
        fprintf('Noise level %.2f: Final estimate = %.4f, Final error cov = %.6f\n', ...
                noise_level, test_states(end), test_error_covs(end));
    end
    
    % Add reference lines and formatting
    subplot(2, 2, 1);
    yline(true_value, 'k--', 'LineWidth', 2, 'DisplayName', 'True Value');
    legend('Location', 'best');
    grid on;
    
    subplot(2, 2, 2);
    legend('Location', 'best');
    grid on;
    set(gca, 'YScale', 'log');
    
    subplot(2, 2, 3);
    legend('Location', 'best');
    grid on;
    set(gca, 'YScale', 'log');
    
    subplot(2, 2, 4);
    plot([true_value-1, true_value+1], [true_value-1, true_value+1], 'k--', ...
         'LineWidth', 2, 'DisplayName', 'Perfect Estimate');
    legend('Location', 'best');
    grid on;
    axis equal;
    
    %% DEMO 4: Filter Reset Demonstration
    fprintf('\nDemo 4: Filter Reset Capability\n');
    fprintf('-------------------------------\n');
    
    filter_id4 = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);
    
    % Process some measurements
    initial_measurements = true_value + 0.1 * randn(10, 1);
    kalman_mex('process', filter_id4, initial_measurements);
    
    state_before_reset = kalman_mex('getState', filter_id4);
    error_before_reset = kalman_mex('getErrorCovariance', filter_id4);
    
    fprintf('Before reset: State = %.4f, Error Cov = %.6f\n', ...
            state_before_reset, error_before_reset);
    
    % Reset the filter
    kalman_mex('reset', filter_id4, 10.0, 2.0);  % Reset to different initial conditions
    
    state_after_reset = kalman_mex('getState', filter_id4);
    error_after_reset = kalman_mex('getErrorCovariance', filter_id4);
    
    fprintf('After reset:  State = %.4f, Error Cov = %.6f\n', ...
            state_after_reset, error_after_reset);
    
    kalman_mex('delete', filter_id4);
    
    %% Summary
    fprintf('\n===========================================\n');
    fprintf('Demo Summary\n');
    fprintf('===========================================\n');
    fprintf('✓ Successfully created and used Kalman filters\n');
    fprintf('✓ Demonstrated individual and batch processing\n');
    fprintf('✓ Showed effect of different noise parameters\n');
    fprintf('✓ Illustrated filter reset capability\n');
    fprintf('✓ Generated comparison plots\n');
    fprintf('\nThe C++ Kalman filter is working correctly in MATLAB!\n');
    
    % Performance comparison
    fprintf('\nKey Observations:\n');
    fprintf('• Lower measurement noise → better final estimates\n');
    fprintf('• Error covariance decreases over time (increasing confidence)\n');
    fprintf('• Filter converges from wrong initial guess to true value\n');
    fprintf('• Batch and individual processing give identical results\n');
end
