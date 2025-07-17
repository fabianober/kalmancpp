function test_kalman_interface()
    % TEST_KALMAN_INTERFACE - Quick test of the MATLAB Kalman filter interface
    %
    % This script performs basic functionality tests to ensure the MEX interface
    % and MATLAB wrapper are working correctly.
    
    fprintf('Testing Kalman Filter MATLAB Interface\n');
    fprintf('======================================\n\n');
    
    success_count = 0;
    total_tests = 0;
    
    %% Test 1: MEX File Compilation
    total_tests = total_tests + 1;
    fprintf('Test 1: Checking MEX file availability... ');
    
    try
        if exist('kalman_mex', 'file')
            fprintf('✓ PASSED\n');
            success_count = success_count + 1;
        else
            fprintf('✗ FAILED - MEX file not found. Run build_mex() first.\n');
        end
    catch ME
        fprintf('✗ FAILED - %s\n', ME.message);
    end
    
    %% Test 2: Basic MEX Interface
    if exist('kalman_mex', 'file')
        total_tests = total_tests + 1;
        fprintf('Test 2: Testing basic MEX interface... ');
        
        try
            % Test create/delete cycle
            filter_id = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);
            kalman_mex('delete', filter_id);
            fprintf('✓ PASSED\n');
            success_count = success_count + 1;
        catch ME
            fprintf('✗ FAILED - %s\n', ME.message);
        end
        
        %% Test 3: MEX Operations
        total_tests = total_tests + 1;
        fprintf('Test 3: Testing MEX operations... ');
        
        try
            filter_id = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);
            
            % Test predict/update cycle
            kalman_mex('predict', filter_id);
            kalman_mex('update', filter_id, 5.0);
            
            % Test getters
            state = kalman_mex('getState', filter_id);
            error_cov = kalman_mex('getErrorCovariance', filter_id);
            
            % Test reset
            kalman_mex('reset', filter_id, 2.0, 0.5);
            
            % Test batch processing
            measurements = [4.9, 5.1, 4.8];
            [states, error_covs] = kalman_mex('process', filter_id, measurements);
            
            kalman_mex('delete', filter_id);
            
            if length(states) == 3 && length(error_covs) == 3
                fprintf('✓ PASSED\n');
                success_count = success_count + 1;
            else
                fprintf('✗ FAILED - Incorrect output dimensions\n');
            end
        catch ME
            fprintf('✗ FAILED - %s\n', ME.message);
        end
        
        %% Test 4: MATLAB Wrapper Class
        total_tests = total_tests + 1;
        fprintf('Test 4: Testing MATLAB wrapper class... ');
        
        try
            % Test constructor
            filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);
            
            % Test basic operations
            filter.predict();
            filter.update(5.0);
            state = filter.getState();
            error_cov = filter.getErrorCovariance();
            
            % Test convenience methods
            filter.step(4.9);
            current_state = filter.getCurrentState();
            
            % Test reset
            filter.reset(3.0, 0.8);
            
            % Test batch processing
            measurements = [4.8, 5.2, 4.9, 5.1];
            [states, error_covs] = filter.process(measurements);
            
            if length(states) == 4 && length(error_covs) == 4 && ...
               isfield(current_state, 'state') && isfield(current_state, 'error_covariance')
                fprintf('✓ PASSED\n');
                success_count = success_count + 1;
            else
                fprintf('✗ FAILED - Incorrect functionality\n');
            end
            
            % Destructor will be called automatically
            clear filter;
            
        catch ME
            fprintf('✗ FAILED - %s\n', ME.message);
        end
        
        %% Test 5: Numerical Correctness
        total_tests = total_tests + 1;
        fprintf('Test 5: Testing numerical correctness... ');
        
        try
            % Test that filter converges to true value
            true_value = 10.0;
            noise_std = 0.1;
            num_measurements = 50;
            
            % Generate measurements
            rng(42); % For reproducible results
            measurements = true_value + noise_std * randn(num_measurements, 1);
            
            % Create filter
            filter = KalmanFilter(0.0, 2.0, 0.001, noise_std^2);
            
            % Process measurements
            [states, error_covs] = filter.process(measurements);
            
            % Check convergence
            final_estimate = states(end);
            final_error = error_covs(end);
            estimation_error = abs(final_estimate - true_value);
            
            if estimation_error < 0.5 && final_error < 0.1 && final_error > 0
                fprintf('✓ PASSED\n');
                fprintf('    Final estimate: %.4f (true: %.1f, error: %.4f)\n', ...
                        final_estimate, true_value, estimation_error);
                fprintf('    Final uncertainty: %.6f\n', final_error);
                success_count = success_count + 1;
            else
                fprintf('✗ FAILED - Poor convergence\n');
                fprintf('    Final estimate: %.4f (error: %.4f)\n', final_estimate, estimation_error);
                fprintf('    Final uncertainty: %.6f\n', final_error);
            end
            
        catch ME
            fprintf('✗ FAILED - %s\n', ME.message);
        end
        
    else
        fprintf('Skipping MEX-dependent tests (MEX file not available)\n');
    end
    
    %% Test Summary
    fprintf('\n');
    fprintf('Test Summary\n');
    fprintf('============\n');
    fprintf('Passed: %d/%d tests\n', success_count, total_tests);
    
    if success_count == total_tests
        fprintf('✓ All tests passed! The interface is working correctly.\n');
        fprintf('\nYou can now:\n');
        fprintf('• Run kalman_demo() for a comprehensive demonstration\n');
        fprintf('• Use KalmanFilter class in your own MATLAB code\n');
        fprintf('• Check MATLAB_README.md for detailed usage instructions\n');
    else
        fprintf('✗ Some tests failed. Please check the error messages above.\n');
        fprintf('\nTroubleshooting:\n');
        fprintf('• Run build_mex() to compile the MEX file\n');
        fprintf('• Check that your compiler supports C++17\n');
        fprintf('• Verify MATLAB version compatibility (R2017b+)\n');
    end
    
    fprintf('\n');
end
