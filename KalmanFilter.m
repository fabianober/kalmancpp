classdef KalmanFilter < handle
    % KALMANFILTER - MATLAB wrapper class for C++ SimpleKalmanFilter
    %
    % This class provides an object-oriented MATLAB interface to the C++
    % Kalman filter implementation. It makes the filter easier to use by
    % hiding the MEX interface details and providing MATLAB-style methods.
    %
    % Example Usage:
    %   % Create a filter
    %   filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);
    %   
    %   % Process measurements
    %   measurements = [4.9, 5.1, 4.8, 5.2];
    %   for i = 1:length(measurements)
    %       filter.predict();
    %       filter.update(measurements(i));
    %       fprintf('Estimate: %.4f\n', filter.getState());
    %   end
    %   
    %   % Or process all at once
    %   [states, errors] = filter.process(measurements);
    %
    % See also: kalman_mex, kalman_demo
    
    properties (Access = private)
        filter_id  % Handle to the C++ filter instance
    end
    
    methods
        function obj = KalmanFilter(initial_estimate, initial_error, process_noise, measurement_noise)
            % KALMANFILTER Constructor
            %
            % Creates a new Kalman filter with the specified parameters.
            %
            % Inputs:
            %   initial_estimate   - Initial guess of the state value
            %   initial_error      - Initial uncertainty (error covariance)
            %   process_noise      - How much the system changes over time
            %   measurement_noise  - How noisy the measurements are
            %
            % Example:
            %   filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);
            
            if nargin ~= 4
                error('KalmanFilter:InvalidInput', ...
                      'Constructor requires 4 arguments: initial_estimate, initial_error, process_noise, measurement_noise');
            end
            
            % Check if MEX file is available
            if ~exist('kalman_mex', 'file')
                error('KalmanFilter:MissingMEX', ...
                      'kalman_mex MEX file not found. Run build_mex() first.');
            end
            
            % Create the C++ filter instance
            obj.filter_id = kalman_mex('create', initial_estimate, initial_error, ...
                                      process_noise, measurement_noise);
        end
        
        function delete(obj)
            % DELETE Destructor
            %
            % Automatically called when the MATLAB object is destroyed.
            % Cleans up the C++ filter instance to prevent memory leaks.
            
            if ~isempty(obj.filter_id)
                try
                    kalman_mex('delete', obj.filter_id);
                catch
                    % Ignore errors during cleanup (MEX might already be unloaded)
                end
                obj.filter_id = [];
            end
        end
        
        function predict(obj)
            % PREDICT Perform the prediction step
            %
            % Projects the current state estimate forward in time based on
            % the system model. In this simple implementation, the state
            % doesn't change but uncertainty increases.
            %
            % Example:
            %   filter.predict();
            
            kalman_mex('predict', obj.filter_id);
        end
        
        function update(obj, measurement)
            % UPDATE Perform the measurement update step
            %
            % Corrects the predicted state using a new measurement.
            % The filter optimally balances the prediction and measurement
            % based on their relative uncertainties.
            %
            % Inputs:
            %   measurement - The new sensor reading
            %
            % Example:
            %   filter.update(5.1);
            
            if nargin ~= 2
                error('KalmanFilter:InvalidInput', 'update() requires a measurement value');
            end
            
            kalman_mex('update', obj.filter_id, measurement);
        end
        
        function state = getState(obj)
            % GETSTATE Get the current state estimate
            %
            % Returns the filter's current best estimate of the true value.
            % This is the filtered result after processing all measurements.
            %
            % Outputs:
            %   state - Current state estimate
            %
            % Example:
            %   current_estimate = filter.getState();
            
            state = kalman_mex('getState', obj.filter_id);
        end
        
        function error_cov = getErrorCovariance(obj)
            % GETERRORCOVARIANCE Get the current uncertainty level
            %
            % Returns the current error covariance, which represents how
            % confident the filter is in its estimate. Lower values mean
            % higher confidence.
            %
            % Outputs:
            %   error_cov - Current error covariance (uncertainty)
            %
            % Example:
            %   uncertainty = filter.getErrorCovariance();
            
            error_cov = kalman_mex('getErrorCovariance', obj.filter_id);
        end
        
        function reset(obj, initial_estimate, initial_error)
            % RESET Reset the filter to new initial conditions
            %
            % Restarts the filter with new initial state and uncertainty.
            % Useful for tracking a different value or changing the starting point.
            %
            % Inputs:
            %   initial_estimate - New initial state estimate
            %   initial_error    - New initial uncertainty
            %
            % Example:
            %   filter.reset(10.0, 2.0);
            
            if nargin ~= 3
                error('KalmanFilter:InvalidInput', ...
                      'reset() requires initial_estimate and initial_error');
            end
            
            kalman_mex('reset', obj.filter_id, initial_estimate, initial_error);
        end
        
        function [states, error_covs] = process(obj, measurements)
            % PROCESS Process multiple measurements at once
            %
            % Efficiently processes an array of measurements and returns
            % the corresponding state estimates and error covariances.
            % This is equivalent to calling predict() and update() for
            % each measurement, but faster for large datasets.
            %
            % Inputs:
            %   measurements - Array of measurement values
            %
            % Outputs:
            %   states      - Array of state estimates (same size as measurements)
            %   error_covs  - Array of error covariances (same size as measurements)
            %
            % Example:
            %   measurements = [4.9, 5.1, 4.8, 5.2];
            %   [estimates, uncertainties] = filter.process(measurements);
            
            if nargin ~= 2
                error('KalmanFilter:InvalidInput', 'process() requires a measurements array');
            end
            
            if ~isnumeric(measurements)
                error('KalmanFilter:InvalidInput', 'measurements must be numeric');
            end
            
            % Convert to column vector if needed
            measurements = measurements(:);
            
            [states, error_covs] = kalman_mex('process', obj.filter_id, measurements);
        end
        
        function step(obj, measurement)
            % STEP Perform a complete filter step (predict + update)
            %
            % Convenience method that performs both prediction and update
            % in a single call. Equivalent to calling predict() followed by update().
            %
            % Inputs:
            %   measurement - The new sensor reading
            %
            % Example:
            %   filter.step(5.1);  % Same as predict() + update(5.1)
            
            if nargin ~= 2
                error('KalmanFilter:InvalidInput', 'step() requires a measurement value');
            end
            
            obj.predict();
            obj.update(measurement);
        end
        
        function result = getCurrentState(obj)
            % GETCURRENTSTATE Get complete current filter state
            %
            % Returns a structure containing both the state estimate and
            % error covariance for convenience.
            %
            % Outputs:
            %   result - Structure with fields:
            %            .state - Current state estimate
            %            .error_covariance - Current uncertainty
            %
            % Example:
            %   status = filter.getCurrentState();
            %   fprintf('State: %.4f, Uncertainty: %.6f\n', ...
            %           status.state, status.error_covariance);
            
            result.state = obj.getState();
            result.error_covariance = obj.getErrorCovariance();
        end
        
        function plotResults(obj, measurements, varargin)
            % PLOTRESULTS Visualize filter performance
            %
            % Creates plots showing the filter's performance on a set of
            % measurements. Useful for debugging and analysis.
            %
            % Inputs:
            %   measurements - Array of measurements to process and plot
            %   varargin    - Optional name-value pairs:
            %                 'TrueValue' - Known true value for comparison
            %                 'Title'     - Custom plot title
            %
            % Example:
            %   measurements = randn(100,1) + 5;  % Noisy measurements around 5
            %   filter.plotResults(measurements, 'TrueValue', 5.0);
            
            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'TrueValue', [], @isnumeric);
            addParameter(p, 'Title', 'Kalman Filter Results', @ischar);
            parse(p, varargin{:});
            
            true_value = p.Results.TrueValue;
            plot_title = p.Results.Title;
            
            % Save current state to restore later
            current_state = obj.getCurrentState();
            
            % Process measurements
            [states, error_covs] = obj.process(measurements);
            
            % Create plots
            figure('Name', plot_title, 'Position', [100, 100, 1000, 600]);
            
            subplot(2, 2, 1);
            plot(1:length(measurements), measurements, 'b.', 'MarkerSize', 8, 'DisplayName', 'Measurements');
            hold on;
            plot(1:length(states), states, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtered');
            if ~isempty(true_value)
                yline(true_value, 'k--', 'LineWidth', 2, 'DisplayName', 'True Value');
            end
            xlabel('Sample');
            ylabel('Value');
            title('Measurements vs Filtered Estimates');
            legend('Location', 'best');
            grid on;
            
            subplot(2, 2, 2);
            semilogy(1:length(error_covs), error_covs, 'g-', 'LineWidth', 2);
            xlabel('Sample');
            ylabel('Error Covariance');
            title('Uncertainty Evolution');
            grid on;
            
            subplot(2, 2, 3);
            if ~isempty(true_value)
                estimation_error = abs(states - true_value);
                semilogy(1:length(estimation_error), estimation_error, 'm-', 'LineWidth', 2);
                ylabel('|Estimate - True Value|');
                title('Estimation Error');
            else
                innovation = abs(measurements - states);
                plot(1:length(innovation), innovation, 'm-', 'LineWidth', 2);
                ylabel('|Measurement - Estimate|');
                title('Innovation (Measurement Residual)');
            end
            xlabel('Sample');
            grid on;
            
            subplot(2, 2, 4);
            scatter(measurements, states, 50, 'b', 'filled', 'Alpha', 0.6);
            hold on;
            if ~isempty(true_value)
                plot([min(measurements), max(measurements)], [min(measurements), max(measurements)], ...
                     'k--', 'LineWidth', 2, 'DisplayName', 'Perfect Estimate');
            end
            xlabel('Measurement');
            ylabel('Filtered Estimate');
            title('Measurement vs Estimate Correlation');
            grid on;
            axis equal;
            
            % Restore original state
            obj.reset(current_state.state, current_state.error_covariance);
        end
    end
end
