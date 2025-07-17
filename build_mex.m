function build_mex()
    % BUILD_MEX - Compile the Kalman filter MEX file
    %
    % This script compiles the C++ Kalman filter into a MEX file that can be
    % called from MATLAB. It handles the compilation process and provides
    % helpful error messages if compilation fails.
    
    fprintf('Building Kalman Filter MEX file...\n');
    fprintf('==================================\n');
    
    % Check if we're in the right directory
    if ~exist('kalman.cpp', 'file') || ~exist('kalman.h', 'file')
        error('kalman.cpp and kalman.h must be in the current directory');
    end
    
    if ~exist('kalman_mex.cpp', 'file')
        error('kalman_mex.cpp must be in the current directory');
    end
    
    try
        % Compile the MEX file
        % Include both the MEX interface and the Kalman filter implementation
        fprintf('Compiling MEX file...\n');
        
        % For Windows
        if ispc
            mex('-v', 'kalman_mex.cpp', 'kalman.cpp', '-std=c++17');
        % For macOS and Linux
        else
            mex('-v', 'kalman_mex.cpp', 'kalman.cpp', 'CXXFLAGS="$CXXFLAGS -std=c++17"');
        end
        
        fprintf('\n✓ MEX file compiled successfully!\n');
        
        % Test basic functionality
        fprintf('Testing MEX file...\n');
        
        % Create a test filter
        filter_id = kalman_mex('create', 0.0, 1.0, 0.01, 0.1);
        
        % Test basic operations
        kalman_mex('predict', filter_id);
        kalman_mex('update', filter_id, 5.0);
        state = kalman_mex('getState', filter_id);
        error_cov = kalman_mex('getErrorCovariance', filter_id);
        
        fprintf('✓ Basic test passed!\n');
        fprintf('  Test state: %.4f\n', state);
        fprintf('  Test error covariance: %.6f\n', error_cov);
        
        % Clean up
        kalman_mex('delete', filter_id);
        
        fprintf('\n✓ MEX file is ready to use!\n');
        fprintf('Run kalman_demo() to see a complete demonstration.\n');
        
    catch ME
        fprintf('\n✗ Compilation failed!\n');
        fprintf('Error: %s\n', ME.message);
        
        % Provide helpful troubleshooting information
        fprintf('\nTroubleshooting:\n');
        fprintf('================\n');
        
        if ispc
            fprintf('Windows users:\n');
            fprintf('• Make sure you have Visual Studio or MinGW installed\n');
            fprintf('• Run "mex -setup" to configure your compiler\n');
            fprintf('• Ensure C++17 support is available\n');
        elseif ismac
            fprintf('macOS users:\n');
            fprintf('• Make sure Xcode Command Line Tools are installed:\n');
            fprintf('  xcode-select --install\n');
            fprintf('• Check that your compiler supports C++17\n');
            fprintf('• You may need to update macOS or Xcode\n');
        else
            fprintf('Linux users:\n');
            fprintf('• Make sure g++ is installed and supports C++17:\n');
            fprintf('  g++ --version\n');
            fprintf('• Install build essentials if needed:\n');
            fprintf('  sudo apt-get install build-essential (Ubuntu/Debian)\n');
            fprintf('  sudo yum groupinstall "Development Tools" (CentOS/RHEL)\n');
        end
        
        fprintf('\nGeneral tips:\n');
        fprintf('• Verify MATLAB version compatibility (R2017b+ recommended for C++17)\n');
        fprintf('• Check that all source files are in the current directory\n');
        fprintf('• Try running "mex -setup" to reconfigure your compiler\n');
        
        rethrow(ME);
    end
end
