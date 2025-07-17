function setup_kalman()
    % SETUP_KALMAN - Initial setup for the Kalman Filter project
    %
    % This script sets up the MATLAB environment for using the Kalman filter:
    % 1. Adds necessary directories to MATLAB path
    % 2. Compiles the MEX interface
    % 3. Runs basic tests
    % 4. Provides usage instructions
    
    fprintf('Kalman Filter Project Setup\n');
    fprintf('===========================\n\n');
    
    try
        % Get project root directory
        setup_dir = fileparts(mfilename('fullpath'));
        project_root = setup_dir;  % This script should be in the root
        
        % Define directory paths
        matlab_dir = fullfile(project_root, 'matlab');
        examples_dir = fullfile(project_root, 'examples');
        docs_dir = fullfile(project_root, 'docs');
        src_dir = fullfile(project_root, 'src');
        
        % Check directory structure
        fprintf('1. Checking project structure...\n');
        required_dirs = {matlab_dir, examples_dir, src_dir};
        for i = 1:length(required_dirs)
            if ~exist(required_dirs{i}, 'dir')
                error('Required directory not found: %s', required_dirs{i});
            end
        end
        fprintf('   ✓ Project structure verified\n');
        
        % Add MATLAB directory to path
        fprintf('2. Setting up MATLAB path...\n');
        addpath(matlab_dir);
        fprintf('   ✓ Added matlab directory to path\n');
        
        % Compile MEX interface
        fprintf('3. Compiling MEX interface...\n');
        current_dir = pwd;
        try
            cd(matlab_dir);
            build_mex();
            fprintf('   ✓ MEX interface compiled successfully\n');
        catch ME
            cd(current_dir);
            fprintf('   ✗ MEX compilation failed: %s\n', ME.message);
            rethrow(ME);
        end
        cd(current_dir);
        
        % Run basic tests
        fprintf('4. Running basic tests...\n');
        try
            cd(matlab_dir);
            test_kalman_interface();
            fprintf('   ✓ Basic tests passed\n');
        catch ME
            cd(current_dir);
            fprintf('   ✗ Tests failed: %s\n', ME.message);
            rethrow(ME);
        end
        cd(current_dir);
        
        % Success message
        fprintf('\n✓ Setup completed successfully!\n\n');
        
        % Usage instructions
        fprintf('Usage Instructions:\n');
        fprintf('==================\n');
        fprintf('1. Run the demo:\n');
        fprintf('   >> cd examples\n');
        fprintf('   >> kalman_demo()\n\n');
        
        fprintf('2. Use the object-oriented interface:\n');
        fprintf('   >> filter = KalmanFilter(0.0, 1.0, 0.01, 0.1);\n');
        fprintf('   >> measurements = [4.9, 5.1, 4.8, 5.2];\n');
        fprintf('   >> [estimates, uncertainties] = filter.process(measurements);\n\n');
        
        fprintf('3. Read the documentation:\n');
        if exist(fullfile(docs_dir, 'MATLAB_README.md'), 'file')
            fprintf('   >> edit(''%s'')\n\n', fullfile(docs_dir, 'MATLAB_README.md'));
        else
            fprintf('   Documentation not found in docs directory\n\n');
        end
        
        fprintf('4. Test the interface:\n');
        fprintf('   >> cd matlab\n');
        fprintf('   >> test_kalman_interface()\n\n');
        
        % Save path for future sessions
        fprintf('Note: To permanently add the matlab directory to your path,\n');
        fprintf('      use: addpath(''%s''); savepath;\n\n', matlab_dir);
        
    catch ME
        fprintf('\n✗ Setup failed!\n');
        fprintf('Error: %s\n\n', ME.message);
        
        fprintf('Troubleshooting:\n');
        fprintf('===============\n');
        fprintf('• Make sure you''re running this from the project root directory\n');
        fprintf('• Verify that all source files are present\n');
        fprintf('• Check that your MEX compiler is configured: mex -setup cpp\n');
        fprintf('• For detailed MEX troubleshooting, see docs/MATLAB_README.md\n');
        
        rethrow(ME);
    end
end
