#include "mex.h"
#include "kalman.h"
#include <map>
#include <memory>

/*
 * MATLAB MEX Interface for SimpleKalmanFilter
 * ===========================================
 *
 * This MEX file provides a MATLAB interface to the C++ SimpleKalmanFilter class.
 * It allows MATLAB users to create, manipulate, and use Kalman filters from MATLAB.
 *
 * MATLAB Usage:
 * =============
 *
 * 1. Create a filter:
 *    filter_id = kalman_mex('create', initial_estimate, initial_error, process_noise, measurement_noise);
 *
 * 2. Predict step:
 *    kalman_mex('predict', filter_id);
 *
 * 3. Update step:
 *    kalman_mex('update', filter_id, measurement);
 *
 * 4. Get current state:
 *    state = kalman_mex('getState', filter_id);
 *
 * 5. Get error covariance:
 *    error_cov = kalman_mex('getErrorCovariance', filter_id);
 *
 * 6. Reset filter:
 *    kalman_mex('reset', filter_id, initial_estimate, initial_error);
 *
 * 7. Delete filter:
 *    kalman_mex('delete', filter_id);
 *
 * 8. Process multiple measurements at once:
 *    [states, error_covs] = kalman_mex('process', filter_id, measurements);
 */

// Global storage for filter instances
static std::map<int, std::unique_ptr<SimpleKalmanFilter>> filters;
static int next_filter_id = 1;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check minimum number of arguments
    if (nrhs < 1)
    {
        mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                          "At least one input argument (command) is required.");
    }

    // Get the command string
    if (!mxIsChar(prhs[0]))
    {
        mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                          "First argument must be a command string.");
    }

    char command[64];
    mxGetString(prhs[0], command, sizeof(command));

    // CREATE COMMAND
    if (strcmp(command, "create") == 0)
    {
        if (nrhs != 5)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "create command requires 4 parameters: initial_estimate, initial_error, process_noise, measurement_noise");
        }

        double initial_estimate = mxGetScalar(prhs[1]);
        double initial_error = mxGetScalar(prhs[2]);
        double process_noise = mxGetScalar(prhs[3]);
        double measurement_noise = mxGetScalar(prhs[4]);

        // Create new filter
        int filter_id = next_filter_id++;
        filters[filter_id] = std::make_unique<SimpleKalmanFilter>(
            initial_estimate, initial_error, process_noise, measurement_noise);

        // Return filter ID
        plhs[0] = mxCreateDoubleScalar(filter_id);

        mexPrintf("Created Kalman filter with ID %d\n", filter_id);
        mexPrintf("  Initial estimate: %.4f\n", initial_estimate);
        mexPrintf("  Initial error: %.4f\n", initial_error);
        mexPrintf("  Process noise: %.4f\n", process_noise);
        mexPrintf("  Measurement noise: %.4f\n", measurement_noise);
    }

    // PREDICT COMMAND
    else if (strcmp(command, "predict") == 0)
    {
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "predict command requires filter_id");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        filters[filter_id]->predict();
    }

    // UPDATE COMMAND
    else if (strcmp(command, "update") == 0)
    {
        if (nrhs != 3)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "update command requires filter_id and measurement");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);
        double measurement = mxGetScalar(prhs[2]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        filters[filter_id]->update(measurement);
    }

    // GET STATE COMMAND
    else if (strcmp(command, "getState") == 0)
    {
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "getState command requires filter_id");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        double state = filters[filter_id]->getState();
        plhs[0] = mxCreateDoubleScalar(state);
    }

    // GET ERROR COVARIANCE COMMAND
    else if (strcmp(command, "getErrorCovariance") == 0)
    {
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "getErrorCovariance command requires filter_id");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        double error_cov = filters[filter_id]->getErrorCovariance();
        plhs[0] = mxCreateDoubleScalar(error_cov);
    }

    // RESET COMMAND
    else if (strcmp(command, "reset") == 0)
    {
        if (nrhs != 4)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "reset command requires filter_id, initial_estimate, and initial_error");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);
        double initial_estimate = mxGetScalar(prhs[2]);
        double initial_error = mxGetScalar(prhs[3]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        filters[filter_id]->reset(initial_estimate, initial_error);
    }

    // DELETE COMMAND
    else if (strcmp(command, "delete") == 0)
    {
        if (nrhs != 2)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "delete command requires filter_id");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        filters.erase(filter_id);
        mexPrintf("Deleted Kalman filter with ID %d\n", filter_id);
    }

    // PROCESS COMMAND (batch processing)
    else if (strcmp(command, "process") == 0)
    {
        if (nrhs != 3)
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "process command requires filter_id and measurements array");
        }

        int filter_id = (int)mxGetScalar(prhs[1]);

        if (filters.find(filter_id) == filters.end())
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidID",
                              "Invalid filter ID");
        }

        // Get measurements array
        if (!mxIsDouble(prhs[2]))
        {
            mexErrMsgIdAndTxt("KalmanMEX:InvalidInput",
                              "Measurements must be a double array");
        }

        double *measurements = mxGetPr(prhs[2]);
        int num_measurements = mxGetNumberOfElements(prhs[2]);

        // Allocate output arrays
        plhs[0] = mxCreateDoubleMatrix(num_measurements, 1, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(num_measurements, 1, mxREAL);
        double *states = mxGetPr(plhs[0]);
        double *error_covs = mxGetPr(plhs[1]);

        // Process each measurement
        for (int i = 0; i < num_measurements; i++)
        {
            filters[filter_id]->predict();
            filters[filter_id]->update(measurements[i]);
            states[i] = filters[filter_id]->getState();
            error_covs[i] = filters[filter_id]->getErrorCovariance();
        }
    }

    else
    {
        mexErrMsgIdAndTxt("KalmanMEX:InvalidCommand",
                          "Unknown command. Valid commands: create, predict, update, getState, getErrorCovariance, reset, delete, process");
    }
}
