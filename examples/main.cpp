#include <iostream>
#include "utils.h"
#include "kalman.h"

int main()
{
    std::cout << "Simple C++ Program with Utils" << std::endl;
    std::cout << "==============================" << std::endl;

    // Test the add function
    int sum = add(5, 3);
    std::cout << "5 + 3 = " << sum << std::endl;

    // Test the multiply function
    double product = multiply(2.5, 4.0);
    std::cout << "2.5 * 4.0 = " << product << std::endl;

    // Test the greet function
    std::string message = greet("World");
    std::cout << message << std::endl;

    // Test the printArray function
    int numbers[] = {1, 2, 3, 4, 5};
    int size = sizeof(numbers) / sizeof(numbers[0]);
    printArray(numbers, size);

    std::cout << "\nKalman Filter Demo" << std::endl;
    std::cout << "==================" << std::endl;

    /*
     * KALMAN FILTER DEMONSTRATION
     * ===========================
     *
     * Scenario: We have a sensor that should read a constant value of 5.0,
     * but the sensor has noise, so readings vary around the true value.
     *
     * Goal: Use a Kalman filter to get a better estimate of the true value
     * by filtering out the noise from multiple measurements.
     *
     * The Kalman filter will:
     * 1. Start with an initial guess (we'll start at 0.0 - deliberately wrong)
     * 2. Process each noisy measurement
     * 3. Gradually converge to the true value (5.0)
     * 4. Become more confident over time (lower uncertainty)
     */

    // Create a simple Kalman filter
    // Parameters: initial_estimate, initial_error, process_noise, measurement_noise
    /*
     * PARAMETER EXPLANATION:
     *
     * initial_estimate = 0.0
     *   Our first guess of the true value. We deliberately start wrong to show convergence.
     *
     * initial_error = 1.0
     *   How uncertain we are about our initial guess. Since we started at 0 but expect
     *   the true value to be around 5, an error of 1.0 means we think the true value
     *   is likely between -1 and +1 initially (which is wrong, but the filter will learn).
     *
     * process_noise = 0.01
     *   How much we expect the true value to change between measurements.
     *   Since we're measuring a constant value, this should be small.
     *   Small Q means "the true value is very stable."
     *
     * measurement_noise = 0.1
     *   How noisy our sensor is. Looking at our measurements (4.8 to 5.2),
     *   they vary by about ±0.2 around the true value, so R=0.1 is reasonable.
     *   This tells the filter "measurements typically have noise of about 0.1"
     */
    SimpleKalmanFilter filter(0.0, 1.0, 0.01, 0.1);

    // Simulate some noisy measurements around true value of 5.0
    /*
     * SIMULATED SENSOR DATA:
     * These represent what a real noisy sensor might output when measuring
     * a constant value of 5.0. Notice how they're all "close" to 5.0 but
     * have random noise that makes them vary between 4.8 and 5.2.
     *
     * In a real application, these would come from actual sensor readings.
     */
    double measurements[] = {4.9, 5.1, 4.8, 5.2, 4.95, 5.05, 4.85, 5.15};
    int num_measurements = sizeof(measurements) / sizeof(measurements[0]);

    std::cout << "True value: 5.0" << std::endl;
    std::cout << "Measurement -> Filtered Estimate" << std::endl;

    /*
     * PROCESSING LOOP:
     * For each measurement, we perform the two-step Kalman filter cycle:
     * 1. PREDICT: Project our current estimate forward (predict where we think the value is)
     * 2. UPDATE: Correct our prediction using the new measurement
     *
     * Watch how the filtered estimate gradually gets closer to 5.0!
     */
    for (int i = 0; i < num_measurements; i++)
    {
        /*
         * PREDICT STEP:
         * "Based on our current knowledge, where do we think the value is now?"
         * In our simple model, we assume the value doesn't change, but our
         * uncertainty increases slightly due to process noise.
         */
        filter.predict();

        /*
         * UPDATE STEP:
         * "We got a new measurement! Let's optimally combine it with our prediction."
         * The filter automatically balances trust between prediction and measurement
         * based on their relative uncertainties.
         */
        filter.update(measurements[i]);

        /*
         * OBSERVE THE RESULTS:
         * Print both the raw measurement and the filtered estimate.
         * Notice how the filtered estimate is usually "better" (closer to 5.0)
         * than the individual noisy measurements!
         */
        std::cout << measurements[i] << " -> " << filter.getState() << std::endl;
    }

    /*
     * FINAL RESULTS:
     * After processing all measurements, let's see how well we did!
     */
    std::cout << "Final estimate: " << filter.getState() << std::endl;
    std::cout << "Final uncertainty: " << filter.getErrorCovariance() << std::endl;

    /*
     * WHAT TO OBSERVE:
     *
     * 1. CONVERGENCE: The final estimate should be very close to 5.0 (true value)
     *    even though we started at 0.0 and had noisy measurements.
     *
     * 2. UNCERTAINTY REDUCTION: The final uncertainty should be much lower than
     *    our initial uncertainty (1.0), showing the filter gained confidence.
     *
     * 3. NOISE FILTERING: The filtered estimates are typically closer to 5.0
     *    than the individual noisy measurements.
     *
     * 4. OPTIMAL BLENDING: Early measurements have more influence (big corrections)
     *    because our initial uncertainty was high. Later measurements have less
     *    influence because we've become more confident in our estimate.
     *
     * This demonstrates the key benefit of Kalman filtering: optimal estimation
     * under uncertainty by balancing predictions and measurements!
     */

    return 0;
}
