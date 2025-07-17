#ifndef KALMAN_H
#define KALMAN_H

/*
 * Simple 1D Kalman Filter Implementation
 * =====================================
 *
 * This is a basic Kalman filter for tracking a single scalar value (1D state).
 * The Kalman filter is an optimal estimator that combines:
 * 1. A prediction of where we think the state should be
 * 2. A noisy measurement of where we observe the state to be
 *
 * The filter works in two steps:
 * 1. PREDICT: Estimate the next state based on our model
 * 2. UPDATE: Correct our estimate using the measurement
 *
 * Key Concept: The filter balances trust between our prediction and measurement
 * based on their respective uncertainties (noise levels).
 */

class SimpleKalmanFilter
{
private:
    // STATE VARIABLES
    double x; // State estimate - our best guess of the true value we're tracking
    double P; // Estimation error covariance - how uncertain we are about our estimate
              // Lower P = more confident, Higher P = less confident

    // NOISE PARAMETERS (tuning parameters)
    double Q; // Process noise covariance - how much we expect the true value to change
              // between predictions. Higher Q = expect more change/volatility
    double R; // Measurement noise covariance - how noisy/unreliable our measurements are
              // Higher R = less trust in measurements, Lower R = more trust in measurements

    // COMPUTED VALUES
    double K; // Kalman gain - determines how much we trust measurement vs prediction
              // K close to 0 = trust prediction more
              // K close to 1 = trust measurement more
              // K is automatically calculated based on P and R

public:
    /*
     * Constructor: Initialize the Kalman filter
     *
     * @param initial_estimate: Our first guess of the true value
     * @param initial_error: How uncertain we are about our initial guess
     * @param process_noise: How much we expect the true value to change over time
     * @param measurement_noise: How noisy/unreliable our sensor measurements are
     */
    SimpleKalmanFilter(double initial_estimate, double initial_error,
                       double process_noise, double measurement_noise);

    /*
     * PREDICT STEP: Project our state estimate forward in time
     *
     * In this simple implementation, we assume the true value doesn't change
     * (constant model), so we only increase our uncertainty due to process noise.
     *
     * What happens:
     * - State estimate x stays the same (no motion model)
     * - Uncertainty P increases by adding process noise Q
     */
    void predict();

    /*
     * UPDATE STEP: Correct our prediction using a new measurement
     *
     * This is where the "magic" happens - we optimally combine our prediction
     * with the measurement based on their relative uncertainties.
     *
     * What happens:
     * 1. Calculate Kalman gain K based on prediction uncertainty vs measurement noise
     * 2. Update state estimate x by blending prediction and measurement
     * 3. Reduce uncertainty P (we're more confident after incorporating measurement)
     *
     * @param measurement: The new sensor reading/observation
     */
    void update(double measurement);

    /*
     * Get the current best estimate of the true value
     * This is our filtered result after processing all measurements so far.
     */
    double getState() const;

    /*
     * Get the current uncertainty level
     * Lower values mean we're more confident in our estimate.
     * This typically decreases as we process more measurements.
     */
    double getErrorCovariance() const;

    /*
     * Reset the filter to start fresh with new initial conditions
     * Useful if you want to restart tracking or change initial parameters.
     */
    void reset(double initial_estimate, double initial_error);
};

#endif // KALMAN_H
