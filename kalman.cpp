#include "kalman.h"

/*
 * Kalman Filter Implementation
 * ===========================
 *
 * This file implements the core Kalman filter algorithms.
 * The filter operates in a continuous cycle of PREDICT -> UPDATE -> PREDICT -> UPDATE...
 */

// Constructor - Initialize the filter with starting parameters
SimpleKalmanFilter::SimpleKalmanFilter(double initial_estimate, double initial_error,
                                       double process_noise, double measurement_noise)
    : x(initial_estimate), P(initial_error), Q(process_noise), R(measurement_noise), K(0.0)
{
    /*
     * Member Initializer List Setup:
     * x = initial_estimate  -> Our first guess of the true value
     * P = initial_error     -> How uncertain we are about our first guess
     * Q = process_noise     -> How much we expect the system to change
     * R = measurement_noise -> How much noise is in our sensor readings
     * K = 0.0              -> Kalman gain starts at zero (will be calculated)
     */
}

/*
 * PREDICT STEP - Project our current estimate forward in time
 * ==========================================================
 *
 * This is the "time update" or "prediction" step of the Kalman filter.
 * We use our model of how the system behaves to predict where it will be next.
 */
void SimpleKalmanFilter::predict()
{
    /*
     * STATE PREDICTION:
     * In this simple example, we assume a "constant" model - the true value
     * doesn't change over time. So our prediction is:
     * x_predicted = x_current (no change)
     *
     * In more complex filters, this might be:
     * x_predicted = F * x_current + B * control_input
     * where F is a state transition model and B handles control inputs
     */
    // x = x;  // No change in this simple constant model

    /*
     * COVARIANCE PREDICTION:
     * Even though our state estimate doesn't change, our uncertainty DOES increase
     * because real systems have some unpredictable variation (process noise).
     *
     * The longer we go without a measurement, the less confident we become.
     * This is modeled by adding process noise Q to our uncertainty P.
     */
    P = P + Q;

    /*
     * Why add Q to P?
     * - P represents how uncertain we are about our estimate
     * - Q represents how much the system might change unpredictably
     * - After time passes, our uncertainty should increase by Q
     * - This makes the filter more willing to trust the next measurement
     */
}

/*
 * UPDATE STEP - Correct our prediction using a new measurement
 * ===========================================================
 *
 * This is the "measurement update" or "correction" step of the Kalman filter.
 * We optimally combine our prediction with the new measurement.
 */
void SimpleKalmanFilter::update(double measurement)
{
    /*
     * STEP 1: Calculate the Kalman Gain
     * ================================
     * The Kalman gain K determines how much we trust the measurement vs our prediction.
     * It's calculated as: K = P / (P + R)
     *
     * Think of it as a "trust ratio":
     * - If P >> R: Our prediction is very uncertain, measurement is reliable -> K ≈ 1 (trust measurement)
     * - If P << R: Our prediction is confident, measurement is noisy -> K ≈ 0 (trust prediction)
     * - If P ≈ R: Both are equally reliable -> K ≈ 0.5 (trust both equally)
     */
    K = P / (P + R);

    /*
     * STEP 2: Update the State Estimate
     * ================================
     * We blend our prediction with the measurement using the Kalman gain.
     *
     * Formula: x_new = x_old + K * (measurement - x_old)
     *
     * Breaking this down:
     * - (measurement - x_old) is the "innovation" or "residual" - how much the measurement differs from our prediction
     * - K * (innovation) is how much we adjust our estimate based on this difference
     * - If K=0: x_new = x_old (ignore measurement completely)
     * - If K=1: x_new = measurement (ignore prediction completely)
     * - If K=0.5: x_new = average of prediction and measurement
     */
    x = x + K * (measurement - x);

    /*
     * STEP 3: Update the Error Covariance
     * ==================================
     * After incorporating a measurement, we should be MORE confident (lower uncertainty).
     *
     * Formula: P_new = (1 - K) * P_old
     *
     * Why does this work?
     * - If K=0 (ignored measurement): P_new = P_old (no change in confidence)
     * - If K=1 (fully trusted measurement): P_new = 0 (completely confident)
     * - If K=0.5: P_new = 0.5 * P_old (confidence improves by half)
     *
     * This ensures P always decreases after a measurement (we get more confident).
     */
    P = (1.0 - K) * P;
}

// Getter function: Return our current best estimate of the true value
double SimpleKalmanFilter::getState() const
{
    /*
     * This is the "filtered" result - our best estimate after combining
     * all previous measurements optimally. This is usually much better
     * than any individual noisy measurement.
     */
    return x;
}

// Getter function: Return our current confidence level
double SimpleKalmanFilter::getErrorCovariance() const
{
    /*
     * P represents our uncertainty/confidence:
     * - Lower P = more confident in our estimate
     * - Higher P = less confident in our estimate
     *
     * P typically:
     * - Increases during predict() steps (uncertainty grows over time)
     * - Decreases during update() steps (measurements improve confidence)
     */
    return P;
}

// Reset function: Start over with new initial conditions
void SimpleKalmanFilter::reset(double initial_estimate, double initial_error)
{
    /*
     * This allows restarting the filter with new parameters.
     * Useful if:
     * - You want to track a different value
     * - You want to change your initial guess
     * - The system has changed significantly
     */
    x = initial_estimate; // New starting estimate
    P = initial_error;    // New starting uncertainty
    K = 0.0;              // Reset Kalman gain
    // Note: Q and R are not reset - they're system properties
}
