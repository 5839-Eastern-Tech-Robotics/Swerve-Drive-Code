#pragma once

#include "robot/utils/util.hpp"
#include "units/time.h"

namespace libmavnetics {

class PID {
public:
  /**
   * Allocates a PIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp     The proportional coefficient. Must be >= 0.
   * @param Ki     The integral coefficient. Must be >= 0.
   * @param Kd     The derivative coefficient. Must be >= 0.
   * @param period The period between controller updates in seconds. The
   *               default is 20 milliseconds. Must be positive.
   */
  constexpr PID(double Kp, double Ki, double Kd, units::second_t period = 20_ms)
      : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_period(period) {
    if (Kp < 0.0 || Ki < 0.0 || Kd < 0.0) {
      m_Kp = 0.0;
      m_Ki = 0.0;
      m_Kd = 0.0;
    }

    if (period <= 0_s)
      m_period = 20_ms;
  }

  constexpr ~PID() = default;
  constexpr PID(const PID &) = default;
  constexpr PID &operator=(const PID &) = default;
  constexpr PID(PID &&) = default;
  constexpr PID &operator=(PID &&) = default;

  /**
   * Sets the PID Controller gain parameters.
   *
   * Sets the proportional, integral, and differential coefficients.
   *
   * @param Kp The proportional coefficient. Must be >= 0.
   * @param Ki The integral coefficient. Must be >= 0.
   * @param Kd The differential coefficient. Must be >= 0.
   */
  constexpr void setGains(double Kp, double Ki, double Kd) {
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
  }

  /**
   * Sets the proportional coefficient of the PID controller gain.
   *
   * @param Kp The proportional coefficient. Must be >= 0.
   */
  constexpr void setP(double Kp) { m_Kp = Kp; }

  /**
   * Sets the integral coefficient of the PID controller gain.
   *
   * @param Ki The integral coefficient. Must be >= 0.
   */
  constexpr void setI(double Ki) { m_Ki = Ki; }

  /**
   * Sets the differential coefficient of the PID controller gain.
   *
   * @param Kd The differential coefficient. Must be >= 0.
   */
  constexpr void setD(double Kd) { m_Kd = Kd; }

  /**
   * Sets the IZone range. When the absolute value of the position error is
   * greater than IZone, the total accumulated error will reset to zero,
   * disabling integral gain until the absolute value of the position error is
   * less than IZone. This is used to prevent integral windup. Must be
   * non-negative. Passing a value of zero will effectively disable integral
   * gain. Passing a value of infinity disables IZone functionality.
   *
   * @param iZone Maximum magnitude of error to allow integral control. Must be
   *   >= 0.
   */
  constexpr void setIZone(double iZone) { m_iZone = iZone; }

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  constexpr double getP() const { return m_Kp; }

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  constexpr double getI() const { return m_Ki; }

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  constexpr double getD() const { return m_Kd; }

  /**
   * Get the IZone range.
   *
   * @return Maximum magnitude of error to allow integral control.
   */
  constexpr double getIZone() const { return m_iZone; }

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  constexpr units::second_t getPeriod() const { return m_period; }

  /**
   * Gets the error tolerance of this controller. Defaults to 0.05.
   *
   * @return The error tolerance of the controller.
   */
  constexpr double getErrorTolerance() const { return m_errorTolerance; }

  /**
   * Gets the error derivative tolerance of this controller. Defaults to ∞.
   *
   * @return The error derivative tolerance of the controller.
   */
  constexpr double getErrorDerivativeTolerance() const {
    return m_errorDerivativeTolerance;
  }

  /**
   * Gets the accumulated error used in the integral calculation of this
   * controller.
   *
   * @return The accumulated error of this controller.
   */
  constexpr double getAccumulatedError() const { return m_totalError; }

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  constexpr void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    m_haveSetpoint = true;

    if (m_continuous) {
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      m_error =
          inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      m_error = m_setpoint - m_measurement;
    }

    m_errorDerivative = (m_error - m_prevError) / m_period.value();
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  constexpr double getSetpoint() const { return m_setpoint; }

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   * The error tolerance defauls to 0.05, and the error derivative tolerance
   * defaults to ∞.
   *
   * This will return false until at least one input value has been computed.
   */
  constexpr bool atSetpoint() const {
    return m_haveMeasurement && m_haveSetpoint &&
           std::fabs(m_error) < m_errorTolerance &&
           std::fabs(m_errorDerivative) < m_errorDerivativeTolerance;
  }

  /**
   * Enables continuous input.
   *
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route
   * to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  constexpr void enableContinuousInput(double minimumInput,
                                       double maximumInput) {
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  /**
   * Disables continuous input.
   */
  constexpr void disableContinuousInput() { m_continuous = false; }

  /**
   * Returns true if continuous input is enabled.
   */
  constexpr bool isContinuousInputEnabled() const { return m_continuous; }

  /**
   * Sets the minimum and maximum contributions of the integral term.
   *
   * The internal integrator is clamped so that the integral term's contribution
   * to the output stays between minimumIntegral and maximumIntegral. This
   * prevents integral windup.
   *
   * @param minimumIntegral The minimum contribution of the integral term.
   * @param maximumIntegral The maximum contribution of the integral term.
   */
  constexpr void setIntegratorRange(double minimumIntegral,
                                    double maximumIntegral) {
    m_minimumIntegral = minimumIntegral;
    m_maximumIntegral = maximumIntegral;
  }

  /**
   * Sets the error which is considered tolerable for use with AtSetpoint().
   *
   * @param errorTolerance error which is tolerable.
   * @param errorDerivativeTolerance error derivative which is tolerable.
   */
  constexpr void setTolerance(double errorTolerance,
                              double errorDerivativeTolerance =
                                  std::numeric_limits<double>::infinity()) {
    m_errorTolerance = errorTolerance;
    m_errorDerivativeTolerance = errorDerivativeTolerance;
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   */
  constexpr double getError() const { return m_error; }

  /**
   * Returns the error derivative.
   */
  constexpr double getErrorDerivative() const { return m_errorDerivative; }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  constexpr double calculate(double measurement) {
    m_measurement = measurement;
    m_prevError = m_error;
    m_haveMeasurement = true;

    if (m_continuous) {
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      m_error =
          inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    } else {
      m_error = m_setpoint - m_measurement;
    }

    m_errorDerivative = (m_error - m_prevError) / m_period.value();

    // If the absolute value of the position error is outside of IZone, reset
    // the total error
    if (std::fabs(m_error) > m_iZone) {
      m_totalError = 0;
    } else if (m_Ki != 0) {
      m_totalError =
          std::clamp(m_totalError + m_error * m_period.value(),
                     m_minimumIntegral / m_Ki, m_maximumIntegral / m_Ki);
    }

    return m_Kp * m_error + m_Ki * m_totalError + m_Kd * m_errorDerivative;
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   */
  constexpr double calculate(double measurement, double setpoint) {
    m_setpoint = setpoint;
    m_haveSetpoint = true;
    return calculate(measurement);
  }

  /**
   * Reset the previous error, the integral term, and disable the controller.
   */
  constexpr void reset() {
    m_error = 0;
    m_prevError = 0;
    m_totalError = 0;
    m_errorDerivative = 0;
    m_haveMeasurement = false;
  }

private:
  // Factor for "proportional" control
  double m_Kp;

  // Factor for "integral" control
  double m_Ki;

  // Factor for "derivative" control
  double m_Kd;

  // The error range where "integral" control applies
  double m_iZone = std::numeric_limits<double>::infinity();

  // The period (in seconds) of the control loop running this controller
  units::second_t m_period;

  double m_maximumIntegral = 1.0;

  double m_minimumIntegral = -1.0;

  double m_maximumInput = 0;

  double m_minimumInput = 0;

  // Do the endpoints wrap around? eg. Absolute encoder
  bool m_continuous = false;

  // The error at the time of the most recent call to Calculate()
  double m_error = 0;
  double m_errorDerivative = 0;

  // The error at the time of the second-most-recent call to Calculate() (used
  // to compute velocity)
  double m_prevError = 0;

  // The sum of the errors for use in the integral calc
  double m_totalError = 0;

  // The error that is considered at setpoint.
  double m_errorTolerance = 0.05;
  double m_errorDerivativeTolerance = std::numeric_limits<double>::infinity();

  double m_setpoint = 0;
  double m_measurement = 0;

  bool m_haveSetpoint = false;
  bool m_haveMeasurement = false;
};

} // namespace libmavnetics
