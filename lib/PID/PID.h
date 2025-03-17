#ifndef PID_H
#define PID_H

class PID {
public:
  /**
   * Constructor: Initializes the PID controller with the given gains/time constants.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param setpoint Desired target value for the control system.
   * @param tau Time constant for the derivative low-pass filter.
   * @param serial Boolean indicating whether the controller uses serial or parallel form.
   */
  PID(double Kp, double Ki, double Kd, double setpoint, double tau, bool serial);

  /**
   * Set the tuning parameters for the parallel form of the PID controller.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  void setParallelTunings(double Kp, double Ki, double Kd);

  /**
   * Overloaded function: Set the tuning parameters for the parallel form of the PID controller 
   * with additional configuration for integral windup prevention.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   * @param tau Time constant for the derivative low-pass filter.
   * @param integralMin Minimum limit for the integral term.
   * @param integralMax Maximum limit for the integral term.
   */
  void setParallelTunings(double Kp, double Ki, double Kd, double tau, double integralMin, double integralMax);

  /**
   * Set the tuning parameters for the serial form of the PID controller.
   *
   * @param Kp Proportional gain.
   * @param Ti Time constant for the integral term.
   * @param Td Time constant for the derivative term.
   */
  void setSerialTunings(double Kp, double Ti, double Td);

  /**
   * Overloaded function: Set the tuning parameters for the serial form of the PID controller 
   * with additional configuration for integral windup prevention.
   *
   * @param Kp Proportional gain.
   * @param Ti Time constant for the integral term.
   * @param Td Time constant for the derivative term.
   * @param tau Time constant for the derivative low-pass filter.
   * @param integralMin Minimum limit for the integral term.
   * @param integralMax Maximum limit for the integral term.
   */
  void setSerialTunings(double Kp, double Ti, double Td, double tau, double integralMin, double integralMax);

  /**
   * Calculates the output of the parallel PID controller based on the input and setpoint.
   *
   * @param input The current process variable.
   * @param setpoint The desired target value for the control system.
   * @return Output of the PID controller.
   */
  double calculateParallel(double input, double setpoint);

  /**
   * Overloaded function: Calculates the output of the parallel PID controller based on the input, 
   * setpoint, and an externally provided derivative value.
   *
   * @param input The current process variable.
   * @param setpoint The desired target value for the control system.
   * @param derivative External derivative value to be used in the PID calculation.
   * @return Output of the PID controller.
   */
  double calculateParallel(double input, double setpoint, double derivative);

  /**
   * Calculates the output of the serial PID controller based on the input and setpoint.
   *
   * @param input The current process variable.
   * @param setpoint The desired target value for the control system.
   * @return Output of the PID controller.
   */
  double calculateSerial(double input, double setpoint);

  /**
   * Calculates the output of the serial PID controller based on the input and setpoint.
   *
   * @param input The current process variable.
   * @param setpoint The desired target value for the control system.
   * @param derivative External derivative value to be used in the PID calculation.
   * @return Output of the PID controller.
   */
  double calculateSerial(double input, double setpoint, double derivative);

private:
  /**
   * Helper function to handle the overflow of micros() in Arduino.
   *
   * @param currentTime Current timestamp in microseconds.
   * @return Adjusted timestamp to handle the overflow.
   */
  unsigned long handleMicrosRollover(unsigned long currentTime);

  double _Kp, _Ki, _Kd;         // Proportional, Integral, and Derivative gains/time constants
  double _previousError;        // Previous error, used for derivative calculation
  double _integral;             // Integral term accumulation
  double _setpoint;             // Desired setpoint
  double _lastDerivative;      // Last derivative calculation, used for filtering
  double _tau;                  // Tau value for the derivative low-pass filter
  double _alpha;                // Alpha value for the derivative low-pass filter
  double _integralMin, _integralMax; // Limits for integral windup prevention
  unsigned long _previousTime;  // Previous time in microseconds, used to calculate dt
};

#endif // PID_H
