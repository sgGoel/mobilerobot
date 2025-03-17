#include <Arduino.h>
#include "PID.h"

  // Constructor: Initializes the PID controller with the given parameters
  PID::PID(double Kp, double Ki, double Kd, double setpoint, double tau, bool serial)
    : _setpoint(setpoint), _previousError(0), _integral(0), _lastDerivative(0), _alpha(1), _integralMin(0), _integralMax(0), _tau(tau) {
      if (serial) {
        PID::setSerialTunings(Kp, Ki, Kd);
      } else {
        PID::setParallelTunings(Kp, Ki, Kd);
      }
    _previousTime = micros(); // Store the current time in microseconds
  }


   //Set tuning parameters for parallel PID form
  void PID::setParallelTunings(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
  }

  // Set tuning parameters for parallel PID form
  void PID::setParallelTunings(double Kp, double Ki, double Kd, double tau, double integralMin, double integralMax) {
    PID::setParallelTunings(Kp, Ki, Kd);
    _tau = tau;
    _integralMin = integralMin;
    _integralMax = integralMax;
  }
  // Set tuning parameters for serial PID form
  void PID::setSerialTunings(double Kp, double Ti, double Td) {
    _Kp = Kp;
    _Ki = 1.0 / Ti; // Integral time constant
    _Kd = Td;       // Derivative time constant
  }

  // Set tuning parameters for serial PID form
  void PID::setSerialTunings(double Kp, double Ti, double Td, double tau, double integralMin, double integralMax) {
    PID::setSerialTunings(Kp, Ti, Td);
    _tau = tau;
    _integralMin = integralMin;
    _integralMax = integralMax;
  }

  // Parallel form PID calculation
  double PID::calculateParallel(double input, double setpoint) {
    _setpoint = setpoint;
    unsigned long currentTime = micros();
    unsigned long timeDifference = handleMicrosRollover(currentTime);
    double dt = (double)timeDifference / 1000000.0; // Time difference in seconds
    _alpha = exp(-dt / _tau); // Calculate alpha for low-pass filter

    double error = _setpoint - input;               // Calculate error
    _integral += error * dt;                        // Integral term
    _integral = constrain(_integral, _integralMin, _integralMax); // Prevent integral windup

    double derivative = (error - _previousError) / dt; // Derivative term
    
    derivative = _alpha * _lastDerivative + (1 - _alpha) * derivative; // Apply low-pass filter
    _lastDerivative = derivative;

    // double output = _Kp * (error + _Ki * _integral + _Kd * derivative); // Calculate pseudo-parallel PID output
    double output = _Kp * error + _Ki * _integral + _Kd * derivative; // Calculate true parallel PID output
    _previousError = error;

    return output;
  }

  double PID::calculateParallel(double input, double setpoint, double derivative) {
    _setpoint = setpoint;
    unsigned long currentTime = micros();
    unsigned long timeDifference = handleMicrosRollover(currentTime);
    double dt = (double)timeDifference / 1000000.0; // Time difference in seconds
    _alpha = exp(-dt / _tau); // Calculate alpha for low-pass filter

    double error = _setpoint - input;               // Calculate error
    _integral += error * dt;                        // Integral term
    _integral = constrain(_integral, _integralMin, _integralMax); // Prevent integral windup

    // double output = _Kp * (error + _Ki * _integral + _Kd * derivative); // Calculate pseudo-parallel PID output
    
    double output = _Kp * error + _Ki * _integral + _Kd * derivative; // Calculate true parallel PID output
    _previousError = error;

    return output;
  }
  

  // Serial form PID calculation
  double PID::calculateSerial(double input, double setpoint) {
    _setpoint = setpoint;
    unsigned long currentTime = micros();
    unsigned long timeDifference = handleMicrosRollover(currentTime);
    double dt = (double)timeDifference / 1000000.0; // Time difference in seconds
    _alpha = exp(-dt / _tau); // Calculate alpha for low-pass filter

    double error = _setpoint - input;               // Calculate error
    _integral += (1.0 / _Ki) * error * dt;          // Integral term
    _integral = constrain(_integral, _integralMin, _integralMax); // Prevent integral windup
  
    double derivative = ((_integral + error) - _previousError) / dt; // Derivative term
    derivative = _alpha * _lastDerivative + (1 - _alpha) * derivative; // Apply low-pass filter
    _lastDerivative = derivative;
    
    double output = _Kp * (error + _integral) * _Kd * derivative; // Calculate serial PID output
    _previousError = _integral + error;

    return output;
  }

  // Serial form PID calculation
  double PID::calculateSerial(double input, double setpoint, double derivative) {
    _setpoint = setpoint;
    unsigned long currentTime = micros();
    unsigned long timeDifference = handleMicrosRollover(currentTime);
    double dt = (double)timeDifference / 1000000.0; // Time difference in seconds
    _alpha = exp(-dt / _tau); // Calculate alpha for low-pass filter

    double error = _setpoint - input;               // Calculate error
    _integral += (1.0 / _Ki) * error * dt;          // Integral term
    _integral = constrain(_integral, _integralMin, _integralMax); // Prevent integral windup
    
    double output = _Kp * (error + _integral) * _Kd * derivative; // Calculate serial PID output
    _previousError = _integral + error;

    return output;
  }

  // Method to handle micros() rollover
  unsigned long PID::handleMicrosRollover(unsigned long currentTime) {
    unsigned long timeDifference;
    if (currentTime < _previousTime) { // Rollover has occurred
      timeDifference = (ULONG_MAX - _previousTime) + currentTime; // Correct time difference
    } else {
      timeDifference = currentTime - _previousTime;
    }
    _previousTime = currentTime;
    return timeDifference;
  }


