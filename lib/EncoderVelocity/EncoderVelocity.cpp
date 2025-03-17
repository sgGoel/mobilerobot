#include "EncoderVelocity.h"

/**
 * @brief Initializes the encoder and conversion factor from counts to radians
 * @param pinA Pin number for encoder signal A
 * @param pinB Pin number for encoder signal B
 * @param countsPerRevolution Number of counts for one full revolution of the encoder
 * @param tau Time constant for the low-pass filter, specified in seconds
 * @param minimumVelocity Minimum velocity to be considered moving, specified in radians per second
 */
EncoderVelocity::EncoderVelocity(int pinA, int pinB, int countsPerRevolution, float tau, float minimumVelocity)
  : _moving(false), _lastPosition(0), _lastChange(0), _velocity(0), _lastCheck(0), _filteredVelocity(0), _tau(tau), _minimumVelocity(minimumVelocity),
    _countsToRadians(2 * PI / countsPerRevolution){
    ESP32Encoder::useInternalWeakPullResistors = UP;
  // _countsToRadians converts from encoder counts to radians
    _encoder.attachFullQuad(pinA, pinB);
    _encoder.setCount(0);
    _lastChange = micros();
    _lastCheck = micros();
    _timeout = 10000;
}

// Calculates and returns the current velocity in radians per second
float EncoderVelocity::getVelocity() {
  // Read the current encoder position
  long currentPosition = _encoder.getCount();
  // Get the current time in microseconds
  unsigned long currentTime = micros();
  // Calculate the time difference, handling potential rollover of micros()
  unsigned long changeDt = (currentTime >= _lastChange) ? (currentTime - _lastChange) : (currentTime + (0xFFFFFFFF - _lastChange));
  //time since the velocity was last checked
  unsigned long checkDt = (currentTime >= _lastCheck) ? (currentTime - _lastCheck) : (currentTime + (0xFFFFFFFF - _lastCheck));
  


     // Check if the encoder position has changed and changeDt is not zero
  if (currentPosition != _lastPosition && changeDt > 0) {
    // Calculate the velocity as the change in position (in radians) divided by the change in time (in seconds)
    _velocity = (float)(currentPosition - _lastPosition) * _countsToRadians / changeDt * 1000000;
    // Update the last position and time
    _lastPosition = currentPosition;
    _lastChange = currentTime;
    //if it's taken way too long to get a reading, reset the velocity to zero
    //only reset if the encoder is moving
  } else if (changeDt > _timeout ) {
    _velocity = 0 ;
  }

  // Calculate the filtered velocity using a first-order low-pass filter
  float alpha = (float)checkDt / (_tau * 1000000.0 + checkDt);
  _filteredVelocity = _filteredVelocity + alpha * (_velocity - _filteredVelocity);

  //update the last time the velocity was checked
  _lastCheck = currentTime;
  // Return the velocity in radians per second
  return _filteredVelocity;
}

// Returns the current position of the encoder in radians
float EncoderVelocity::getPosition() {
  // Read the current encoder position (in counts) and convert it to radians
  return _encoder.getCount() * _countsToRadians;
}

// Resets the position readings of the encoder
void EncoderVelocity::resetPosition() {
  // Reset the encoder's position reading to zero (in counts)
  _encoder.setCount(0);

  // Reset internal tracking of the last position (in counts) and time (in microseconds)
  _lastPosition = 0;
  _lastChange = micros();
}
