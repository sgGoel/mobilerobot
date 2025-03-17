#ifndef EncoderVelocity_h
#define EncoderVelocity_h

#include <Arduino.h>
#include <ESP32Encoder.h>
//counts for gobilda motors
//https://www.gobilda.com/yellow-jacket-planetary-gear-motors/
// rounded to nearest int
#define CPR_30_RPM 5282 // 5281.1 
#define CPR_60_RPM 2786 // 2786.2
#define CPR_312_RPM 538 // 537.7
class EncoderVelocity {
public:
  // Constructor: Initializes with the given pins and counts per revolution (CPR)
  EncoderVelocity(int pinA, int pinB, int countsPerRevolution, float tau = 0.01, float minimumVelocity = 0.01);

  // Returns the current velocity in radians per second
  float getVelocity();

  // Returns the current position in radians
  float getPosition();

  // Resets the encoder position readings
  void resetPosition();

private:
  ESP32Encoder _encoder;         // Encoder object for reading the encoder
  long _lastPosition;       // Last recorded encoder position (in counts)
  unsigned long _lastChange;  // Time of the last encoder change (in microseconds)
  unsigned long _lastCheck;  // Time of the last velocity calculation (in microseconds)
  unsigned long _timeout;    // Timeout for velocity calculation (in microseconds)
  float _tau;               // Time constant for velocity filter (in seconds)
  float _filteredVelocity;  // Filtered velocity (in radians per second)
  float _velocity;          // Estimated velocity (in radians per second)
  float _countsToRadians;   // Conversion factor from counts to radians
  float _minimumVelocity;   // Minimum velocity to be considered moving (in radians per second)
  bool _moving;             // Flag for whether the encoder is considered moving
  
};

#endif
