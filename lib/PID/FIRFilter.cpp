#include "FIRFilter.h"
#include <cstring> // For std::fill_n
#include <algorithm> // For std::fill_n
// Constructor
FIRFilter::FIRFilter(unsigned int size) {
    bufferSize = size;
    bufferShift = getShiftAmount(bufferSize);
    gyroBuffer = new float[bufferSize];
    bufferIndex = 0;
    runningSum = 0.0;
    count = 0;
    initializeBuffer();
}

// Destructor
FIRFilter::~FIRFilter() {
    delete[] gyroBuffer;
}

// Initializes the buffer with zeros
void FIRFilter::initializeBuffer() {
    std::fill_n(gyroBuffer, bufferSize, 0.0f);
}

float FIRFilter::update(float newReading) {
    // Remove the oldest value and add the new value to the running sum
    runningSum -= gyroBuffer[bufferIndex];
    runningSum += newReading;

    // Update the buffer with the new reading
    gyroBuffer[bufferIndex] = newReading;
    bufferIndex = (bufferIndex + 1) % bufferSize;

    // Compute the average
    if (count < bufferSize) {
        count++;
        return runningSum / count;
    }
    return runningSum / bufferSize; // Division is efficient for power of 2 sizes
}

// Calculates the amount to shift for efficient division based on buffer size
unsigned int FIRFilter::getShiftAmount(unsigned int value) {
    unsigned int shiftAmount = 0;
    while (value > 1) {
        value >>= 1;
        shiftAmount++;
    }
    return shiftAmount;
}
