#ifndef FIRFILTER_H
#define FIRFILTER_H

class FIRFilter {
public:
    /**
     * Constructor for FIRFilter.
     * Initializes a FIR filter with a buffer of the specified size.
     * 
     * @param size The size of the buffer for the FIR filter. 
     *             This size must be a power of 2.
     */
    FIRFilter(unsigned int size);

    /**
     * Destructor for FIRFilter.
     * Cleans up dynamically allocated memory.
     */
    ~FIRFilter();

    /**
     * Updates the filter with a new reading and returns the filtered value.
     * 
     * @param newReading The new data point to add to the filter.
     * @return The filtered output value.
     */
    float update(float newReading);

private:
    // Initializes the buffer with zeros
    void initializeBuffer();

    // Calculates the amount to shift for efficient division, based on buffer size
    unsigned int getShiftAmount(unsigned int value);

    float *gyroBuffer;         // Dynamic array to hold the gyro readings
    unsigned int bufferSize;   // Size of the buffer, must be a power of 2
    unsigned int bufferShift;  // Amount to shift for division (log2 of bufferSize)
    unsigned int bufferIndex;  // Current index in the buffer for the next reading
    float runningSum;          // Running sum of the last 'n' readings
    unsigned int count;        // Count of readings added, used during initial fill
};

#endif // FIRFILTER_H
