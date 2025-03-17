#ifndef SIMPLEFILTERS_H
#define SIMPLEFILTERS_H

class LeadFilter {
public:
    LeadFilter(double alpha, double Td);
    void setParameters(double alpha, double Td);
    double calculate(double input);

private:
    double _alpha;
    double _Td;
    double _lastInput;
    double _lastOutput;
    unsigned long _lastTime;
};

class LagFilter {
public:
    LagFilter(double Ti);
    void setParameters(double Ti);
    double calculate(double input);

private:
    double _Ti;
    double _lastOutput;
    unsigned long _lastTime;
};

/**
 * @brief The LeadLagFilter class represents a lead-lag filter.
 * 
 * The lead-lag filter is used to filter a signal by combining a lead filter and a lag filter.
 * It provides a smoother response to the input signal by introducing a time delay and a time constant.
 */
class LeadLagFilter {
public:
    /**
     * @brief Constructs a LeadLagFilter object with the specified parameters.
     * 
     * @param alpha The smoothing factor of the filter.
     * @param Td The time delay of the lead filter.
     * @param Ti The time constant of the lag filter.
     */
    LeadLagFilter(double alpha, double Td, double Ti);

    /**
     * @brief Sets the parameters of the lead-lag filter.
     * 
     * @param alpha The smoothing factor of the filter.
     * @param Td The time delay of the lead filter.
     * @param Ti The time constant of the lag filter.
     */
    void setParameters(double alpha, double Td, double Ti);

    /**
     * @brief Calculates the filtered output for the given input.
     * 
     * @param input The input signal to be filtered.
     * @return The filtered output.
     */
    double calculate(double input);

private:
    double _alpha; /**< The smoothing factor of the filter. */
    double _Td; /**< The time delay of the lead filter. */
    double _Ti; /**< The time constant of the lag filter. */
    LeadFilter _leadFilter; /**< The lead filter used in the lead-lag filter. */
    LagFilter _lagFilter; /**< The lag filter used in the lead-lag filter. */
};
#endif // SIMPLEFILTERS_H
// class LeadLagFilter {
// public:
//     LeadLagFilter(double alpha, double Td, double Ti);
//     void setParameters(double alpha, double Td, double Ti);
//     double calculate(double input);
// private:
//     double _alpha;
//     double _Td;
//     double _Ti;
//     LeadFilter _leadFilter;
//     LagFilter _lagFilter;
// };
// #endif // SIMPLEFILTERS_H

