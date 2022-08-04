#include "generate_input.h"

// Round the value to certain decimal places
double digit_round(double value, int digit)
{
    return roundf(value * pow(10, digit)) / pow(10, digit);
}

// Randomly generate earliest arrival times following a Poisson distribution
vector<double> generate_traffic(double timeStep, int num, double p, int seed)
{
    vector<double> earliestArrivalTimes;    // vector of earliest arrival times
    double t;   // time (second)
    default_random_engine generator(time(NULL) + seed);
    uniform_real_distribution<double> unif(0.0, 1.0);

    earliestArrivalTimes.push_back(0.0);    // vehicle with index 0 does not exist

    t = 1.0;    // Start from 1 second
    while (num > 0) // While the number of vehicles is not enough 
    {
        if (unif(generator) < p)
        {
            earliestArrivalTimes.push_back(digit_round(t, 1));
            num -= 1;
        }
        t += timeStep;
    }
    return earliestArrivalTimes;
}
