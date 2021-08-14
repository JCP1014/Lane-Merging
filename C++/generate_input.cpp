#include "generate_input.h"

float digit_round(float value, int digit)
{
    return roundf(value * pow(10, digit)) / pow(10, digit);
}

vector<float> generate_traffic(float timeStep, int num, float p, int seed)
{
    vector<float> earliestArrivalTimes;
    float t;
    default_random_engine generator(time(NULL) + seed);
    uniform_real_distribution<float> unif(0.0, 1.0);

    earliestArrivalTimes.push_back(0.0);

    t = 1.0;
    while (num > 0)
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