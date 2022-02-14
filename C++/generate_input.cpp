#include "generate_input.h"

double digit_round(double value, int digit)
{
    return roundf(value * pow(10, digit)) / pow(10, digit);
}

vector<double> generate_traffic(double timeStep, int num, double p, int seed)
{
    vector<double> earliestArrivalTimes;
    double t;
    default_random_engine generator(time(NULL) + seed);
    uniform_real_distribution<double> unif(0.0, 1.0);

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

vector<double> read_data(string fileName)
{
    fstream file;
    string line;
    double tmp;
    vector<double> allData;
    file.open(fileName);
    while (getline(file, line))
        allData.push_back(stof(line));
    file.close();
    return allData;
}