#ifndef GENERATE_INPUT_H
#define GENERATE_INPUT_H

#include <vector>
#include <random>
#include <time.h>
#include <fstream>
using namespace std;

double digit_round(double value, int digit);
vector<double> generate_traffic(double timeStep, int num, double p, int seed);
vector<double> read_data(string fileName);

#endif