#ifndef GENERATE_INPUT_H
#define GENERATE_INPUT_H

#include <vector>
#include <random>
#include <time.h>
#include <fstream>
using namespace std;

float digit_round(float value, int digit);
vector<float> generate_traffic(float timeStep, int num, float p, int seed);
vector<float> read_data(string fileName);

#endif