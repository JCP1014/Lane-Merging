#ifndef GENERATE_INPUT_H
#define GENERATE_INPUT_H

#include <vector>
#include <random>
#include <time.h>
#define endl '\n'
using namespace std;

float digit_round(float value, int digit);
vector<float> generate_traffic(float timeStep, int num, float p, int seed);

#endif