#ifndef REDUCED_DP_H
#define REDUCED_DP_H

#include <stack>
#include <iostream>
#include "solution.h"
#include "fcfs.h"
using namespace std;

tuple<tuple<char, int, float>, tuple<char, int, float>, int, int, int, float> reduced_dp(vector<float> a, vector<float> b, vector<float> c, tuple<char, int, float> last_X, tuple<char, int, float> last_Y);
tuple<float, float, double> schedule_by_reduced_dp(vector<float> a, vector<float> b, vector<float> c);

#endif