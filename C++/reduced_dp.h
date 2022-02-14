#ifndef REDUCED_DP_H
#define REDUCED_DP_H

#include <stack>
#include <iostream>
#include "solution.h"
#include "fcfs.h"
using namespace std;

tuple<tuple<char, int, double>, tuple<char, int, double>, int, int, int, double> reduced_dp(vector<double> a, vector<double> b, vector<double> c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y);
tuple<double, double, double> schedule_by_reduced_dp(vector<double> a, vector<double> b, vector<double> c);

#endif