#ifndef PRUNED_DP_H
#define PRUNED_DP_H

#include <stack>
#include <iostream>
#include "utility.h"
#include "solution.h"
#include "fafg.h"
using namespace std;

tuple<tuple<char, int, double>, tuple<char, int, double>, int, int, int, double> pruned_dp(vector<double> a, vector<double> b, vector<double> c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y);
tuple<double, double, double> schedule_by_pruned_dp(vector<double> a, vector<double> b, vector<double> c);

#endif