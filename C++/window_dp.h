#ifndef WINDOW_DP_H
#define WINDOW_DP_H

#include <stack>
#include <iostream>
#include "utility.h"
#include "solution.h"
using namespace std;

tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_dp_3d(vector<double> a, vector<double> b, vector<double> c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y);
tuple<double, double, double> schedule_by_window_dp(vector<double> a_all, vector<double> b_all, vector<double> c_all, int carNum);

#endif