#ifndef WINDOW_DP_H
#define WINDOW_DP_H

#include <stack>
#include <iostream>
#include "fcfs.h"
#include "solution.h"
#include "get_window.h"
using namespace std;

extern double W_same, W_diff;
tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_oneSol_dp_v2(vector<double> a, vector<double> b, vector<double> c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y);
tuple<double, double, double> schedule_by_window_dp_v2(vector<double> a_all, vector<double> b_all, vector<double> c_all, int carNum);

#endif