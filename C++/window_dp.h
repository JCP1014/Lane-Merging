#ifndef WINDOW_DP_H
#define WINDOW_DP_H

#include <stack>
#include <iostream>
#include "fcfs.h"
#include "solution.h"
#include "get_window.h"
using namespace std;

extern float W_same, W_diff;
tuple<tuple<char, int, float>, tuple<char, int, float>, float> window_oneSol_dp_v2(vector<float> a, vector<float> b, vector<float> c, tuple<char, int, float> last_X, tuple<char, int, float> last_Y);
tuple<float, float, double> schedule_by_window_dp_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all, int carNum);

#endif