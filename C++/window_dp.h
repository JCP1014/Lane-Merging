#ifndef WINDOW_DP_H
#define WINDOW_DP_H

#include <bits/stdc++.h>
#include "generate_input.h"
#include "fcfs.h"
#include "dp_2d.h"
#include "solution.h"
#include "get_window.h"
using namespace std;

extern float W_same, W_diff;
tuple<tuple<char, int, float>, tuple<char, int, float>, double> window_oneSol_dp_v2(vector<float> a, vector<float> b, vector<float> c, tuple<char, int, float> last_X, tuple<char, int, float> last_Y);
tuple<tuple<char, int, float>, tuple<char, int, float>, int, int, int> reduced_dp(vector<float> a, vector<float> b, vector<float> c, tuple<char, int, float> last_X, tuple<char, int, float> last_Y);
pair<float, double> schedule_by_num_window_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all, int carNum);
pair<float, double> schedule_by_reduced_dp(vector<float> a, vector<float> b, vector<float> c);

#endif