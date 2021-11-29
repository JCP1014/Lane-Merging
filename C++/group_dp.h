#ifndef GROUP_DP_H
#define GROUP_DP_H

#include <bits/stdc++.h>
#include "generate_input.h"
#include "solution.h"
using namespace std;

extern float W_same, W_diff;
vector<pair<int, int>> grouping(vector<float> &traffic, float timeStep);
float get_tail_time(vector<float> &traffic, pair<int, int> index, float head_time);
pair<float, double> grouped_dp(vector<float> a, vector<float> b, vector<float> c, vector<pair<int, int>> grouped_a, vector<pair<int, int>> grouped_b, vector<pair<int, int>> grouped_c);
pair<float, double> schedule_by_group(vector<float> a_all, vector<float> b_all, vector<float> c_all, float timeStep);

#endif