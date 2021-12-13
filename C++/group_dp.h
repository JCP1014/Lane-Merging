#ifndef GROUP_DP_H
#define GROUP_DP_H

#include <chrono>
#include <stack>
#include "get_group.h"
#include "solution.h"
using namespace std;

extern float W_same, W_diff;
float get_tail_time(vector<float> &traffic, pair<int, int> index, float head_time);
float get_wait_time(vector<float> &traffic, pair<int, int> index, float head_time);
pair<float, float> grouped_dp(vector<float> a, vector<float> b, vector<float> c, vector<pair<int, int>> grouped_a, vector<pair<int, int>> grouped_b, vector<pair<int, int>> grouped_c);
tuple<float, float, double> schedule_by_group_dp(vector<float> a_all, vector<float> b_all, vector<float> c_all, float timeStep);

#endif