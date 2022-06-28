#ifndef GROUP_DP_H
#define GROUP_DP_H

#include <chrono>
#include <stack>
#include "utility.h"
#include "solution.h"
using namespace std;

double get_tail_time(vector<double> &traffic, pair<int, int> index, double head_time);
double get_wait_time(vector<double> &traffic, pair<int, int> index, double head_time);
pair<double, double> grouped_dp(vector<double> a, vector<double> b, vector<double> c, vector<pair<int, int>> grouped_a, vector<pair<int, int>> grouped_b, vector<pair<int, int>> grouped_c);
tuple<double, double, double> schedule_by_group_dp(vector<double> a_all, vector<double> b_all, vector<double> c_all);

#endif