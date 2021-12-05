#ifndef GROUP_MILP_H
#define GROUP_MILP_H

#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "get_group.h"
using namespace std;

extern float W_same, W_diff;
vector<pair<int, int>> fixed_threshold_grouping(vector<float> &traffic, float timeStep);
pair<float, double> solve_group_milp(vector<float> A, vector<float> B, vector<float> C, float timeStep);

#endif