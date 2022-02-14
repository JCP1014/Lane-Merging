#ifndef GROUP_MILP_H
#define GROUP_MILP_H

#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "get_group.h"
using namespace std;

extern double W_same, W_diff;
vector<pair<int, int>> fixed_threshold_grouping(vector<double> &traffic, double timeStep);
tuple<double, double, double> solve_group_milp(vector<double> A, vector<double> B, vector<double> C, double timeStep);

#endif