#ifndef GROUP_MILP_H
#define GROUP_MILP_H

#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "utility.h"
using namespace std;

tuple<double, double, double> solve_group_milp(vector<double> A, vector<double> B, vector<double> C);

#endif