#ifndef MILP_H
#define MILP_H

#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "generate_input.h"
using namespace std;

extern float W_same, W_diff;
pair<float, double> solve_milp(vector<float> A, vector<float> B, vector<float> C);

#endif