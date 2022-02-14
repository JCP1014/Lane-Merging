#ifndef MILP_H
#define MILP_H

#include <bits/stdc++.h>
#include "gurobi_c++.h"
#include "generate_input.h"
using namespace std;

extern double W_same, W_diff;
tuple<double, double, double> solve_milp(vector<double> A, vector<double> B, vector<double> C);

#endif