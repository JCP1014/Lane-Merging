#ifndef MILP_H
#define MILP_H

#include <libsumo/libtraci.h>
#include "gurobi_c++.h"
#include "utility.h"

using namespace std;
using namespace libtraci;

void milp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C);

#endif