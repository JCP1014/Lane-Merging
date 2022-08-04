#ifndef GROUP_DP_H
#define GROUP_DP_H

#include <stack>
#include <libsumo/libtraci.h>
#include "utility.h"
#include "solution.h"

using namespace std;
using namespace libtraci;

double get_tail_time(vector<vehicle> &traffic, pair<int, int> index, double head_time);
void compute_member_time(vector<vehicle> &traffic, pair<int, int> index, double head_time, vector<vehicle> &schedule);
void group_dp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C);

#endif