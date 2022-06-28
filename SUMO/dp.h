#ifndef DP_H
#define DP_H

#include <stack>
#include <libsumo/libtraci.h>
#include "utility.h"
#include "solution.h"

using namespace std;
using namespace libtraci;

tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_dp_compute_entering_time(vector<vehicle> &a, vector<vehicle> &b, vector<vehicle> &c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C);
vector<vehicle> get_window_by_num(vector<vehicle> &traffic, int carNum);
void schedule_by_window_dp(vector<vehicle> a_all, vector<vehicle> b_all, vector<vehicle> c_all, int carNum, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C);

#endif