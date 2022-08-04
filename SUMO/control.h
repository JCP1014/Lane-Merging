#ifndef CONTROL_H
#define CONTROL_H

#include "utility.h"
#include "get_vehicle_info.h"
#include "fafg.h"
#include "milp.h"
#include "dp.h"
#include "group_milp.h"
#include "group_dp.h"

#define FAFG 0
#define MILP 1
#define DP 2
#define GROUP_MILP 3
#define GROUP_DP 4

bool sort_time(vehicle a, vehicle b);
void set_headway(vector<vehicle> &schedule);
void set_headway(vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C);
void set_initial_gap(vector<string> &IDs, vector<bool> &inited, double laneLength, double minGap, double endTime);
void run(int approach, int alpha, int beta, int gamma, int windowSize = 100);

#endif