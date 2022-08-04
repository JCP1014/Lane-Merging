#ifndef FCFS_H
#define FCFS_H

#include <random>
#include <libsumo/libtraci.h>
#include "utility.h"

using namespace std;
using namespace libtraci;

void fcfs_compute_entering_time(vector<vehicle> a_all, vector<vehicle> b_all, vector<vehicle> c_all, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C);

#endif