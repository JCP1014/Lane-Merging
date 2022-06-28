#ifndef GET_VEHICLE_INFO_H
#define GET_VEHICLE_INFO_H

#include <libsumo/libtraci.h>
#include "utility.h"

using namespace std;
using namespace libtraci;

vector<vehicle> compute_earliest_arrival(double laneLength, vector<vehicle> &schedule, char lane);
void print_schedule(vector<vehicle> schedule, string lane);

#endif