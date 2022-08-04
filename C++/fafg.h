#ifndef FAFG_H
#define FAFG_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <chrono>
#include "utility.h"
using namespace std;

tuple<double, double, double> first_arrive_first_go(vector<double> A_all, vector<double> B_all, vector<double> C_all);

#endif