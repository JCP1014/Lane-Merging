#ifndef DP_2D_H
#define DP_2D_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <stack>
#include <chrono>
#include "utility.h"
using namespace std;

// Solution of 2D table
struct GreedySol
{
    double time = INFINITY; // the scheduled entering time of the last vehicle
    char table = '0';   // which table we derive from
};

GreedySol update_greedySol(GreedySol s, double newTime, char newTable);
tuple<double, double, double> dp_2d(vector<double> A_all, vector<double> B_all, vector<double> C_all);

#endif