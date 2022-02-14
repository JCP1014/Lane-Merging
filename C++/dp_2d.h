#ifndef DP_2D_H
#define DP_2D_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <stack>
#include <chrono>
using namespace std;

extern double W_same, W_diff;
struct GreedySol
{
    double time = INFINITY;
    char table = '0';
};

GreedySol update_greedySol(GreedySol s, double newTime, char newTable);
tuple<double, double, double> greedy_dp(vector<double> a_all, vector<double> b_all, vector<double> c_all);

#endif