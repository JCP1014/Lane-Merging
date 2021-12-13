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

extern float W_same, W_diff;
struct GreedySol
{
    float time = INFINITY;
    char table = '0';
};

GreedySol update_greedySol(GreedySol s, float newTime, char newTable);
tuple<float, float, double> greedy_dp(vector<float> a_all, vector<float> b_all, vector<float> c_all);

#endif