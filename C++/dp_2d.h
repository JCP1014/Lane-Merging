#ifndef DP_2D_H
#define DP_2D_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <stack>
#include <chrono>
#define endl '\n'
using namespace std;

struct GreedySol
{
    float time = INFINITY;
    char table = '0';
};

GreedySol update_greedySol(GreedySol s, float newTime, char newTable);
pair<float, double> greedy_dp(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff);

#endif