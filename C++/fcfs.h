#ifndef FCFS_H
#define FCFS_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <chrono>
#define endl '\n'
using namespace std;

tuple<char, int, float> schedule_single_lane(char lane, vector<float> traffic, float W_same, float W_diff, tuple<char, int, float> prev);
tuple<float, double> first_come_first_serve_v1(float timeStep, vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff);
tuple<float, double> first_come_first_serve_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff);

#endif