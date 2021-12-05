#ifndef FCFS_H
#define FCFS_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <chrono>
using namespace std;

extern float W_same, W_diff;
tuple<char, int, float> schedule_single_lane(char lane, vector<float> traffic, tuple<char, int, float> prev);
pair<float, double> first_come_first_serve_v1(float timeStep, vector<float> a_all, vector<float> b_all, vector<float> c_all);
pair<float, double> first_come_first_serve_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all);

#endif