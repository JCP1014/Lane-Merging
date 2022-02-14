#ifndef FCFS_H
#define FCFS_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <chrono>
using namespace std;

extern double W_same, W_diff;
tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<double> traffic, tuple<char, int, double> prev);
pair<double, double> first_come_first_serve_v1(double timeStep, vector<double> a_all, vector<double> b_all, vector<double> c_all);
tuple<double, double, double> first_come_first_serve_v2(vector<double> a_all, vector<double> b_all, vector<double> c_all);

#endif