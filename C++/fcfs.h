#ifndef FCFS_H
#define FCFS_H

#include <iostream>
#include <vector>
#include <random>
#include <tuple>
#include <time.h>
#include <chrono>
#include "utility.h"
using namespace std;

pair<double, double> first_come_first_serve_v1(double timeStep, vector<double> a_all, vector<double> b_all, vector<double> c_all);
tuple<double, double, double> first_come_first_serve_v2(vector<double> a_all, vector<double> b_all, vector<double> c_all);

#endif