#ifndef UTILITY_H
#define UTILITY_H

#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

extern double W_same, W_diff;

vector<double> read_data(string fileName);
vector<double> get_window_by_num(vector<double> &traffic, int carNum);
vector<pair<int, int>> grouping(vector<double> &traffic);
tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<double> traffic, tuple<char, int, double> prev);

#endif