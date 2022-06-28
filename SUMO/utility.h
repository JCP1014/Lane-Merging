#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

extern double W_same, W_diff;

struct vehicle
{
    string id;
    double time;
    vehicle() : id(""), time(0) {}
    vehicle(string s, double d) : id(s), time(d) {}
};

struct find_id
{
    string target;
    find_id(string id) : target(id){};
    bool operator()(const vehicle &v) const
    {
        return v.id == target;
    }
};

bool sort_vehicle(vehicle a, vehicle b);
bool sort_id(string a, string b);
vector<pair<int, int>> grouping(vector<vehicle> &traffic);
tuple<tuple<char, int, double>, double> schedule_single_lane(char lane, vector<vehicle> traffic, tuple<char, int, double> prev, vector<vehicle> &schedule);

#endif