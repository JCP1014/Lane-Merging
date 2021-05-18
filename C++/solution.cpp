#include <random>
#include <algorithm>
#include "solution.h"
using namespace std;

Solution::Solution(float time[2], string table, string lane)
{
    this->time[0] = INFINITY;
    this->time[1] = INFINITY;
    this->table = "0";
    this->lane = "0";
}

float Solution::getObj()
{
    int arr_len = sizeof(this->time) / sizeof(*this->time);
    return *max_element(this->time, this->time + arr_len);
}

void Solution::setValue(float newTime[2], string newTable, string newLane)
{
    this->time[0] = newTime[0];
    this->time[1] = newTime[1];
    this->table = newTable;
    this->lane = newLane;
}
