#include "get_window.h"

vector<double> get_window_by_num(vector<double> &traffic, int carNum)
{
    if (carNum >= traffic.size())
        carNum = traffic.size() - 1;
    vector<double> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);
    return subtraffic;
}