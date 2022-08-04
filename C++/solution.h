#ifndef SOLUTION_H
#define SOLUTION_H

#include <string>
#include <vector>
#include <cmath>
#include <tuple>
#include <iostream>
using namespace std;

struct Solution
{
    double time[2] = {INFINITY, INFINITY};  // scheduled entering times of the last vehicle going to Lane X and Lane Y
    string table = "";  // the table which we derive from
    string lane = "";   // the lane which the last vehicle goes to (or the last two vehicles go to)
    Solution *src = NULL;   // the source solution which we derive from
};

Solution update_sol(Solution s, double newTimeX, double newTimeY, string newTable, string newLane);
Solution update_sol(Solution s, double newTimeX, double newTimeY, string newTable, string newLane, Solution *newSrc);
Solution choose_best_sol(Solution s, vector<Solution> solVec);
string get_opt_table(vector<Solution> solVec);
void print_3d_table(vector<vector<vector<Solution>>> &table);

#endif