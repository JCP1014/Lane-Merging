#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <tuple>
#include <time.h>
#define endl '\n'
using namespace std;

struct Solution
{
    // double time[2] = {INFINITY, INFINITY};
    double timeX = INFINITY;
    double timeY = INFINITY;
    string table = "";
    string lane = "";
};

// Solution update_sol(Solution s, double newTimeX, double newTimeY, string newTable, string newLane)
// {
//     s.time[0] = newTimeX;
//     s.time[1] = newTimeY;
//     s.table = newTable;
//     s.lane = newLane;
//     return s;
// }

Solution update_sol(Solution s, double newTimeX, double newTimeY, string newTable, string newLane)
{
    // s.time[0] = newTimeX;
    // s.time[1] = newTimeY;
    // s.table = newTable;
    // s.lane = newLane;
    // return s;
    tie(s.timeX, s.timeY, s.table, s.lane) = make_tuple(newTimeX, newTimeY, newTable,newLane);
    return s;
}

int main()
{
    ios::sync_with_stdio(false);
    cin.tie(0);

    Solution L_AB[5][5][5];
    cout << L_AB[1][1][1].timeX << " " << L_AB[1][1][1].timeY << " " << L_AB[1][1][1].table << " " << L_AB[1][1][1].lane << endl;
    // L_AB[1][1][1] = {.timeX=1, .timeY=2, .table="AB", .lane="XY"};
    // tie(L_AB[1][1][1].timeX, L_AB[1][1][1].timeY, L_AB[1][1][1].table, L_AB[1][1][1].lane) = make_tuple(1, 2, "AB", "XY");
    L_AB[1][1][1] = update_sol(L_AB[1][1][1], 1, 2, "AB", "XY");
    cout << L_AB[1][1][1].timeX << " " << L_AB[1][1][1].timeY << " " << L_AB[1][1][1].table << " " << L_AB[1][1][1].lane << endl;

    return 0;
}
