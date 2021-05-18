#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <tuple>
#include <time.h>
#include <stack>
#include <chrono>
#define endl '\n'
using namespace std;

struct Solution
{
    float time[2] = {INFINITY, INFINITY};
    string table = "";
    string lane = "";
};

struct Lane
{
    int num;
    vector<float> traffic;
};

float digit_round(float value, int digit)
{
    return roundf(value * pow(10, digit)) / pow(10, digit);
}
vector<float> generate_traffic(float timeStep, int num, float p)
{
    vector<float> earliestArrivalTimes;
    float t;
    default_random_engine generator(time(NULL));
    uniform_real_distribution<float> unif(0.0, 1.0);

    earliestArrivalTimes.push_back(0.0);

    t = 1.0;
    while (num > 0)
    {
        if (unif(generator) < p)
        {
            earliestArrivalTimes.push_back(digit_round(t, 1));
            num -= 1;
        }
        t += timeStep;
    }
    return earliestArrivalTimes;
}

Solution update_sol(Solution s, float newTimeX, float newTimeY, string newTable, string newLane)
{
    s.time[0] = newTimeX;
    s.time[1] = newTimeY;
    s.table = newTable;
    s.lane = newLane;
    return s;
}

Solution update_by_minMax(Solution s, vector<Solution> solVec)
{
    float minMax = max(solVec[0].time[0], solVec[0].time[1]);
    int index = 0;
    float tmp;

    for (int i = 1; i < solVec.size(); ++i)
    {
        // int arr_len = sizeof(sol.time) / sizeof(*sol.time);
        // int maxVal = *max_element(sol.time, sol.time + arr_len);
        tmp = max(solVec[i].time[0], solVec[i].time[1]);
        if (tmp < minMax)
        {
            minMax = tmp;
            index = i;
        }
    }
    s = update_sol(s, solVec[index].time[0], solVec[index].time[1], solVec[index].table, solVec[index].lane);
    return s;
}

string get_opt_table(vector<Solution> solVec)
{
    float minMax = max(solVec[0].time[0], solVec[0].time[1]);
    int index = 0;
    float tmp;
    string optTable = "AB";

    for (int i = 1; i < solVec.size(); ++i)
    {
        tmp = max(solVec[i].time[0], solVec[i].time[1]);
        if (tmp < minMax)
        {
            minMax = tmp;
            index = i;
        }
    }
    switch (index)
    {
    case 0:
    {
        optTable = "AB";
        break;
    }
    case 1:
    {
        optTable = "AC";
        break;
    }
    case 2:
    {
        optTable = "BB";
        break;
    }
    case 3:
    {
        optTable = "BC";
        break;
    }
    default:
    {
        optTable = "";
        break;
    }
    }
    return optTable;
}

tuple<tuple<char, int, float>, tuple<char, int, float>, double> window_oneSol_dp(vector<float> a, vector<float> b, vector<float> c, float W_same, float W_diff, tuple<char, int, float> last_X, tuple<char, int, float> last_Y)
{
    auto t_start = chrono::high_resolution_clock::now();
    int alpha = a.size() - 1;
    int beta = b.size() - 1;
    int gamma = c.size() - 1;
    Solution L_AB[alpha + 1][beta + 1][gamma + 1];
    Solution L_AC[alpha + 1][beta + 1][gamma + 1];
    Solution L_BB[alpha + 1][beta + 1][gamma + 1];
    Solution L_BC[alpha + 1][beta + 1][gamma + 1];
    string last_XY;
    float T_X, T_Y;
    vector<Solution> tmpSolVec;

    // Initialize
    // L_AB[0][0][0].time[0] = get<2>(last_X);
    // L_AB[0][0][0].time[1] = get<2>(last_Y);
    // L_AB[0][0][0].table = "0";
    // L_AB[0][0][0].lane = "0";
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");

    last_XY = get<0>(last_X) + get<0>(last_Y);
    T_X = get<2>(last_X);
    T_Y = get<2>(last_Y);

    if (last_XY == "AB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_same), max(b[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_same), max(c[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_diff), max(c[1], T_Y + W_diff), "BC", "XY");
        // vector<Solution> solVec(2);
        tmpSolVec.push_back(Solution());
        tmpSolVec.push_back(Solution());
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = update_by_minMax(L_BB[0][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_same), T_Y, "AB", "X0");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_same), "AB", "0Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_same), T_Y, "AC", "X0");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_diff), "AC", "0Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_diff), T_Y, "BC", "X0");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_diff), "BC", "0Y");
    }
    else if (last_XY == "AC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_same), max(b[1], T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_same), max(c[1], T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_diff), max(c[1], T_Y + W_same), "BC", "XY");
        tmpSolVec.push_back(Solution());
        tmpSolVec.push_back(Solution());
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_diff), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = update_by_minMax(L_BB[0][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_same), T_Y, "AB", "X0");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_diff), "AB", "0Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_same), T_Y, "AC", "X0");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_same), "AC", "0Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_diff), T_Y, "BC", "X0");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_same), "BC", "0Y");
    }
    else if (last_XY == "BB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_diff), max(b[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_diff), max(c[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_same), max(c[1], T_Y + W_diff), "BC", "XY");
        tmpSolVec.push_back(Solution());
        tmpSolVec.push_back(Solution());
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = update_by_minMax(L_BB[0][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_diff), T_Y, "AB", "X0");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_same), "AB", "0Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_diff), T_Y, "AC", "X0");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_diff), "AC", "0Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_same), T_Y, "BC", "X0");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_diff), "BC", "0Y");
    }
    else if (last_XY == "BC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_diff), max(b[1], T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_diff), max(c[1], T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_same), max(c[1], T_Y + W_same), "BC", "XY");
        tmpSolVec.push_back(Solution());
        tmpSolVec.push_back(Solution());
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_diff), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = update_by_minMax(L_BB[0][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_diff), T_Y, "AB", "X0");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_diff), "AB", "0Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_diff), T_Y, "AC", "X0");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_same), "AC", "0Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_same), T_Y, "BC", "X0");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_same), "BC", "0Y");
    }
    else
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], a[1], b[1], "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], a[1], c[1], "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], b[1], c[1], "BC", "XY");
        L_BB[0][1][0] = (a[1] <= c[1]) ? update_sol(L_BB[0][1][0], -W_diff, b[1], "BB", "0Y") : update_sol(L_BB[0][1][0], b[1], -W_diff, "BB", "X0");
        if (beta >= 2)
        {
            L_BB[0][2][0] = (a[1] <= c[1]) ? update_sol(L_BB[0][2][0], b[1], b[2], "BB", "XY") : update_sol(L_BB[0][2][0], b[2], b[1], "BB", "YX");
        }
        L_AB[1][0][0] = update_sol(L_AB[1][0][0], a[1], -W_diff, "AB", "X0");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, b[1], "AB", "0Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], a[1], -W_diff, "AC", "X0");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, c[1], "AC", "0Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], b[1], -W_diff, "BC", "X0");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, c[1], "BC", "0Y");
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], max(a[i], L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "XY");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], max(a[i], L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "XY");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], max(c[k], L_AC[1][0][k - 1].time[1] + W_same), "AC", "XY");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], max(c[k], L_BC[0][1][k - 1].time[1] + W_same), "BC", "XY");
    if (beta >= 3)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_BB[0][j - 2][0].time[0] + W_same), max(b[j], L_BB[0][j - 2][0].time[1] + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_BB[0][j - 2][0].time[0] + W_same), max(b[j - 1], L_BB[0][j - 2][0].time[1] + W_same), "BB", "YX");
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], max(b[j - 1], L_BB[0][j - 2][0].time[0] + W_same) + W_same), L_BB[0][j - 2][0].time[1], "BB", "XX");
            tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[0][j - 2][0].time[0], max(b[j], max(b[j - 1], L_BB[0][j - 2][0].time[1] + W_same) + W_same), "BB", "YY");
            L_BB[0][j][0] = update_by_minMax(L_BB[0][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], max(a[i], L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X0");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], max(b[j], L_AB[0][j - 1][0].time[1] + W_same), "AB", "0Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], max(a[i], L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X0");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], max(c[k], L_AC[0][0][k - 1].time[1] + W_same), "AC", "0Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], max(b[j], L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X0");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], max(c[k], L_BC[0][0][k - 1].time[1] + W_same), "BC", "0Y");

    if (beta >= 2)
    {
        if (last_XY == "AB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = update_by_minMax(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "AC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = update_by_minMax(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = update_by_minMax(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = update_by_minMax(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), b[2], "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), b[1], "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], b[1], max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], b[2], max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = update_by_minMax(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j - 1][0].time[0] + W_same), max(b[j], L_AB[i - 1][j - 1][0].time[1] + W_same), "AB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], max(b[j], L_AB[i - 1][j - 1][0].time[0] + W_diff) + W_diff), L_AB[i - 1][j - 1][0].time[1], "AB", "XX");
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_BB[i - 1][j - 1][0].time[0] + W_diff), max(b[j], L_BB[i - 1][j - 1][0].time[1] + W_same), "BB", "XY");
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], max(b[j], L_BB[i - 1][j - 1][0].time[0] + W_same) + W_diff), L_BB[i - 1][j - 1][0].time[1], "BB", "XX");
            L_AB[i][j][0] = update_by_minMax(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], max(a[i], L_AC[i - 1][0][k - 1].time[0] + W_same), max(c[k], L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BB[0][j - 1][k - 1].time[1] + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BB[0][j - 1][k - 1].time[1] + W_same) + W_diff), "BB", "YY");
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], L_BC[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BC[0][j - 1][k - 1].time[1] + W_same), "BC", "XY");
            tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BC[0][j - 1][k - 1].time[1] + W_diff) + W_diff), "BC", "YY");
            L_BC[0][j][k] = update_by_minMax(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec.push_back(Solution());
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j], L_AB[i][j - 2][0].time[1] + W_same), "AB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][0].time[1] + W_same), "AB", "YX");
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same), max(b[j], L_BB[i][j - 2][0].time[1] + W_same), "BB", "XY");
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j], L_BB[i][j - 2][0].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same), "BB", "YX");
            tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same) + W_same), L_BB[i][j - 2][0].time[1], "BB", "XX");
            tmpSolVec[5] = update_sol(tmpSolVec[5], L_BB[i][j - 2][0].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same) + W_same), "BB", "YY");
            L_BB[i][j][0] = update_by_minMax(L_BB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 1; j <= beta; ++j)
        {
            for (int k = 1; k <= gamma; ++k)
            {
                // L_AB
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j - 1][k].time[0] + W_same), max(b[j], L_AB[i - 1][j - 1][k].time[1] + W_same), "AB", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], max(b[j], L_AB[i - 1][j - 1][k].time[0] + W_diff) + W_diff), L_AB[i - 1][j - 1][k].time[1], "AB", "XX");
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_AC[i - 1][j - 1][k].time[0] + W_same), max(b[j], L_AC[i - 1][j - 1][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], L_BB[i - 1][j - 1][k].time[0] + W_diff), max(b[j], L_BB[i - 1][j - 1][k].time[1] + W_same), "BB", "XY");
                tmpSolVec[4] = update_sol(tmpSolVec[4], max(a[i], max(b[j], L_BB[i - 1][j - 1][k].time[0] + W_same) + W_diff), L_BB[i - 1][j - 1][k].time[1], "BB", "XX");
                tmpSolVec[5] = update_sol(tmpSolVec[5], max(a[i], L_BC[i - 1][j - 1][k].time[0] + W_diff), max(b[j], L_BC[i - 1][j - 1][k].time[1] + W_diff), "BC", "XY");
                L_AB[i][j][k] = update_by_minMax(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k - 1].time[0] + W_same), max(c[k], L_AB[i - 1][j][k - 1].time[1] + W_diff), "AB", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_AC[i - 1][j][k - 1].time[0] + W_same), max(c[k], L_AC[i - 1][j][k - 1].time[1] + W_same), "AC", "XY");
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_BB[i - 1][j][k - 1].time[0] + W_diff), max(c[k], L_BB[i - 1][j][k - 1].time[1] + W_diff), "BB", "XY");
                tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], L_BC[i - 1][j][k - 1].time[0] + W_diff), max(c[k], L_BC[i - 1][j][k - 1].time[1] + W_same), "BC", "XY");
                L_AC[i][j][k] = update_by_minMax(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_AB[i][j - 1][k - 1].time[0] + W_diff), max(c[k], L_AB[i][j - 1][k - 1].time[1] + W_diff), "AB", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k - 1].time[0] + W_diff), max(c[k], L_AC[i][j - 1][k - 1].time[1] + W_same), "AC", "XY");
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], L_BB[i][j - 1][k - 1].time[0] + W_same), max(c[k], L_BB[i][j - 1][k - 1].time[1] + W_diff), "BB", "XY");
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BB[i][j - 1][k - 1].time[1] + W_same) + W_diff), "BB", "YY");
                tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], L_BC[i][j - 1][k - 1].time[0] + W_same), max(c[k], L_BC[i][j - 1][k - 1].time[1] + W_same), "BC", "XY");
                tmpSolVec[5] = update_sol(tmpSolVec[5], L_BC[i][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BC[i][j - 1][k - 1].time[1] + W_diff) + W_diff), "BC", "YY");
                L_BC[i][j][k] = update_by_minMax(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec.push_back(Solution());
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff), max(b[j], L_AB[i][j - 2][k].time[1] + W_same), "AB", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][k].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][k].time[1] + W_same), "AB", "YX");
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j - 1], L_AC[i][j - 2][k].time[0] + W_diff), max(b[j], L_AC[i][j - 2][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j], L_AC[i][j - 2][k].time[0] + W_diff), max(b[j - 1], L_AC[i][j - 2][k].time[1] + W_diff), "AC", "YX");
                tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same), max(b[j], L_BB[i][j - 2][k].time[1] + W_same), "BB", "XY");
                tmpSolVec[5] = update_sol(tmpSolVec[5], max(b[j], L_BB[i][j - 2][k].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same), "BB", "YX");
                tmpSolVec[6] = update_sol(tmpSolVec[6], max(b[j], max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same) + W_same), L_BB[i][j - 2][k].time[1], "BB", "XX");
                tmpSolVec[7] = update_sol(tmpSolVec[7], L_BB[i][j - 2][k].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same) + W_same), "BB", "YY");
                tmpSolVec[8] = update_sol(tmpSolVec[8], max(b[j - 1], L_BC[i][j - 2][k].time[0] + W_same), max(b[j], L_BC[i][j - 2][k].time[1] + W_diff), "BC", "XY");
                tmpSolVec[9] = update_sol(tmpSolVec[9], max(b[j], L_BC[i][j - 2][k].time[0] + W_same), max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_diff), "BC", "YX");
                tmpSolVec[10] = update_sol(tmpSolVec[10], L_BC[i][j - 2][k].time[0], max(b[j], max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_same) + W_diff), "BC", "YY");
                L_BB[i][j][k] = update_by_minMax(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }
    // Push order to stack
    stack<tuple<char, int, float>> stack_X, stack_Y;
    int i = alpha;
    int j = beta;
    int k = gamma;
    string optTable;
    string table = "";
    string lanes = "";
    // Choose the optimal solution and the table to start backtracking
    tmpSolVec.push_back(L_AB[i][j][k]);
    tmpSolVec.push_back(L_AC[i][j][k]);
    tmpSolVec.push_back(L_BB[i][j][k]);
    tmpSolVec.push_back(L_BC[i][j][k]);
    optTable = get_opt_table(tmpSolVec);
    tmpSolVec.clear();

    if (optTable == "AB")
    {
        table = L_AB[i][j][k].table;
        lanes = L_AB[i][j][k].lane;
        if (lanes == "XY")
        {
            stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
            stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
            --i;
            --j;
        }
        else if (lanes == "XX")
        {
            stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
            if (table == "AB")
                stack_X.push(make_tuple('B', j, max(b[j], L_AB[i - 1][j - 1][k].time[0] + W_diff)));
            else if (table == "BB")
                stack_X.push(make_tuple('B', j, max(b[j], L_BB[i - 1][j - 1][k].time[0] + W_same)));
            else
                cout << "bug" << endl;
            --i;
            --j;
        }
        else if (lanes[0] == 'X')
        {
            stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
            --i;
        }
        else if (lanes[1] == 'Y')
        {
            stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
            --j;
        }
    }
    else if (optTable == "AC")
    {
        table = L_AC[i][j][k].table;
        lanes = L_AC[i][j][k].lane;
        if (lanes == "XY")
        {
            stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
            stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
            --i;
            --k;
        }
        else if (lanes[0] == 'X')
        {
            stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
            --i;
        }
        else if (lanes[1] == 'Y')
        {
            stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
            --k;
        }
    }
    else if (optTable == "BB")
    {
        table = L_BB[i][j][k].table;
        lanes = L_BB[i][j][k].lane;
        if (lanes == "XY")
        {
            stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
        }
        else if (lanes == "YX")
        {
            stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
        }
        else if (lanes == "XX")
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same)));
        }
        else if (lanes == "YY")
        {
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            if (table == "BB")
                stack_Y.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same)));
            else if (table == "BC")
                stack_Y.push(make_tuple('B', j - 1, max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_same)));
            else
                cout << "bug" << endl;
        }
        j -= 2;
    }
    else if (optTable == "BC")
    {
        table = L_BC[i][j][k].table;
        lanes = L_BC[i][j][k].lane;
        if (lanes == "XY")
        {
            stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
            stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
            --j;
            --k;
        }
        else if (lanes == "YY")
        {
            stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
            if (table == "BB")
                stack_Y.push(make_tuple('B', j, max(b[j], L_BB[i][j - 1][k - 1].time[1] + W_same)));
            else if (table == "BC")
                stack_Y.push(make_tuple('B', j, max(b[j], L_BC[i][j - 1][k - 1].time[1] + W_diff)));
            else
                cout << "bug" << endl;
            --j;
            --k;
        }
        else if (lanes[0] == 'X')
        {
            stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
            --j;
        }
        else if (lanes[1] == 'Y')
        {
            stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
            --k;
        }
    }

    // Backtracking
    while (i > 0 || j > 0 || k > 0)
    {
        cout << i << " " << j << " " << k << " ";
        cout << table << " " << lanes << endl;
        if (table == "AB")
        {
            table = L_AB[i][j][k].table;
            lanes = L_AB[i][j][k].lane;
            if (lanes == "XY")
            {
                stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
                stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
                --i;
                --j;
            }
            else if (lanes == "XX")
            {
                stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
                if (table == "AB")
                    stack_X.push(make_tuple('B', j, max(b[j], L_AB[i - 1][j - 1][k].time[0] + W_diff)));
                else if (table == "BB")
                    stack_X.push(make_tuple('B', j, max(b[j], L_BB[i - 1][j - 1][k].time[0] + W_same)));
                else
                    cout << "bug" << endl;
                --i;
                --j;
            }
            else if (lanes[0] == 'X')
            {
                stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
                --i;
            }
            else if (lanes[1] == 'Y')
            {
                stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
                --j;
            }
        }
        else if (table == "AC")
        {
            table = L_AC[i][j][k].table;
            lanes = L_AC[i][j][k].lane;
            if (lanes == "XY")
            {
                stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
                stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
                --i;
                --k;
            }
            else if (lanes[0] == 'X')
            {
                stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
                --i;
            }
            else if (lanes[1] == 'Y')
            {
                stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
                --k;
            }
        }
        else if (table == "BB")
        {
            table = L_BB[i][j][k].table;
            lanes = L_BB[i][j][k].lane;
            if (lanes == "XY")
            {
                stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            }
            else if (lanes == "YX")
            {
                stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            }
            else if (lanes == "XX")
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same)));
            }
            else if (lanes == "YY")
            {
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                if (table == "BB")
                    stack_Y.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same)));
                else if (table == "BC")
                    stack_Y.push(make_tuple('B', j - 1, max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_same)));
                else
                    cout << "bug" << endl;
            }
            j -= 2;
        }
        else if (table == "BC")
        {
            table = L_BC[i][j][k].table;
            lanes = L_BC[i][j][k].lane;
            if (lanes == "XY")
            {
                stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
                stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
                --j;
                --k;
            }
            else if (lanes == "YY")
            {
                stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
                if (table == "BB")
                    stack_Y.push(make_tuple('B', j, max(b[j], L_BB[i][j - 1][k - 1].time[1] + W_same)));
                else if (table == "BC")
                    stack_Y.push(make_tuple('B', j, max(b[j], L_BC[i][j - 1][k - 1].time[1] + W_diff)));
                else
                    cout << "bug" << endl;
                --j;
                --k;
            }
            else if (lanes[0] == 'X')
            {
                stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
                --j;
            }
            else if (lanes[1] == 'Y')
            {
                stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
                --k;
            }
        }
    }

    // Delete the redundant element (i==0 or j==0 or k==0)
    while (get<1>(stack_X.top()) <= 0)
        stack_X.pop();
    while (get<1>(stack_Y.top()) <= 0)
        stack_Y.pop();

    // Output order
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        stack_X.pop();
    }
    last_X = stack_X.top();
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        stack_Y.pop();
    }
    last_Y = stack_Y.top();

    // Cauculate computation time
    auto t_end = chrono::high_resolution_clock::now();
    double computeTime = chrono::duration_cast<chrono::nanoseconds>(t_end - t_start).count();
    computeTime *= 1e-9;

    return make_tuple(last_X, last_Y, computeTime);
}

vector<float> get_window_by_num(vector<float> &traffic, int carNum)
{
    vector<float> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);
    return subtraffic;
}

tuple<float, double> schedule_by_num_window(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff, int carNum)
{
    tuple<char, int, float> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, float> last_Y = make_tuple('0', 0, 0.0);
    double tmp;
    float T_last;
    auto t0 = chrono::high_resolution_clock::now();
    while ((a_all.size() > 1 && a_all.size() > 1) ||
           (b_all.size() > 1 && c_all.size() > 1) ||
           (b_all.size() > 2))
    {
        vector<float> a = get_window_by_num(a_all, carNum);
        vector<float> b = get_window_by_num(b_all, carNum);
        vector<float> c = get_window_by_num(c_all, carNum);
        tie(last_X, last_Y, tmp) = window_oneSol_dp(a, b, c, W_same, W_diff, last_X, last_Y);
        cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    // cout << "Time taken by program is : " << fixed
    //      << totalComputeTime << setprecision(9);
    // cout << " sec" << endl;
    T_last = (get<2>(last_X) >= get<2>(last_Y)) ? get<2>(last_X) : get<2>(last_Y);
    return make_tuple(T_last, totalComputeTime);
}

int main(int argc, char *argv[])
{
    ios::sync_with_stdio(false);
    cin.tie(0);

    float timeStep = 1;
    float W_same, W_diff;
    int alpha, beta, gamma;
    float p, pA, pB, pC;
    vector<float> a_all, b_all, c_all;

    if (argc == 5)
    {
        W_same = atof(argv[3]);
        W_diff = atof(argv[4]);
        alpha = atoi(argv[2]);
        beta = atoi(argv[2]);
        gamma = atoi(argv[2]);
        p = atof(argv[1]);
        pA = p / 3;
        pB = p / 3;
        pC = p / 3;
    }
    else
    {
        cout << "Arguments: lambda, N, W=, W+" << endl;
        return 0;
    }

    a_all = generate_traffic(timeStep, alpha, pA);
    b_all = generate_traffic(timeStep, beta, pB);
    c_all = generate_traffic(timeStep, gamma, pC);
    schedule_by_num_window(a_all, b_all, c_all, W_same, W_diff, 5);

    return 0;
}
