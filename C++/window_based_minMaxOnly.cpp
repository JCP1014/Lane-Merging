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

struct GreedySol
{
    float time = INFINITY;
    char table = '0';
};

float digit_round(float value, int digit)
{
    return roundf(value * pow(10, digit)) / pow(10, digit);
}

vector<float> generate_traffic(float timeStep, int num, float p, int seed)
{
    vector<float> earliestArrivalTimes;
    float t;
    default_random_engine generator(time(NULL) + seed);
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
    // float minMin = min(solVec[0].time[0], solVec[0].time[1]);
    int index = 0;
    float tmpMax, tmpMin;

    for (int i = 1; i < solVec.size(); ++i)
    {
        // int arr_len = sizeof(sol.time) / sizeof(*sol.time);
        // int maxVal = *max_element(sol.time, sol.time + arr_len);
        tmpMax = max(solVec[i].time[0], solVec[i].time[1]);
        if (tmpMax < minMax)
        {
            minMax = tmpMax;
            index = i;
        }
    }
    s = update_sol(s, solVec[index].time[0], solVec[index].time[1], solVec[index].table, solVec[index].lane);
    return s;
}

GreedySol update_greedySol(GreedySol s, float newTime, char newTable)
{
    s.time = newTime;
    s.table = newTable;
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

tuple<tuple<char, int, float>, tuple<char, int, float>, double> window_oneSol_dp_v1(vector<float> a, vector<float> b, vector<float> c, float W_same, float W_diff, tuple<char, int, float> last_X, tuple<char, int, float> last_Y)
{
    auto t_start = chrono::high_resolution_clock::now();
    int alpha = a.size() - 1;
    int beta = b.size() - 1;
    int gamma = c.size() - 1;
    vector<vector<vector<Solution>>> L_AB;
    vector<vector<vector<Solution>>> L_AC;
    vector<vector<vector<Solution>>> L_BB;
    vector<vector<vector<Solution>>> L_BC;
    string last_XY;
    float T_X, T_Y;
    vector<Solution> tmpSolVec;

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");

    last_XY.push_back(get<0>(last_X));
    last_XY.push_back(get<0>(last_Y));
    T_X = get<2>(last_X);
    T_Y = get<2>(last_Y);
    if (last_XY == "AB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_same), max(b[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_same), max(c[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_diff), max(c[1], T_Y + W_diff), "BC", "XY");
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
                if (j >= 2)
                {
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
                    tmpSolVec.push_back(Solution());
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff), max(b[j], L_AB[i][j - 2][k].time[1] + W_same), "AB", "XY");
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][k].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][k].time[1] + W_same), "AB", "YX");
                    tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff) + W_same), L_AB[i][j - 2][k].time[1], "AB", "XX");
                    tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j - 1], L_AC[i][j - 2][k].time[0] + W_diff), max(b[j], L_AC[i][j - 2][k].time[1] + W_diff), "AC", "XY");
                    tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], L_AC[i][j - 2][k].time[0] + W_diff), max(b[j - 1], L_AC[i][j - 2][k].time[1] + W_diff), "AC", "YX");
                    tmpSolVec[5] = update_sol(tmpSolVec[5], max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same), max(b[j], L_BB[i][j - 2][k].time[1] + W_same), "BB", "XY");
                    tmpSolVec[6] = update_sol(tmpSolVec[6], max(b[j], L_BB[i][j - 2][k].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same), "BB", "YX");
                    tmpSolVec[7] = update_sol(tmpSolVec[7], max(b[j], max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same) + W_same), L_BB[i][j - 2][k].time[1], "BB", "XX");
                    tmpSolVec[8] = update_sol(tmpSolVec[8], L_BB[i][j - 2][k].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same) + W_same), "BB", "YY");
                    tmpSolVec[9] = update_sol(tmpSolVec[9], max(b[j - 1], L_BC[i][j - 2][k].time[0] + W_same), max(b[j], L_BC[i][j - 2][k].time[1] + W_diff), "BC", "XY");
                    tmpSolVec[10] = update_sol(tmpSolVec[10], max(b[j], L_BC[i][j - 2][k].time[0] + W_same), max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_diff), "BC", "YX");
                    tmpSolVec[11] = update_sol(tmpSolVec[11], L_BC[i][j - 2][k].time[0], max(b[j], max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_diff) + W_same), "BC", "YY");
                    L_BB[i][j][k] = update_by_minMax(L_BB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                }
            }
        }
    }

    // Print table
    // cout << "L_AB-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_AB[i][j][k].time[0] << " " << L_AB[i][j][k].time[1] << " " << L_AB[i][j][k].table << " " << L_AB[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
    // cout << "L_AC-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_AC[i][j][k].time[0] << " " << L_AC[i][j][k].time[1] << " " << L_AC[i][j][k].table << " " << L_AC[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
    // cout << "L_BB-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_BB[i][j][k].time[0] << " " << L_BB[i][j][k].time[1] << " " << L_BB[i][j][k].table << " " << L_BB[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
    // cout << "L_BC-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_BC[i][j][k].time[0] << " " << L_BC[i][j][k].time[1] << " " << L_BC[i][j][k].table << " " << L_BC[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }

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
            j -= 2;
        }
        else if (lanes == "YX")
        {
            stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            j -= 2;
        }
        else if (lanes == "XX")
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            if (table == "AB")
                stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff)));
            else if (table == "BB")
                stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same)));
            j -= 2;
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
            j -= 2;
        }
        else if (lanes[0] == 'X')
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            --j;
        }
        else if (lanes[1] == 'Y')
        {
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            --j;
        }
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
        if (i < 0 || j < 0 || k < 0)
            exit(1);
        // cout << i << " " << j << " " << k << " ";
        // cout << table << " " << lanes << endl;
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
                j -= 2;
            }
            else if (lanes == "YX")
            {
                stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                j -= 2;
            }
            else if (lanes == "XX")
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                if (table == "AB")
                    stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff)));
                else if (table == "BB")
                    stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same)));
                j -= 2;
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
                j -= 2;
            }
            else if (lanes[0] == 'X')
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                --j;
            }
            else if (lanes[1] == 'Y')
            {
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                --j;
            }
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
    // cout << "Lane X: " << endl;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    last_X = stack_X.top();
    // cout << "Lane Y: " << endl;
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    last_Y = stack_Y.top();

    // Cauculate computation time
    auto t_end = chrono::high_resolution_clock::now();
    double computeTime = chrono::duration_cast<chrono::nanoseconds>(t_end - t_start).count();
    computeTime *= 1e-9;

    return make_tuple(last_X, last_Y, computeTime);
}

tuple<tuple<char, int, float>, tuple<char, int, float>, double> window_oneSol_dp_v2(vector<float> a, vector<float> b, vector<float> c, float W_same, float W_diff, tuple<char, int, float> last_X, tuple<char, int, float> last_Y)
{
    auto t_start = chrono::high_resolution_clock::now();
    int alpha = a.size() - 1;
    int beta = b.size() - 1;
    int gamma = c.size() - 1;
    vector<vector<vector<Solution>>> L_AB;
    vector<vector<vector<Solution>>> L_AC;
    vector<vector<vector<Solution>>> L_BB;
    vector<vector<vector<Solution>>> L_BC;
    string last_XY;
    float T_X, T_Y;
    vector<Solution> tmpSolVec;

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");

    last_XY.push_back(get<0>(last_X));
    last_XY.push_back(get<0>(last_Y));
    T_X = get<2>(last_X);
    T_Y = get<2>(last_Y);
    if (last_XY == "AB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_same), max(b[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_same), max(c[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_diff), max(c[1], T_Y + W_diff), "BC", "XY");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
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
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_diff), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
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
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
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
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_diff), "BB", "0Y");
        L_BB[0][1][0] = update_by_minMax(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
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
            tmpSolVec.resize(4);
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
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
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
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
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
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
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
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), max(b[2], T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), max(b[1], T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
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
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], L_AC[i][0][0].time[0] + W_diff), b[2], "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], L_AC[i][0][0].time[0] + W_diff), b[1], "AC", "YX");
                L_BB[i][2][0] = update_by_minMax(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
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
            tmpSolVec.resize(4);
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
            tmpSolVec.resize(4);
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
            tmpSolVec.resize(6);
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
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X");
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y");
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y");
                L_AB[i][j][k] = update_by_minMax(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X");
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y");
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y");
                L_AC[i][j][k] = update_by_minMax(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X");
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y");
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y");
                L_BC[i][j][k] = update_by_minMax(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X");
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y");
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y");
                L_BB[i][j][k] = update_by_minMax(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    // Print table
    // cout << "L_AB-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_AB[i][j][k].time[0] << " " << L_AB[i][j][k].time[1] << " " << L_AB[i][j][k].table << " " << L_AB[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
    // cout << "L_AC-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_AC[i][j][k].time[0] << " " << L_AC[i][j][k].time[1] << " " << L_AC[i][j][k].table << " " << L_AC[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
    // cout << "L_BB-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_BB[i][j][k].time[0] << " " << L_BB[i][j][k].time[1] << " " << L_BB[i][j][k].table << " " << L_BB[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }
    // cout << "L_BC-------------------------------" << endl;
    // for (int k = 0; k <= gamma; ++k)
    // {
    //     cout << "k = " << k << endl;
    //     for (int i = 0; i <= alpha; ++i)
    //     {
    //         for (int j = 0; j <= beta; ++j)
    //             cout << L_BC[i][j][k].time[0] << " " << L_BC[i][j][k].time[1] << " " << L_BC[i][j][k].table << " " << L_BC[i][j][k].lane << " || ";
    //         cout << endl;
    //     }
    //     cout << endl;
    // }

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
        if (lanes == "X")
        {
            stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
            --i;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
            --j;
        }
        else if (lanes == "XY")
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
        if (lanes == "X")
        {
            stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
            --i;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
            --k;
        }
        else if (lanes == "XY")
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
        if (lanes == "X")
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            --j;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            --j;
        }
        else if (lanes == "XY")
        {
            stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            j -= 2;
        }
        else if (lanes == "YX")
        {
            stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            j -= 2;
        }
        else if (lanes == "XX")
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            if (table == "AB")
                stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff)));
            else if (table == "BB")
                stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same)));
            j -= 2;
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
            j -= 2;
        }
        else if (lanes[0] == 'X')
        {
            stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
            --j;
        }
        else if (lanes[1] == 'Y')
        {
            stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
            --j;
        }
    }
    else if (optTable == "BC")
    {
        table = L_BC[i][j][k].table;
        lanes = L_BC[i][j][k].lane;
        if (lanes == "X")
        {
            stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
            --j;
        }
        else if (lanes == "Y")
        {
            stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
            --k;
        }
        else if (lanes == "XY")
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
        if (i < 0 || j < 0 || k < 0)
            exit(1);
        // cout << i << " " << j << " " << k << " ";
        // cout << table << " " << lanes << endl;
        if (table == "AB")
        {
            table = L_AB[i][j][k].table;
            lanes = L_AB[i][j][k].lane;
            if (lanes == "X")
            {
                stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
                --i;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
                --j;
            }
            else if (lanes == "XY")
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
            if (lanes == "X")
            {
                stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
                --i;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
                --k;
            }
            else if (lanes == "XY")
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
            if (lanes == "X")
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                --j;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                --j;
            }
            else if (lanes == "XY")
            {
                stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                j -= 2;
            }
            else if (lanes == "YX")
            {
                stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                j -= 2;
            }
            else if (lanes == "XX")
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                if (table == "AB")
                    stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff)));
                else if (table == "BB")
                    stack_X.push(make_tuple('B', j - 1, max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same)));
                j -= 2;
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
                j -= 2;
            }
            else if (lanes[0] == 'X')
            {
                stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
                --j;
            }
            else if (lanes[1] == 'Y')
            {
                stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
                --j;
            }
        }
        else if (table == "BC")
        {
            table = L_BC[i][j][k].table;
            lanes = L_BC[i][j][k].lane;
            if (lanes == "X")
            {
                stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
                --j;
            }
            else if (lanes == "Y")
            {
                stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
                --k;
            }
            else if (lanes == "XY")
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
    // cout << "Lane X: " << endl;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    last_X = stack_X.top();
    // cout << "Lane Y: " << endl;
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    last_Y = stack_Y.top();

    // Cauculate computation time
    auto t_end = chrono::high_resolution_clock::now();
    double computeTime = chrono::duration_cast<chrono::nanoseconds>(t_end - t_start).count();
    computeTime *= 1e-9;

    return make_tuple(last_X, last_Y, computeTime);
}

tuple<char, int, float> schedule_single_lane(char lane, vector<float> traffic, float W_same, float W_diff, tuple<char, int, float> prev)
{
    char prevLane = get<0>(prev);
    float prevTime = get<2>(prev);
    tuple<char, int, float> last = make_tuple('0', 0, 0.0);
    if (prevLane == '0')
    {
        last = make_tuple(lane, 1, traffic[1]);
    }
    else if (lane == prevLane)
    {
        last = make_tuple(lane, 1, max(traffic[1], prevTime + W_same));
    }
    else
    {
        last = make_tuple(lane, 1, max(traffic[1], prevTime + W_diff));
    }
    for (int i = 2; i < traffic.size(); ++i)
    {
        prevTime = get<2>(last);
        last = make_tuple(lane, i, max(traffic[i], prevTime + W_same));
    }
    return last;
}

vector<float> get_window_by_num(vector<float> &traffic, int carNum)
{
    if (carNum >= traffic.size())
        carNum = traffic.size() - 1;
    vector<float> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);
    return subtraffic;
}

tuple<float, double> schedule_by_num_window_v1(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff, int carNum)
{
    tuple<char, int, float> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, float> last_Y = make_tuple('0', 0, 0.0);
    double tmp;
    float T_last;
    auto t0 = chrono::high_resolution_clock::now();
    // while ((a_all.size() > 1 && b_all.size() > 1) ||
    //        (b_all.size() > 1 && c_all.size() > 1) ||
    //        (b_all.size() > 2))
    while (a_all.size() > 1 || b_all.size() > 1 || c_all.size() > 1)
    {
        vector<float> a = get_window_by_num(a_all, carNum);
        vector<float> b = get_window_by_num(b_all, carNum);
        vector<float> c = get_window_by_num(c_all, carNum);
        if (a.size() > 1 && b.size() > 1 && c.size() > 1)
            tie(last_X, last_Y, tmp) = window_oneSol_dp_v1(a, b, c, W_same, W_diff, last_X, last_Y);
        else if (a.size() > 1 && b.size() > 1)
        {
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('B', b, W_same, W_diff, last_Y);
        }
        else if (a.size() > 1 and c.size() > 1)
        {
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        }
        else if (b.size() > 1 and c.size() > 1)
        {
            last_X = schedule_single_lane('B', b, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        }
        else if (a.size() > 1)
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
        else if (c.size() > 1)
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        else if (b.size() > 1)
        {
            if (get<2>(last_X) < get<2>(last_Y))
            {
                if (get<0>(last_X) == '0')
                    last_X = make_tuple('B', 1, b[1]);
                else if (get<0>(last_X) == 'A')
                    last_X = make_tuple('B', 1, max(b[1], get<2>(last_X) + W_diff));
                else
                    last_X = make_tuple('B', 1, max(b[1], get<2>(last_X) + W_same));
                if (b.size() > 2)
                {
                    if (get<0>(last_Y) == '0')
                        last_Y = make_tuple('B', 2, b[2]);
                    else if (get<0>(last_Y) == 'C')
                        last_Y = make_tuple('B', 2, max(b[2], get<2>(last_Y) + W_diff));
                    else
                        last_Y = make_tuple('B', 2, max(b[2], get<2>(last_Y) + W_same));
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                            last_X = make_tuple('B', i, max(b[i], get<2>(last_X) + W_same));
                        else
                            last_Y = make_tuple('B', i, max(b[i], get<2>(last_Y) + W_same));
                    }
                }
            }
            else
            {
                if (get<0>(last_Y) == '0')
                    last_Y = make_tuple('B', 1, b[1]);
                else if (get<0>(last_Y) == 'C')
                    last_Y = make_tuple('B', 1, max(b[1], get<2>(last_Y) + W_diff));
                else
                    last_Y = make_tuple('B', 1, max(b[1], get<2>(last_Y) + W_same));
                if (b.size() > 2)
                {
                    if (get<0>(last_X) == '0')
                        last_X = make_tuple('B', 2, b[2]);
                    else if (get<0>(last_X) == 'A')
                        last_X = make_tuple('B', 2, max(b[2], get<2>(last_X) + W_diff));
                    else
                        last_X = make_tuple('B', 2, max(b[2], get<2>(last_X) + W_same));
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                            last_Y = make_tuple('B', i, max(b[i], get<2>(last_Y) + W_same));
                        else
                            last_X = make_tuple('B', i, max(b[i], get<2>(last_X) + W_same));
                    }
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    // cout << "Time taken by program is : " << fixed
    //      << totalComputeTime << setprecision(9);
    // cout << " sec" << endl;
    T_last = (get<2>(last_X) >= get<2>(last_Y)) ? get<2>(last_X) : get<2>(last_Y);
    cout << "v1 result: " << T_last << " " << totalComputeTime << endl;
    return make_tuple(T_last, totalComputeTime);
}

tuple<float, double> schedule_by_num_window_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff, int carNum)
{
    tuple<char, int, float> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, float> last_Y = make_tuple('0', 0, 0.0);
    double tmp;
    float T_last;
    auto t0 = chrono::high_resolution_clock::now();
    // while ((a_all.size() > 1 && b_all.size() > 1) ||
    //        (b_all.size() > 1 && c_all.size() > 1) ||
    //        (b_all.size() > 2))
    while (a_all.size() > 1 || b_all.size() > 1 || c_all.size() > 1)
    {
        vector<float> a = get_window_by_num(a_all, carNum);
        vector<float> b = get_window_by_num(b_all, carNum);
        vector<float> c = get_window_by_num(c_all, carNum);
        if (a.size() > 1 && b.size() > 1 && c.size() > 1)
            tie(last_X, last_Y, tmp) = window_oneSol_dp_v2(a, b, c, W_same, W_diff, last_X, last_Y);
        else if (a.size() > 1 && b.size() > 1)
        {
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('B', b, W_same, W_diff, last_Y);
        }
        else if (a.size() > 1 and c.size() > 1)
        {
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        }
        else if (b.size() > 1 and c.size() > 1)
        {
            last_X = schedule_single_lane('B', b, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        }
        else if (a.size() > 1)
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
        else if (c.size() > 1)
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        else if (b.size() > 1)
        {
            if (get<2>(last_X) < get<2>(last_Y))
            {
                if (get<0>(last_X) == '0')
                    last_X = make_tuple('B', 1, b[1]);
                else if (get<0>(last_X) == 'A')
                    last_X = make_tuple('B', 1, max(b[1], get<2>(last_X) + W_diff));
                else
                    last_X = make_tuple('B', 1, max(b[1], get<2>(last_X) + W_same));
                if (b.size() > 2)
                {
                    if (get<0>(last_Y) == '0')
                        last_Y = make_tuple('B', 2, b[2]);
                    else if (get<0>(last_Y) == 'C')
                        last_Y = make_tuple('B', 2, max(b[2], get<2>(last_Y) + W_diff));
                    else
                        last_Y = make_tuple('B', 2, max(b[2], get<2>(last_Y) + W_same));
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                            last_X = make_tuple('B', i, max(b[i], get<2>(last_X) + W_same));
                        else
                            last_Y = make_tuple('B', i, max(b[i], get<2>(last_Y) + W_same));
                    }
                }
            }
            else
            {
                if (get<0>(last_Y) == '0')
                    last_Y = make_tuple('B', 1, b[1]);
                else if (get<0>(last_Y) == 'C')
                    last_Y = make_tuple('B', 1, max(b[1], get<2>(last_Y) + W_diff));
                else
                    last_Y = make_tuple('B', 1, max(b[1], get<2>(last_Y) + W_same));
                if (b.size() > 2)
                {
                    if (get<0>(last_X) == '0')
                        last_X = make_tuple('B', 2, b[2]);
                    else if (get<0>(last_X) == 'A')
                        last_X = make_tuple('B', 2, max(b[2], get<2>(last_X) + W_diff));
                    else
                        last_X = make_tuple('B', 2, max(b[2], get<2>(last_X) + W_same));
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                            last_Y = make_tuple('B', i, max(b[i], get<2>(last_Y) + W_same));
                        else
                            last_X = make_tuple('B', i, max(b[i], get<2>(last_X) + W_same));
                    }
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    // cout << "Time taken by program is : " << fixed
    //      << totalComputeTime << setprecision(9);
    // cout << " sec" << endl;
    T_last = (get<2>(last_X) >= get<2>(last_Y)) ? get<2>(last_X) : get<2>(last_Y);
    cout << "v2 result: " << T_last << " " << totalComputeTime << endl;
    return make_tuple(T_last, totalComputeTime);
}

tuple<float, double> first_come_first_serve_v1(float timeStep, vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<float> a(a_all.begin() + 1, a_all.end());
    vector<float> b(b_all.begin() + 1, b_all.end());
    vector<float> c(c_all.begin() + 1, c_all.end());
    float t = 0;         // time
    char B_prevTo = 'Y'; // Which lane the previous vehicle in lane B go to
    float X_lastT = -W_diff;
    float Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';

    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        t = t + timeStep;
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            if (a[0] == t && b[0] == t && c[0] == t)
            {
                if (a.size() > 1 && b.size() > 1 && c.size() > 1)
                {
                    if (b[1] >= a[1] && b[1] >= c[1])
                    {
                        if (X_lastFrom == 'A')
                            X_lastT = max(a[0], X_lastT + W_same);
                        else
                            X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';

                        if (Y_lastFrom == 'C')
                            Y_lastT = max(c[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';

                        if (X_lastT <= t && Y_lastT <= t)
                        {
                            if (a[0] >= c[0])
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                        }
                        else
                        {
                            if (X_lastT < Y_lastT)
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else if (Y_lastT < X_lastT)
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (a[0] >= c[0])
                                {
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                    b.erase(b.begin());
                                    X_lastFrom = 'B';
                                }
                                else
                                {
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                    b.erase(b.begin());
                                    Y_lastFrom = 'B';
                                }
                            }
                        }
                    }
                    else if (a[1] >= b[1] && a[1] >= c[1])
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';

                        if (Y_lastFrom == 'C')
                            Y_lastT = max(c[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';

                        X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';
                    }
                    else if (c[1] >= a[1] && c[1] >= b[1])
                    {
                        if (X_lastFrom == 'A')
                            X_lastT = max(a[0], X_lastT + W_same);
                        else
                            X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';

                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';

                        Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';
                    }
                }
                else if (b.size() < 2)
                {
                    if (X_lastFrom == 'A')
                        X_lastT = max(a[0], X_lastT + W_same);
                    else
                        X_lastT = max(a[0], X_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';

                    if (Y_lastFrom == 'C')
                        Y_lastT = max(c[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';

                    if (X_lastT <= t && Y_lastT <= t)
                    {
                        if (a.size() < 1)
                        {
                            X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (c.size() < 1)
                        {
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (a[0] >= c[0])
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                        }
                    }
                    else
                    {
                        if (X_lastT < Y_lastT)
                        {
                            X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (Y_lastT < X_lastT)
                        {
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (a.size() < 1)
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else if (c.size() < 1)
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (a[0] >= c[0])
                                {
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                    b.erase(b.begin());
                                    X_lastFrom = 'B';
                                }
                                else
                                {
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                    b.erase(b.begin());
                                    Y_lastFrom = 'B';
                                }
                            }
                        }
                    }
                }
                else if (a.size() < 2)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    b.erase(b.begin());
                    X_lastFrom = 'B';

                    if (Y_lastFrom == 'C')
                        Y_lastT = max(c[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';

                    X_lastT = max(a[0], Y_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';
                }
                else if (c.size() < 2)
                {
                    if (X_lastFrom == 'A')
                        X_lastT = max(a[0], X_lastT + W_same);
                    else
                        X_lastT = max(a[0], X_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';

                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';

                    Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';
                }
            }
            else if (a[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (a[0] == t && b[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
            }
            else if (b[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'B')
                    X_lastT = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (c[0] > a[0])
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                }
                else
                {
                    if (X_lastT <= t && Y_lastT <= t)
                    {
                        if (c[0] > a[0])
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                    }
                    else
                    {
                        if (X_lastT < Y_lastT)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (Y_lastT < X_lastT)
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (c[0] > a[0])
                            {
                                if (Y_lastFrom == 'B')
                                    Y_lastT = max(b[0], Y_lastT + W_same);
                                else
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (X_lastFrom == 'B')
                                    X_lastT = max(b[0], X_lastT + W_same);
                                else
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                        }
                    }
                }
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
        }
        else if (a.size() > 0 && c.size() > 0)
        {
            if (a[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
        }
        else if (a.size() > 0 && b.size() > 0)
        {
            if (a[0] == t && b[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    b.erase(b.begin());
                    Y_lastFrom = 'B';
                }
                else
                {
                    if (X_lastT < Y_lastT)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                    else if (Y_lastT < X_lastT)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                }
            }
        }
        else if (b.size() > 0 && c.size() > 0)
        {
            if (b[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'B')
                    X_lastFrom = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    b.erase(b.begin());
                    X_lastFrom = 'B';
                }
                else
                {
                    if (X_lastT < Y_lastT)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                    else if (Y_lastT < X_lastT)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                }
            }
        }
        else if (a.size() > 0)
        {
            if (X_lastFrom == 'A')
                X_lastT = max(a[0], X_lastT + W_same);
            else
                X_lastT = max(a[0], X_lastT + W_diff);
            a.erase(a.begin());
            X_lastFrom = 'A';
        }
        else if (c.size() > 0)
        {
            if (Y_lastFrom == 'C')
                Y_lastT = max(c[0], Y_lastT + W_same);
            else
                Y_lastT = max(c[0], Y_lastT + W_diff);
            c.erase(c.begin());
            Y_lastFrom = 'C';
        }
        else if (b.size() > 0)
        {
            if (B_prevTo == 'Y')
            {
                if (X_lastFrom == 'B')
                    X_lastT = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';
                B_prevTo = 'X';
            }
            else
            {
                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
                B_prevTo = 'Y';
            }
        }
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    float T_last = max(X_lastT, Y_lastT);
    cout << "result: " << T_last << " " << totalComputeTime << endl;
    return make_tuple(T_last, totalComputeTime);
}

tuple<float, double> first_come_first_serve_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<float> a(a_all.begin() + 1, a_all.end());
    vector<float> b(b_all.begin() + 1, b_all.end());
    vector<float> c(c_all.begin() + 1, c_all.end());
    float X_lastT = -W_diff;
    float Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';
    float first = 0;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    int tmp;

    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            first = min({a[0], b[0], c[0]});
            if (first == a[0])
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                X_lastFrom = 'A';
                a.erase(a.begin());
            }
            if (first == c[0])
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                Y_lastFrom = 'C';
                c.erase(c.begin());
            }
            if (first == b[0])
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    X_lastFrom = 'B';
                    b.erase(b.begin());
                }
                else if (Y_lastT < X_lastT)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    b.erase(b.begin());
                }
                else
                {
                    if (a[0] > c[0])
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        X_lastFrom = 'B';
                        b.erase(b.begin());
                    }
                    else if (c[0] > a[0])
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        Y_lastFrom = 'B';
                        b.erase(b.begin());
                    }
                    else
                    {
                        if (distribution(generator) == 0)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            X_lastFrom = 'B';
                            b.erase(b.begin());
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                            b.erase(b.begin());
                        }
                    }
                }
            }
        }
        else if (a.size() > 1 && b.size() > 1)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('A', a, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('B', b, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            a.clear();
            b.clear();
        }
        else if (a.size() > 0 and c.size() > 0)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('A', a, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('C', c, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            a.clear();
            c.clear();
        }
        else if (b.size() > 0 and c.size() > 0)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('B', b, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('C', c, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            b.clear();
            c.clear();
        }
        else if (a.size() > 0)
        {
            tie(X_lastFrom, tmp, X_lastT) = schedule_single_lane('A', a, W_same, W_diff, make_tuple(X_lastFrom, 0, X_lastT));
            a.clear();
        }
        else if (c.size() > 0)
        {
            tie(Y_lastFrom, tmp, Y_lastT) = schedule_single_lane('C', c, W_same, W_diff, make_tuple(Y_lastFrom, 0, Y_lastT));
            c.clear();
        }
        else if (b.size() > 0)
        {
            while (b.size() > 0)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    X_lastFrom = 'B';
                    b.erase(b.begin());
                }
                else
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    b.erase(b.begin());
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    float T_last = max(X_lastT, Y_lastT);
    cout << "result: " << T_last << " " << totalComputeTime << endl;
    return make_tuple(T_last, totalComputeTime);
}

tuple<float, double> greedy_dp(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff)
{
    auto t0 = chrono::high_resolution_clock::now();
    int alpha = a_all.size() - 1;
    int beta = b_all.size() - 1;
    int gamma = c_all.size() - 1;
    vector<vector<GreedySol>> LX_A, LX_B, LY_C, LY_B;
    int beta_sum = 1;
    int beta_X = 1, beta_Y = 1;
    float min_X, min_Y;
    float T_last;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);

    LX_A.resize(alpha + 1, vector<GreedySol>(beta + 1));
    LX_B.resize(alpha + 1, vector<GreedySol>(beta + 1));
    LY_C.resize(gamma + 1, vector<GreedySol>(beta + 1));
    LY_B.resize(gamma + 1, vector<GreedySol>(beta + 1));

    // Initialize
    LX_A[0][0] = update_greedySol(LX_A[0][0], 0, '0');
    LX_B[0][0] = update_greedySol(LX_B[0][0], 0, '0');
    LX_A[1][0] = update_greedySol(LX_A[1][0], a_all[1], '0');
    LY_C[0][0] = update_greedySol(LY_C[0][0], 0, '0');
    LY_B[0][0] = update_greedySol(LY_B[0][0], 0, '0');
    LY_C[1][0] = update_greedySol(LY_C[1][0], c_all[1], '0');
    for (int i = 2; i <= alpha; ++i)
    {
        LX_A[i][0] = update_greedySol(LX_A[i][0], max(a_all[i], LX_A[i - 1][0].time + W_same), 'A');
    }
    for (int i = 2; i <= gamma; ++i)
    {
        LY_C[i][0] = update_greedySol(LY_C[i][0], max(c_all[i], LY_C[i - 1][0].time + W_same), 'C');
    }

    while (beta_sum <= beta)
    {
        if (beta_X == 1)
            LX_B[0][beta_X] = update_greedySol(LX_B[0][beta_X], b_all[beta_sum], '0');
        else
            LX_B[0][beta_X] = update_greedySol(LX_B[0][beta_X], max(b_all[beta_sum], LX_B[0][beta_X - 1].time + W_same), 'B');

        if (beta_Y == 1)
            LY_B[0][beta_Y] = update_greedySol(LY_B[0][beta_Y], b_all[beta_sum], '0');
        else
            LY_B[0][beta_Y] = update_greedySol(LY_B[0][beta_Y], max(b_all[beta_sum], LY_B[0][beta_Y - 1].time + W_same), 'B');

        for (int i = 1; i <= alpha; ++i)
        {
            if (max(a_all[i], LX_A[i - 1][beta_X].time + W_same) <= max(a_all[i], LX_B[i - 1][beta_X].time + W_diff))
            {
                LX_A[i][beta_X] = update_greedySol(LX_A[i][beta_X], max(a_all[i], LX_A[i - 1][beta_X].time + W_same), 'A');
            }
            else
            {
                LX_A[i][beta_X] = update_greedySol(LX_A[i][beta_X], max(a_all[i], LX_B[i - 1][beta_X].time + W_diff), 'B');
            }
            if (max(b_all[beta_sum], LX_A[i][beta_X - 1].time + W_diff) <= max(b_all[beta_sum], LX_B[i][beta_X - 1].time + W_same))
            {
                LX_B[i][beta_X] = update_greedySol(LX_B[i][beta_X], max(b_all[beta_sum], LX_A[i][beta_X - 1].time + W_diff), 'A');
            }
            else
            {
                LX_B[i][beta_X] = update_greedySol(LX_B[i][beta_X], max(b_all[beta_sum], LX_B[i][beta_X - 1].time + W_same), 'B');
            }
        }
        for (int i = 1; i <= gamma; ++i)
        {
            if (max(c_all[i], LY_C[i - 1][beta_Y].time + W_same) <= max(c_all[i], LY_B[i - 1][beta_Y].time + W_diff))
            {
                LY_C[i][beta_Y] = update_greedySol(LY_C[i][beta_Y], max(c_all[i], LY_C[i - 1][beta_Y].time + W_same), 'C');
            }
            else
            {
                LY_C[i][beta_Y] = update_greedySol(LY_C[i][beta_Y], max(c_all[i], LY_B[i - 1][beta_Y].time + W_diff), 'B');
            }
            if (max(b_all[beta_sum], LY_C[i][beta_Y - 1].time + W_diff) <= max(b_all[beta_sum], LY_B[i][beta_Y - 1].time + W_same))
            {
                LY_B[i][beta_Y] = update_greedySol(LY_B[i][beta_Y], max(b_all[beta_sum], LY_C[i][beta_Y - 1].time + W_diff), 'C');
            }
            else
            {
                LY_B[i][beta_Y] = update_greedySol(LY_B[i][beta_Y], max(b_all[beta_sum], LY_B[i][beta_Y - 1].time + W_same), 'B');
            }
        }
        min_X = min(LX_A[alpha][beta_X].time, LX_B[alpha][beta_X].time);
        min_Y = min(LY_C[gamma][beta_Y].time, LY_B[gamma][beta_Y].time);
        if (min_X < min_Y)
            ++beta_X;
        else if (min_Y < min_X)
            ++beta_Y;
        else
        {
            if (distribution(generator) == 0)
                ++beta_X;
            else
                ++beta_Y;
        }
        ++beta_sum;
    }

    // Choose optimal solution for Lane X
    --beta_X;
    --beta_Y;
    stack<tuple<char, int, float>> stack_X, stack_Y;
    char prevTable;
    int i = alpha;
    int j = beta_X;
    if (LX_A[i][j].time <= LX_B[i][j].time)
    {
        stack_X.push(make_tuple('A', i, LX_A[i][j].time));
        prevTable = LX_A[i][j].table;
        --i;
    }
    else
    {
        stack_X.push(make_tuple('B', j, LX_B[i][j].time));
        prevTable = LX_B[i][j].table;
        --j;
    }
    // Backtracking LX
    while (i > 0 || j > 0)
    {
        if (prevTable == 'A')
        {
            stack_X.push(make_tuple('A', i, LX_A[i][j].time));
            prevTable = LX_A[i][j].table;
            --i;
        }
        else if (prevTable == 'B')
        {
            stack_X.push(make_tuple('B', j, LX_B[i][j].time));
            prevTable = LX_B[i][j].table;
            --j;
        }
        else
        {
            cout << "BUG " << i << " " << j << " prevTable = " << prevTable << endl;
        }
    }
    // Output order for Lane X
    // cout << "Lane X: " << endl;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        stack_X.pop();
    }
    T_last = get<2>(stack_X.top());

    // Choose optimal solution for Lane Y
    i = gamma;
    j = beta_Y;
    if (LY_C[i][j].time <= LY_B[i][j].time)
    {
        stack_Y.push(make_tuple('C', i, LY_C[i][j].time));
        prevTable = LY_C[i][j].table;
        --i;
    }
    else
    {
        stack_Y.push(make_tuple('B', j, LY_B[i][j].time));
        prevTable = LY_B[i][j].table;
        --j;
    }
    // Backtracking LY
    while (i > 0 || j > 0)
    {
        if (prevTable == 'C')
        {
            stack_Y.push(make_tuple('C', i, LY_C[i][j].time));
            prevTable = LY_C[i][j].table;
            --i;
        }
        else if (prevTable == 'B')
        {
            stack_Y.push(make_tuple('B', j, LY_B[i][j].time));
            prevTable = LY_B[i][j].table;
            --j;
        }
        else
        {
            cout << "BUG " << i << " " << j << " prevTable = " << prevTable << endl;
        }
    }
    // Output order for Lane Y
    // cout << "Lane Y: " << endl;
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        stack_Y.pop();
    }
    if (get<2>(stack_Y.top()) > T_last)
        T_last = get<2>(stack_Y.top());

    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    cout << "result: " << T_last << " " << totalComputeTime << endl;
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

    a_all = generate_traffic(timeStep, alpha, p, 0);
    b_all = generate_traffic(timeStep, beta, p, 1);
    c_all = generate_traffic(timeStep, gamma, p, 2);

    first_come_first_serve_v1(timeStep, a_all, b_all, c_all, W_same, W_diff);
    first_come_first_serve_v2(a_all, b_all, c_all, W_same, W_diff);
    schedule_by_num_window_v1(a_all, b_all, c_all, W_same, W_diff, 5);
    schedule_by_num_window_v1(a_all, b_all, c_all, W_same, W_diff, 10);
    schedule_by_num_window_v1(a_all, b_all, c_all, W_same, W_diff, 20);
    schedule_by_num_window_v1(a_all, b_all, c_all, W_same, W_diff, 100);
    schedule_by_num_window_v2(a_all, b_all, c_all, W_same, W_diff, 5);
    schedule_by_num_window_v2(a_all, b_all, c_all, W_same, W_diff, 10);
    schedule_by_num_window_v2(a_all, b_all, c_all, W_same, W_diff, 20);
    schedule_by_num_window_v2(a_all, b_all, c_all, W_same, W_diff, 100);
    greedy_dp(a_all, b_all, c_all, W_same, W_diff);

    return 0;
}
