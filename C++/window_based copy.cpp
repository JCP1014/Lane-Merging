#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <tuple>
#include <time.h>
#include <stack>
#include <chrono>
#include "generate_input.h"
#include "fcfs.h"
#include "dp_2d.h"
#define endl '\n'
using namespace std;

struct Solution
{
    float time[2] = {INFINITY, INFINITY};
    string table = "";
    string lane = "";
    Solution *src = NULL;
};

struct Lane
{
    int num;
    vector<float> traffic;
};

Solution update_sol(Solution s, float newTimeX, float newTimeY, string newTable, string newLane)
{
    s.time[0] = newTimeX;
    s.time[1] = newTimeY;
    s.table = newTable;
    s.lane = newLane;
    return s;
}

Solution update_sol(Solution s, float newTimeX, float newTimeY, string newTable, string newLane, Solution *newSrc)
{
    s.time[0] = newTimeX;
    s.time[1] = newTimeY;
    s.table = newTable;
    s.lane = newLane;
    s.src = newSrc;
    return s;
}

Solution choose_best_sol(Solution s, vector<Solution> solVec)
{
    float minMax = max(solVec[0].time[0], solVec[0].time[1]);
    float minSum;
    float minSrcSum, minSrcMax;
    int index = 0;
    float tmpMax, tmpMin, tmpSum, tmpSrcMax, tmpSrcMin, tmpSrcSum;

    for (int i = 1; i < solVec.size(); ++i)
    {
        tmpMax = max(solVec[i].time[0], solVec[i].time[1]);
        if (tmpMax < minMax)
        {
            minMax = tmpMax;
            index = i;
        }
        else if (tmpMax == minMax)
        {
            tmpSum = solVec[i].time[0] + solVec[i].time[1];
            minSum = solVec[index].time[0] + solVec[index].time[1];
            if (tmpSum < minSum)
            {
                minMax = tmpMax;
                index = i;
            }
            else if (tmpSum == minSum && solVec[i].src)
            {
                tmpSrcMax = max(solVec[i].src->time[0], solVec[i].src->time[1]);
                minSrcMax = max(solVec[index].src->time[0], solVec[index].src->time[1]);
                if (tmpSrcMax < minSrcMax)
                {
                    minMax = tmpMax;
                    index = i;
                }
                else if (tmpSrcMax == minSrcMax)
                {
                    tmpSrcSum = solVec[i].src->time[0] + solVec[i].src->time[1];
                    minSrcSum = solVec[index].src->time[0] + solVec[index].src->time[1];
                    if (tmpSrcSum < minSrcSum)
                    {
                        minMax = tmpMax;
                        index = i;
                    }
                }
            }
        }
    }
    s = update_sol(s, solVec[index].time[0], solVec[index].time[1], solVec[index].table, solVec[index].lane);
    return s;
}

string get_opt_table(vector<Solution> solVec)
{
    float minMax = max(solVec[0].time[0], solVec[0].time[1]);
    float minSum = solVec[0].time[0] + solVec[0].time[1];
    int index = 0;
    float tmpMax, tmpMin, tmpSum;
    string optTable = "AB";
    for (int i = 1; i < solVec.size(); ++i)
    {
        tmpMax = max(solVec[i].time[0], solVec[i].time[1]);
        tmpSum = solVec[i].time[0] + solVec[i].time[1];
        if (tmpMax < minMax)
        {
            minMax = tmpMax;
            minSum = tmpSum;
            index = i;
        }
        else if (tmpMax == minMax)
        {
            if (tmpSum < minSum)
            {
                minMax = tmpMax;
                minSum = tmpSum;
                index = i;
            }
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
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X0");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "0Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
            L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], b[1], max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], b[2], max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(4);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j - 1][0].time[0] + W_same), max(b[j], L_AB[i - 1][j - 1][0].time[1] + W_same), "AB", "XY", &L_AB[i - 1][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], max(b[j], L_AB[i - 1][j - 1][0].time[0] + W_diff) + W_diff), L_AB[i - 1][j - 1][0].time[1], "AB", "XX", &L_AB[i - 1][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_BB[i - 1][j - 1][0].time[0] + W_diff), max(b[j], L_BB[i - 1][j - 1][0].time[1] + W_same), "BB", "XY", &L_BB[i - 1][j - 1][0]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], max(b[j], L_BB[i - 1][j - 1][0].time[0] + W_same) + W_diff), L_BB[i - 1][j - 1][0].time[1], "BB", "XX", &L_BB[i - 1][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
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
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BB[0][j - 1][k - 1].time[1] + W_diff), "BB", "XY", &L_BB[0][j - 1][k - 1]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BB[0][j - 1][k - 1].time[1] + W_same) + W_diff), "BB", "YY", &L_BB[0][j - 1][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], L_BC[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BC[0][j - 1][k - 1].time[1] + W_same), "BC", "XY", &L_BC[0][j - 1][k - 1]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BC[0][j - 1][k - 1].time[1] + W_diff) + W_diff), "BC", "YY", &L_BC[0][j - 1][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(6);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j], L_AB[i][j - 2][0].time[1] + W_same), "AB", "XY", &L_AB[i][j - 2][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][0].time[1] + W_same), "AB", "YX", &L_AB[i][j - 2][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same), max(b[j], L_BB[i][j - 2][0].time[1] + W_same), "BB", "XY", &L_BB[i][j - 2][0]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j], L_BB[i][j - 2][0].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same), "BB", "YX", &L_BB[i][j - 2][0]);
            tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same) + W_same), L_BB[i][j - 2][0].time[1], "BB", "XX", &L_BB[i][j - 2][0]);
            tmpSolVec[5] = update_sol(tmpSolVec[5], L_BB[i][j - 2][0].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same) + W_same), "BB", "YY", &L_BB[i][j - 2][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
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
                tmpSolVec.resize(6);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j - 1][k].time[0] + W_same), max(b[j], L_AB[i - 1][j - 1][k].time[1] + W_same), "AB", "XY", &L_AB[i - 1][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], max(b[j], L_AB[i - 1][j - 1][k].time[0] + W_diff) + W_diff), L_AB[i - 1][j - 1][k].time[1], "AB", "XX", &L_AB[i - 1][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_AC[i - 1][j - 1][k].time[0] + W_same), max(b[j], L_AC[i - 1][j - 1][k].time[1] + W_diff), "AC", "XY", &L_AC[i - 1][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], L_BB[i - 1][j - 1][k].time[0] + W_diff), max(b[j], L_BB[i - 1][j - 1][k].time[1] + W_same), "BB", "XY", &L_BB[i - 1][j - 1][k]);
                tmpSolVec[4] = update_sol(tmpSolVec[4], max(a[i], max(b[j], L_BB[i - 1][j - 1][k].time[0] + W_same) + W_diff), L_BB[i - 1][j - 1][k].time[1], "BB", "XX", &L_BB[i - 1][j - 1][k]);
                tmpSolVec[5] = update_sol(tmpSolVec[5], max(a[i], L_BC[i - 1][j - 1][k].time[0] + W_diff), max(b[j], L_BC[i - 1][j - 1][k].time[1] + W_diff), "BC", "XY", &L_BC[i - 1][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k - 1].time[0] + W_same), max(c[k], L_AB[i - 1][j][k - 1].time[1] + W_diff), "AB", "XY", &L_AB[i - 1][j][k - 1]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_AC[i - 1][j][k - 1].time[0] + W_same), max(c[k], L_AC[i - 1][j][k - 1].time[1] + W_same), "AC", "XY", &L_AC[i - 1][j][k - 1]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_BB[i - 1][j][k - 1].time[0] + W_diff), max(c[k], L_BB[i - 1][j][k - 1].time[1] + W_diff), "BB", "XY", &L_BB[i - 1][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], L_BC[i - 1][j][k - 1].time[0] + W_diff), max(c[k], L_BC[i - 1][j][k - 1].time[1] + W_same), "BC", "XY", &L_BC[i - 1][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(6);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_AB[i][j - 1][k - 1].time[0] + W_diff), max(c[k], L_AB[i][j - 1][k - 1].time[1] + W_diff), "AB", "XY", &L_AB[i][j - 1][k - 1]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k - 1].time[0] + W_diff), max(c[k], L_AC[i][j - 1][k - 1].time[1] + W_same), "AC", "XY", &L_AC[i][j - 1][k - 1]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], L_BB[i][j - 1][k - 1].time[0] + W_same), max(c[k], L_BB[i][j - 1][k - 1].time[1] + W_diff), "BB", "XY", &L_BB[i][j - 1][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BB[i][j - 1][k - 1].time[1] + W_same) + W_diff), "BB", "YY", &L_BB[i][j - 1][k - 1]);
                tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], L_BC[i][j - 1][k - 1].time[0] + W_same), max(c[k], L_BC[i][j - 1][k - 1].time[1] + W_same), "BC", "XY", &L_BC[i][j - 1][k - 1]);
                tmpSolVec[5] = update_sol(tmpSolVec[5], L_BC[i][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BC[i][j - 1][k - 1].time[1] + W_diff) + W_diff), "BC", "YY", &L_BC[i][j - 1][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                if (j >= 2)
                {
                    tmpSolVec.resize(12);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff), max(b[j], L_AB[i][j - 2][k].time[1] + W_same), "AB", "XY", &L_AB[i][j - 2][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][k].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][k].time[1] + W_same), "AB", "YX", &L_AB[i][j - 2][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], max(b[j - 1], L_AB[i][j - 2][k].time[0] + W_diff) + W_same), L_AB[i][j - 2][k].time[1], "AB", "XX", &L_AB[i][j - 2][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j - 1], L_AC[i][j - 2][k].time[0] + W_diff), max(b[j], L_AC[i][j - 2][k].time[1] + W_diff), "AC", "XY", &L_AC[i][j - 2][k]);
                    tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], L_AC[i][j - 2][k].time[0] + W_diff), max(b[j - 1], L_AC[i][j - 2][k].time[1] + W_diff), "AC", "YX", &L_AC[i][j - 2][k]);
                    tmpSolVec[5] = update_sol(tmpSolVec[5], max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same), max(b[j], L_BB[i][j - 2][k].time[1] + W_same), "BB", "XY", &L_BB[i][j - 2][k]);
                    tmpSolVec[6] = update_sol(tmpSolVec[6], max(b[j], L_BB[i][j - 2][k].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same), "BB", "YX", &L_BB[i][j - 2][k]);
                    tmpSolVec[7] = update_sol(tmpSolVec[7], max(b[j], max(b[j - 1], L_BB[i][j - 2][k].time[0] + W_same) + W_same), L_BB[i][j - 2][k].time[1], "BB", "XX", &L_BB[i][j - 2][k]);
                    tmpSolVec[8] = update_sol(tmpSolVec[8], L_BB[i][j - 2][k].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][k].time[1] + W_same) + W_same), "BB", "YY", &L_BB[i][j - 2][k]);
                    tmpSolVec[9] = update_sol(tmpSolVec[9], max(b[j - 1], L_BC[i][j - 2][k].time[0] + W_same), max(b[j], L_BC[i][j - 2][k].time[1] + W_diff), "BC", "XY", &L_BC[i][j - 2][k]);
                    tmpSolVec[10] = update_sol(tmpSolVec[10], max(b[j], L_BC[i][j - 2][k].time[0] + W_same), max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_diff), "BC", "YX", &L_BC[i][j - 2][k]);
                    tmpSolVec[11] = update_sol(tmpSolVec[11], L_BC[i][j - 2][k].time[0], max(b[j], max(b[j - 1], L_BC[i][j - 2][k].time[1] + W_diff) + W_same), "BC", "YY", &L_BC[i][j - 2][k]);
                    L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
            L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], b[1], max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], b[2], max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(4);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j - 1][0].time[0] + W_same), max(b[j], L_AB[i - 1][j - 1][0].time[1] + W_same), "AB", "XY", &L_AB[i - 1][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], max(b[j], L_AB[i - 1][j - 1][0].time[0] + W_diff) + W_diff), L_AB[i - 1][j - 1][0].time[1], "AB", "XX", &L_AB[i - 1][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_BB[i - 1][j - 1][0].time[0] + W_diff), max(b[j], L_BB[i - 1][j - 1][0].time[1] + W_same), "BB", "XY", &L_BB[i - 1][j - 1][0]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], max(b[j], L_BB[i - 1][j - 1][0].time[0] + W_same) + W_diff), L_BB[i - 1][j - 1][0].time[1], "BB", "XX", &L_BB[i - 1][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
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
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BB[0][j - 1][k - 1].time[1] + W_diff), "BB", "XY", &L_BB[0][j - 1][k - 1]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BB[0][j - 1][k - 1].time[1] + W_same) + W_diff), "BB", "YY", &L_BB[0][j - 1][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], L_BC[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BC[0][j - 1][k - 1].time[1] + W_same), "BC", "XY", &L_BC[0][j - 1][k - 1]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BC[0][j - 1][k - 1].time[1] + W_diff) + W_diff), "BC", "YY", &L_BC[0][j - 1][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(6);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j], L_AB[i][j - 2][0].time[1] + W_same), "AB", "XY", &L_AB[i][j - 2][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][0].time[1] + W_same), "AB", "YX", &L_AB[i][j - 2][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same), max(b[j], L_BB[i][j - 2][0].time[1] + W_same), "BB", "XY", &L_BB[i][j - 2][0]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j], L_BB[i][j - 2][0].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same), "BB", "YX", &L_BB[i][j - 2][0]);
            tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same) + W_same), L_BB[i][j - 2][0].time[1], "BB", "XX", &L_BB[i][j - 2][0]);
            tmpSolVec[5] = update_sol(tmpSolVec[5], L_BB[i][j - 2][0].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same) + W_same), "BB", "YY", &L_BB[i][j - 2][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
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
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
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

tuple<tuple<char, int, float>, tuple<char, int, float>, double> reduced_dp(vector<float> a, vector<float> b, vector<float> c, float W_same, float W_diff, tuple<char, int, float> last_X, tuple<char, int, float> last_Y)
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_same), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_same), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
        if (beta >= 2)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], T_Y + W_diff), "BB", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], T_Y + W_diff), "BB", "YX");
            L_BB[0][2][0] = choose_best_sol(L_BB[0][2][0], tmpSolVec);
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
            L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_diff), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[2], T_X + W_same), max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
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
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], b[1], max(b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], b[2], max(b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(4);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j - 1][0].time[0] + W_same), max(b[j], L_AB[i - 1][j - 1][0].time[1] + W_same), "AB", "XY", &L_AB[i - 1][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], max(b[j], L_AB[i - 1][j - 1][0].time[0] + W_diff) + W_diff), L_AB[i - 1][j - 1][0].time[1], "AB", "XX", &L_AB[i - 1][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(a[i], L_BB[i - 1][j - 1][0].time[0] + W_diff), max(b[j], L_BB[i - 1][j - 1][0].time[1] + W_same), "BB", "XY", &L_BB[i - 1][j - 1][0]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(a[i], max(b[j], L_BB[i - 1][j - 1][0].time[0] + W_same) + W_diff), L_BB[i - 1][j - 1][0].time[1], "BB", "XX", &L_BB[i - 1][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
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
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BB[0][j - 1][k - 1].time[1] + W_diff), "BB", "XY", &L_BB[0][j - 1][k - 1]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BB[0][j - 1][k - 1].time[1] + W_same) + W_diff), "BB", "YY", &L_BB[0][j - 1][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j], L_BC[0][j - 1][k - 1].time[0] + W_same), max(c[k], L_BC[0][j - 1][k - 1].time[1] + W_same), "BC", "XY", &L_BC[0][j - 1][k - 1]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[0][j - 1][k - 1].time[0], max(c[k], max(b[j], L_BC[0][j - 1][k - 1].time[1] + W_diff) + W_diff), "BC", "YY", &L_BC[0][j - 1][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(6);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j - 1], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j], L_AB[i][j - 2][0].time[1] + W_same), "AB", "XY", &L_AB[i][j - 2][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 2][0].time[0] + W_diff), max(b[j - 1], L_AB[i][j - 2][0].time[1] + W_same), "AB", "YX", &L_AB[i][j - 2][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same), max(b[j], L_BB[i][j - 2][0].time[1] + W_same), "BB", "XY", &L_BB[i][j - 2][0]);
            tmpSolVec[3] = update_sol(tmpSolVec[3], max(b[j], L_BB[i][j - 2][0].time[0] + W_same), max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same), "BB", "YX", &L_BB[i][j - 2][0]);
            tmpSolVec[4] = update_sol(tmpSolVec[4], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[0] + W_same) + W_same), L_BB[i][j - 2][0].time[1], "BB", "XX", &L_BB[i][j - 2][0]);
            tmpSolVec[5] = update_sol(tmpSolVec[5], L_BB[i][j - 2][0].time[0], max(b[j], max(b[j - 1], L_BB[i][j - 2][0].time[1] + W_same) + W_same), "BB", "YY", &L_BB[i][j - 2][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    // Compute tables
    bool isCut = false;
    int N = max(max(alpha, beta), max(beta, gamma));
    int cut_i = alpha, cut_j = beta, cut_k = gamma;

    // Find the first cut
    for (int square = 1; square <= N; ++square)
    {
        for (int k = 1; k < square && k <= gamma; ++k)
        {
            for (int i = square, j = 1; j <= square && i <= alpha && j <= beta; ++j)
            {
                // L_AB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_AB[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_AB[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_AB[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_AB[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_AC[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_AC[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_AC[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_AC[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_BC[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_BC[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_BC[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_BC[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_BB[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_BB[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_BB[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_BB[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                if (isCut)
                {
                    cut_i = i;
                    cut_j = j;
                    cut_k = k;
                    break;
                }
            }
            if (isCut)
                break;
            for (int j = square, i = 1; i <= square && i <= alpha && j <= beta; ++i)
            {

                // L_AB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_AB[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_AB[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_AB[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_AB[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_AC[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_AC[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_AC[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_AC[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_BC[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_BC[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_BC[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_BC[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || L_BB[i][j][k].time[0] <= a[i + 1]) &&
                    (j == beta || L_BB[i][j][k].time[0] <= b[j + 1]) &&
                    (j == beta || L_BB[i][j][k].time[1] <= b[j + 1]) &&
                    (k == gamma || L_BB[i][j][k].time[1] <= c[k + 1]))
                    isCut = true;

                if (isCut)
                {
                    cut_i = i;
                    cut_j = j;
                    cut_k = k;
                    break;
                }
            }
            if (isCut)
                break;
        }
        if (isCut)
            break;

        if (square <= gamma)
        {
            int k = square;
            for (int i = 1; i <= square && i <= alpha; ++i)
            {
                for (int j = 1; j <= square && j <= beta; ++j)
                {

                    // L_AB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                    L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_AB[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_AB[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_AB[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_AB[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_AC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                    L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_AC[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_AC[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_AC[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_AC[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_BC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                    L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_BC[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_BC[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_BC[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_BC[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_BB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                    L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_BB[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_BB[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_BB[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_BB[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    if (isCut)
                    {
                        cut_i = i;
                        cut_j = j;
                        cut_k = k;
                        break;
                    }
                }
                if (isCut)
                    break;
            }
            if (isCut)
                break;
        }
    }

    // If the cut is found
    while (cut_i < alpha || cut_j < beta || cut_k < gamma)
    {
        cout << "----- Cut at " << cut_i << ", " << cut_j << ", " << cut_k << endl;
        cout << L_AB[cut_i][cut_j][cut_k].time[0] << " " << L_AB[cut_i][cut_j][cut_k].time[1] << endl;
        cout << L_AC[cut_i][cut_j][cut_k].time[0] << " " << L_AB[cut_i][cut_j][cut_k].time[1] << endl;
        cout << L_BB[cut_i][cut_j][cut_k].time[0] << " " << L_AB[cut_i][cut_j][cut_k].time[1] << endl;
        cout << L_BC[cut_i][cut_j][cut_k].time[0] << " " << L_AB[cut_i][cut_j][cut_k].time[1] << endl;
        // Fill the overlapping area
        tmpSolVec.push_back(L_AB[cut_i][cut_j][cut_k]);
        tmpSolVec.push_back(L_AC[cut_i][cut_j][cut_k]);
        tmpSolVec.push_back(L_BB[cut_i][cut_j][cut_k]);
        tmpSolVec.push_back(L_BC[cut_i][cut_j][cut_k]);
        string tmpRes = get_opt_table(tmpSolVec);
        cout << tmpSolVec.size() << " ";
        cout << "tmpRes = " << tmpRes << endl;
        tmpSolVec.clear();
        if (tmpRes == "AB")
        {
            for (int i = cut_i + 1; i <= alpha; ++i)
                L_AB[i][cut_j][cut_k] = update_sol(L_AB[i][cut_j][cut_k], max(a[i], L_AB[i - 1][cut_j][cut_k].time[0] + W_same), L_AB[i - 1][cut_j][cut_k].time[1], "AB", "X", &L_AB[i - 1][cut_j][cut_k]);
            for (int j = cut_j + 1; j <= beta; ++j)
                L_AB[cut_i][j][cut_k] = update_sol(L_AB[cut_i][j][cut_k], L_AB[cut_i][j - 1][cut_k].time[0], max(b[j], L_AB[cut_i][j - 1][cut_k].time[1] + W_same), "AB", "Y", &L_AB[cut_i][j - 1][cut_k]);
            for (int k = cut_k + 1; k <= gamma; ++k)
                L_AC[cut_i][cut_j][k] = update_sol(L_AC[cut_i][cut_j][k], L_AC[cut_i][cut_j][k - 1].time[0], max(c[k], L_AC[cut_i][cut_j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[cut_i][cut_j][k - 1]);
        }
        else if (tmpRes == "AC")
        {
            for (int i = cut_i + 1; i <= alpha; ++i)
                L_AC[i][cut_j][cut_k] = update_sol(L_AC[i][cut_j][cut_k], max(a[i], L_AC[i - 1][cut_j][cut_k].time[0] + W_same), L_AC[i - 1][cut_j][cut_k].time[1], "AC", "X", &L_AC[i - 1][cut_j][cut_k]);
            for (int k = cut_k + 1; k <= gamma; ++k)
                L_AC[cut_i][cut_j][k] = update_sol(L_AC[cut_i][cut_j][k], L_AC[cut_i][cut_j][k - 1].time[0], max(c[k], L_AC[cut_i][cut_j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[cut_i][cut_j][k - 1]);
        }
        else if (tmpRes == "BC")
        {
            for (int j = cut_j + 1; j <= beta; ++j)
                L_BC[cut_i][j][cut_k] = update_sol(L_BC[cut_i][j][cut_k], max(b[j], L_BC[cut_i][j - 1][cut_k].time[0] + W_same), L_BC[cut_i][j - 1][cut_k].time[1], "BC", "X", &L_BC[cut_i][j - 1][cut_k]);
            for (int k = cut_k + 1; k <= gamma; ++k)
                L_BC[cut_i][cut_j][k] = update_sol(L_BC[cut_i][cut_j][k], L_BC[cut_i][cut_j][k - 1].time[0], max(c[k], L_BC[cut_i][cut_j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[cut_i][cut_j][k - 1]);
            for (int i = cut_i + 1; i <= alpha; ++i)
                L_AC[i][cut_j][cut_k] = update_sol(L_AC[i][cut_j][cut_k], max(a[i], L_AC[i - 1][cut_j][cut_k].time[0] + W_same), L_AC[i - 1][cut_j][cut_k].time[1], "AC", "X", &L_AC[i - 1][cut_j][cut_k]);
        }
        else if (tmpRes == "BB")
        {
            for (int j = cut_j + 1; j <= beta; ++j)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[cut_i][j - 1][cut_k].time[0] + W_same), L_BB[cut_i][j - 1][cut_k].time[1], "BB", "X", &L_BB[cut_i][j - 1][cut_k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[cut_i][j - 1][cut_k].time[0], max(b[j], L_BB[cut_i][j - 1][cut_k].time[1] + W_same), "BB", "Y", &L_BB[cut_i][j - 1][cut_k]);
                L_BB[cut_i][j][cut_k] = choose_best_sol(L_BB[cut_i][j][cut_k], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int i = cut_i + 1; i <= alpha; ++i)
                L_AB[i][cut_j][cut_k] = update_sol(L_AB[i][cut_j][cut_k], max(a[i], L_AB[i - 1][cut_j][cut_k].time[0] + W_same), L_AB[i - 1][cut_j][cut_k].time[1], "AB", "X", &L_AB[i - 1][cut_j][cut_k]);
            for (int k = cut_k + 1; k <= gamma; ++k)
                L_BC[cut_i][cut_j][k] = update_sol(L_BC[cut_i][cut_j][k], L_BC[cut_i][cut_j][k - 1].time[0], max(c[k], L_BC[cut_i][cut_j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[cut_i][cut_j][k - 1]);
        }

        // Start from the cut, and continue finding the next cut
        isCut = false;
        int cut_min = min(min(cut_i, cut_j), min(cut_j, cut_k));
        for (int square = 1; square <= (N - cut_min); ++square)
        {
            // cout << "square = " << square << endl;
            for (int k = cut_k + 1; k <= (cut_k + square) && k <= gamma; ++k)
            {
                for (int i = cut_i + square, j = cut_j + 1; j <= (cut_j + square) && i <= alpha && j <= beta; ++j)
                {
                    // L_AB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                    L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_AB[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_AB[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_AB[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_AB[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_AC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                    L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_AC[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_AC[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_AC[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_AC[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_BC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                    L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_BC[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_BC[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_BC[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_BC[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_BB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                    L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_BB[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_BB[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_BB[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_BB[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // cout << "1: " << square << ", " << cut_i << ", " << cut_j << ", " << cut_k << ", " << i << ", " << j << ", " << k << endl;
                    if (isCut)
                    {
                        cut_i = i;
                        cut_j = j;
                        cut_k = k;
                        break;
                    }
                }
                if (isCut)
                    break;
                for (int j = cut_j + square, i = cut_i + 1; i <= (cut_i + square) && i <= alpha && j <= beta; ++i)
                {

                    // L_AB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                    L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_AB[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_AB[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_AB[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_AB[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_AC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                    L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_AC[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_AC[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_AC[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_AC[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_BC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                    L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_BC[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_BC[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_BC[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_BC[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // L_BB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                    L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || L_BB[i][j][k].time[0] <= a[i + 1]) &&
                        (j == beta || L_BB[i][j][k].time[0] <= b[j + 1]) &&
                        (j == beta || L_BB[i][j][k].time[1] <= b[j + 1]) &&
                        (k == gamma || L_BB[i][j][k].time[1] <= c[k + 1]))
                        isCut = true;

                    // cout << "2: " << square << ", " << cut_i << ", " << cut_j << ", " << cut_k << ", " << i << ", " << j << ", " << k << endl;
                    if (isCut)
                    {
                        cut_i = i;
                        cut_j = j;
                        cut_k = k;
                        break;
                    }
                }
                if (isCut)
                    break;
            }
            if (isCut)
                break;

            if (square <= gamma)
            {
                int k = square;
                for (int i = cut_i + 1; i <= (cut_i + square) && i <= alpha; ++i)
                {
                    for (int j = cut_j + 1; j <= (cut_j + square) && j <= beta; ++j)
                    {

                        // L_AB
                        tmpSolVec.resize(4);
                        tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                        tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                        tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                        tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                        L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                        tmpSolVec.clear();
                        if ((i == alpha || L_AB[i][j][k].time[0] <= a[i + 1]) &&
                            (j == beta || L_AB[i][j][k].time[0] <= b[j + 1]) &&
                            (j == beta || L_AB[i][j][k].time[1] <= b[j + 1]) &&
                            (k == gamma || L_AB[i][j][k].time[1] <= c[k + 1]))
                            isCut = true;

                        // L_AC
                        tmpSolVec.resize(4);
                        tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                        tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                        tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                        tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                        L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                        tmpSolVec.clear();
                        if ((i == alpha || L_AC[i][j][k].time[0] <= a[i + 1]) &&
                            (j == beta || L_AC[i][j][k].time[0] <= b[j + 1]) &&
                            (j == beta || L_AC[i][j][k].time[1] <= b[j + 1]) &&
                            (k == gamma || L_AC[i][j][k].time[1] <= c[k + 1]))
                            isCut = true;

                        // L_BC
                        tmpSolVec.resize(4);
                        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                        tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                        tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                        tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                        L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                        tmpSolVec.clear();
                        if ((i == alpha || L_BC[i][j][k].time[0] <= a[i + 1]) &&
                            (j == beta || L_BC[i][j][k].time[0] <= b[j + 1]) &&
                            (j == beta || L_BC[i][j][k].time[1] <= b[j + 1]) &&
                            (k == gamma || L_BC[i][j][k].time[1] <= c[k + 1]))
                            isCut = true;

                        // L_BB
                        tmpSolVec.resize(4);
                        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                        tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                        tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                        tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                        L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                        tmpSolVec.clear();
                        if ((i == alpha || L_BB[i][j][k].time[0] <= a[i + 1]) &&
                            (j == beta || L_BB[i][j][k].time[0] <= b[j + 1]) &&
                            (j == beta || L_BB[i][j][k].time[1] <= b[j + 1]) &&
                            (k == gamma || L_BB[i][j][k].time[1] <= c[k + 1]))
                            isCut = true;

                        // cout << "3: " << i << ", " << j << ", " << k << endl;
                        if (isCut)
                        {
                            cut_i = i;
                            cut_j = j;
                            cut_k = k;
                            break;
                        }
                    }
                    if (isCut)
                        break;
                }
                if (isCut)
                    break;
            }
        }
    }
    cout << "Finish" << endl;
    cout << L_AB[alpha][beta][gamma].time[0] << " " << L_AB[alpha][beta][gamma].time[1] << endl;
    cout << L_AC[alpha][beta][gamma].time[0] << " " << L_AB[alpha][beta][gamma].time[1] << endl;
    cout << L_BB[alpha][beta][gamma].time[0] << " " << L_AB[alpha][beta][gamma].time[1] << endl;
    cout << L_BC[alpha][beta][gamma].time[0] << " " << L_AB[alpha][beta][gamma].time[1] << endl;
    // Print table
    cout << "L_AB-------------------------------" << endl;
    for (int k = 0; k <= gamma; ++k)
    {
        cout << "k = " << k << endl;
        for (int i = 0; i <= alpha; ++i)
        {
            for (int j = 0; j <= beta; ++j)
                cout << L_AB[i][j][k].time[0] << " " << L_AB[i][j][k].time[1] << " " << L_AB[i][j][k].table << " " << L_AB[i][j][k].lane << " || ";
            cout << endl;
        }
        cout << endl;
    }
    cout << "L_AC-------------------------------" << endl;
    for (int k = 0; k <= gamma; ++k)
    {
        cout << "k = " << k << endl;
        for (int i = 0; i <= alpha; ++i)
        {
            for (int j = 0; j <= beta; ++j)
                cout << L_AC[i][j][k].time[0] << " " << L_AC[i][j][k].time[1] << " " << L_AC[i][j][k].table << " " << L_AC[i][j][k].lane << " || ";
            cout << endl;
        }
        cout << endl;
    }
    cout << "L_BB-------------------------------" << endl;
    for (int k = 0; k <= gamma; ++k)
    {
        cout << "k = " << k << endl;
        for (int i = 0; i <= alpha; ++i)
        {
            for (int j = 0; j <= beta; ++j)
                cout << L_BB[i][j][k].time[0] << " " << L_BB[i][j][k].time[1] << " " << L_BB[i][j][k].table << " " << L_BB[i][j][k].lane << " || ";
            cout << endl;
        }
        cout << endl;
    }
    cout << "L_BC-------------------------------" << endl;
    for (int k = 0; k <= gamma; ++k)
    {
        cout << "k = " << k << endl;
        for (int i = 0; i <= alpha; ++i)
        {
            for (int j = 0; j <= beta; ++j)
                cout << L_BC[i][j][k].time[0] << " " << L_BC[i][j][k].time[1] << " " << L_BC[i][j][k].table << " " << L_BC[i][j][k].lane << " || ";
            cout << endl;
        }
        cout << endl;
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

tuple<float, double> schedule_by_reduced_dp(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff, int carNum)
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
        int cut_i = a_all.size(), cut_j = b_all.size(), cut_k = c_all.size();
        vector<float> a = get_window_by_num(a_all, cut_i);
        vector<float> b = get_window_by_num(b_all, cut_j);
        vector<float> c = get_window_by_num(c_all, cut_k);
        if (a.size() > 1 && b.size() > 1 && c.size() > 1)
            tie(last_X, last_Y, tmp) = reduced_dp(a, b, c, W_same, W_diff, last_X, last_Y);
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
    cout << "reduced result: " << T_last << " " << totalComputeTime << endl;
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
    // a_all = {0, 409, 410};
    // b_all = {0, 462, 463};
    // c_all = {0, 433, 526};
    // a_all = {0, 2, 4, 5, 7, 8};
    // b_all = {0, 3, 4, 13, 15, 19};
    // c_all = {0, 4, 6, 7, 11, 12};

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
    // schedule_by_num_window_reduced(a_all, b_all, c_all, W_same, W_diff, 5);
    // schedule_by_num_window_reduced(a_all, b_all, c_all, W_same, W_diff, 10);
    // schedule_by_num_window_reduced(a_all, b_all, c_all, W_same, W_diff, 20);
    schedule_by_num_window_reduced(a_all, b_all, c_all, W_same, W_diff, 100);
    greedy_dp(a_all, b_all, c_all, W_same, W_diff);

    cout << "a_all = {" << a_all[0];
    for (int i = 1; i < a_all.size(); ++i)
        cout << ", " << a_all[i];
    cout << "};" << endl;
    cout << "b_all = {" << b_all[0];
    for (int i = 1; i < b_all.size(); ++i)
        cout << ", " << b_all[i];
    cout << "};" << endl;
    cout << "c_all = {" << c_all[0];
    for (int i = 1; i < c_all.size(); ++i)
        cout << ", " << c_all[i];
    cout << "};" << endl;

    return 0;
}