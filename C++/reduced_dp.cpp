#include "reduced_dp.h"

// Do not traverse all space
tuple<tuple<char, int, double>, tuple<char, int, double>, int, int, int, double> reduced_dp(vector<double> a, vector<double> b, vector<double> c, tuple<char, int, double> last_X, tuple<char, int, double> last_Y)
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
    double T_X, T_Y;
    vector<Solution> tmpSolVec;
    double wait_time = 0;

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

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "AC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_same), max(b[1], T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_same), max(c[1], T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_diff), max(c[1], T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_diff), max(b[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_diff), max(c[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_same), max(c[1], T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(a[1], T_X + W_diff), max(b[1], T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(a[1], T_X + W_diff), max(c[1], T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(b[1], T_X + W_same), max(c[1], T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(a[1], T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(b[1], T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(a[1], T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(c[1], T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(b[1], T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(c[1], T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[1], T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(b[1], T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], a[1], b[1], "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], a[1], c[1], "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], b[1], c[1], "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], a[1], -W_diff, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, b[1], "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], a[1], -W_diff, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, c[1], "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], b[1], -W_diff, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, c[1], "BC", "Y");
        L_BB[0][1][0] = (a[1] <= c[1]) ? update_sol(L_BB[0][1][0], -W_diff, b[1], "BB", "Y") : update_sol(L_BB[0][1][0], b[1], -W_diff, "BB", "X");
    }
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], max(b[j], L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], max(a[i], L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], max(b[j], L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], max(a[i], L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], max(c[k], L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], max(b[j], L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], max(c[k], L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    if (beta > 1)
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
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], max(b[j], L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
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
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], max(c[k], L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], max(c[k], L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], max(b[j], L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    // Compute tables
    bool isCut = false;
    int N = max(max(alpha, beta), max(beta, gamma));
    int cut_i = alpha, cut_j = beta, cut_k = gamma;

    // Find where to cut
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
                if ((i == alpha || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at AB: " << L_AB[i][j][k].time[0] << ", " << L_AB[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at AC: " << L_AC[i][j][k].time[0] << ", " << L_AC[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }
                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at BC: " << L_BC[i][j][k].time[0] << ", " << L_BC[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }
                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1] + W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1] + W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0] + W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0] + W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at BB: " << L_BB[i][j][k].time[0] << ", " << L_BB[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }

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
                if ((i == alpha || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at AB: " << L_AB[i][j][k].time[0] << ", " << L_AB[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at AC: " << L_AC[i][j][k].time[0] << ", " << L_AC[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at BC: " << L_BC[i][j][k].time[0] << ", " << L_BC[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1]+W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1]+W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0]+W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0]+W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                if ((i == alpha || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= a[i + 1]) &&
                    (j == beta || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= b[j + 1]) &&
                    (k == gamma || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= c[k + 1]))
                {
                    // cout << "Break at BB: " << L_BB[i][j][k].time[0] << ", " << L_BB[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                    isCut = true;
                }

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
                    if ((i == alpha || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= a[i + 1]) &&
                        (j == beta || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= b[j + 1]) &&
                        (k == gamma || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= c[k + 1]))
                    {
                        // cout << "Break at AB: " << L_AB[i][j][k].time[0] << ", " << L_AB[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                        isCut = true;
                    }

                    // L_AC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                    L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= a[i + 1]) &&
                        (j == beta || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= b[j + 1]) &&
                        (k == gamma || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= c[k + 1]))
                    {
                        // cout << "Break at AC: " << L_AC[i][j][k].time[0] << ", " << L_AC[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                        isCut = true;
                    }

                    // L_BC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                    L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= a[i + 1]) &&
                        (j == beta || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= b[j + 1]) &&
                        (k == gamma || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= c[k + 1]))
                    {
                        // cout << "Break at BC: " << L_BC[i][j][k].time[0] << ", " << L_BC[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                        isCut = true;
                    }

                    // L_BB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(b[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(b[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(b[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(b[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                    L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    if ((i == alpha || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= a[i + 1]) &&
                        (j == beta || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= b[j + 1]) &&
                        (k == gamma || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= c[k + 1]))
                    {
                        // cout << "Break at BB: " << L_BB[i][j][k].time[0] << ", " << L_BB[i][j][k].time[1] << " " << a[i + 1] << " " << b[j + 1] << " " << c[k + 1] << " " << (i == alpha) << " " << (j == beta) << " " << (k == gamma) << endl;
                        isCut = true;
                    }

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

    // Print table
    // cout << "L_AB-------------------------------" << endl;
    // print_3d_table(L_AB);
    // cout << "L_AC-------------------------------" << endl;
    // print_3d_table(L_AC);
    // cout << "L_BB-------------------------------" << endl;
    // print_3d_table(L_BB);
    // cout << "L_BC-------------------------------" << endl;
    // print_3d_table(L_BC);

    // Push order to stack
    stack<tuple<char, int, double>> stack_X, stack_Y;
    int i = cut_i;
    int j = cut_j;
    int k = cut_k;
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
        if (get<0>(stack_X.top()) == 'A')
            wait_time += (get<2>(stack_X.top()) - a[get<1>(stack_X.top())]);
        else
            wait_time += (get<2>(stack_X.top()) - b[get<1>(stack_X.top())]);
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    if (get<0>(stack_X.top()) == 'A')
        wait_time += (get<2>(stack_X.top()) - a[get<1>(stack_X.top())]);
    else
        wait_time += (get<2>(stack_X.top()) - b[get<1>(stack_X.top())]);
    last_X = stack_X.top();

    // cout << "Lane Y: " << endl;
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        if (get<0>(stack_Y.top()) == 'C')
            wait_time += (get<2>(stack_Y.top()) - c[get<1>(stack_Y.top())]);
        else
            wait_time += (get<2>(stack_Y.top()) - b[get<1>(stack_Y.top())]);
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    if (get<0>(stack_Y.top()) == 'C')
        wait_time += (get<2>(stack_Y.top()) - c[get<1>(stack_Y.top())]);
    else
        wait_time += (get<2>(stack_Y.top()) - b[get<1>(stack_Y.top())]);
    last_Y = stack_Y.top();

    // Cauculate computation time
    auto t_end = chrono::high_resolution_clock::now();
    double computeTime = chrono::duration_cast<chrono::nanoseconds>(t_end - t_start).count();
    computeTime *= 1e-9;

    return make_tuple(last_X, last_Y, cut_i, cut_j, cut_k, wait_time);
}

tuple<double, double, double> schedule_by_reduced_dp(vector<double> a, vector<double> b, vector<double> c)
{
    auto t0 = chrono::high_resolution_clock::now();
    tuple<char, int, double> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, double> last_Y = make_tuple('0', 0, 0.0);
    double tmp;
    int cut_i = 0, cut_j = 0, cut_k = 0;
    double total_wait = 0;
    double wait_time = 0;
    int vehicle_num = a.size() + b.size() + c.size() - 3;

    tie(last_X, last_Y, cut_i, cut_j, cut_k, wait_time) = reduced_dp(a, b, c, last_X, last_Y);
    total_wait += wait_time;
    // cout << "Cut at " << cut_i << ", " << cut_j << ", " << cut_k << endl;
    while (a.size() > 1 || b.size() > 1 || c.size() > 1)
    {
        a.erase(a.begin() + 1, a.begin() + cut_i + 1);
        b.erase(b.begin() + 1, b.begin() + cut_j + 1);
        c.erase(c.begin() + 1, c.begin() + cut_k + 1);
        if (a.size() > 1 && b.size() > 1 && c.size() > 1)
        {
            tie(last_X, last_Y, cut_i, cut_j, cut_k, wait_time) = reduced_dp(a, b, c, last_X, last_Y);
            total_wait += wait_time;
            // cout << "Cut at " << cut_i << ", " << cut_j << ", " << cut_k << endl;
        }
        else if (a.size() > 1 && b.size() > 1)
        {
            a.erase(a.begin());
            b.erase(b.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X);
            total_wait += wait_time;
            tie(last_Y, wait_time) = schedule_single_lane('B', b, last_Y);
            total_wait += wait_time;
            a.clear();
            b.clear();
        }
        else if (a.size() > 1 and c.size() > 1)
        {
            a.erase(a.begin());
            c.erase(c.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X);
            total_wait += wait_time;
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y);
            total_wait += wait_time;
            a.clear();
            c.clear();
        }
        else if (b.size() > 1 and c.size() > 1)
        {
            b.erase(b.begin());
            c.erase(c.begin());
            tie(last_X, wait_time) = schedule_single_lane('B', b, last_X);
            total_wait += wait_time;
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y);
            total_wait += wait_time;
            b.clear();
            c.clear();
        }
        else if (a.size() > 1)
        {
            a.erase(a.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X);
            total_wait += wait_time;
            a.clear();
        }
        else if (c.size() > 1)
        {
            c.erase(c.begin());
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y);
            total_wait += wait_time;
            c.clear();
        }
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
                total_wait += (get<2>(last_X) - b[1]);
                if (b.size() > 2)
                {
                    if (get<0>(last_Y) == '0')
                        last_Y = make_tuple('B', 2, b[2]);
                    else if (get<0>(last_Y) == 'C')
                        last_Y = make_tuple('B', 2, max(b[2], get<2>(last_Y) + W_diff));
                    else
                        last_Y = make_tuple('B', 2, max(b[2], get<2>(last_Y) + W_same));
                    total_wait += (get<2>(last_Y) - b[2]);
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                        {
                            last_X = make_tuple('B', i, max(b[i], get<2>(last_X) + W_same));
                            total_wait += (get<2>(last_X) - b[i]);
                        }
                        else
                        {
                            last_Y = make_tuple('B', i, max(b[i], get<2>(last_Y) + W_same));
                            total_wait += (get<2>(last_Y) - b[i]);
                        }
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
                total_wait += (get<2>(last_Y) - b[1]);
                if (b.size() > 2)
                {
                    if (get<0>(last_X) == '0')
                        last_X = make_tuple('B', 2, b[2]);
                    else if (get<0>(last_X) == 'A')
                        last_X = make_tuple('B', 2, max(b[2], get<2>(last_X) + W_diff));
                    else
                        last_X = make_tuple('B', 2, max(b[2], get<2>(last_X) + W_same));
                    total_wait += (get<2>(last_X) - b[2]);
                    for (int i = 3; i < b.size(); ++i)
                    {
                        if (i % 2 == 1)
                        {
                            last_Y = make_tuple('B', i, max(b[i], get<2>(last_Y) + W_same));
                            total_wait += (get<2>(last_Y) - b[i]);
                        }
                        else
                        {
                            last_X = make_tuple('B', i, max(b[i], get<2>(last_X) + W_same));
                            total_wait += (get<2>(last_X) - b[i]);
                        }
                    }
                }
            }
            b.clear();
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    double T_last = max(get<2>(last_X), get<2>(last_Y));
    double T_delay = total_wait / vehicle_num;
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    // cout << "dp_reduced result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
    return {T_last, T_delay, totalComputeTime};
}
