#include "dp.h"

tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_dp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, tuple<char, int, double> last_X, tuple<char, int, double> last_Y, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    int alpha = A.size() - 1;
    int beta = B.size() - 1;
    int gamma = C.size() - 1;
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
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1].time, T_X + W_same), max(B[1].time, T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1].time, T_X + W_same), max(C[1].time, T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1].time, T_X + W_diff), max(C[1].time, T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1].time, T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1].time, T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1].time, T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1].time, T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1].time, T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1].time, T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1].time, T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "AC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1].time, T_X + W_same), max(B[1].time, T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1].time, T_X + W_same), max(C[1].time, T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1].time, T_X + W_diff), max(C[1].time, T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1].time, T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1].time, T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1].time, T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1].time, T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1].time, T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1].time, T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1].time, T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BB")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1].time, T_X + W_diff), max(B[1].time, T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1].time, T_X + W_diff), max(C[1].time, T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1].time, T_X + W_same), max(C[1].time, T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1].time, T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1].time, T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1].time, T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1].time, T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1].time, T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1].time, T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1].time, T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BC")
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1].time, T_X + W_diff), max(B[1].time, T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1].time, T_X + W_diff), max(C[1].time, T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1].time, T_X + W_same), max(C[1].time, T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1].time, T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1].time, T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1].time, T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1].time, T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1].time, T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1].time, T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1].time, T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], A[1].time, B[1].time, "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], A[1].time, C[1].time, "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], B[1].time, C[1].time, "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], A[1].time, -W_diff, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, B[1].time, "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], A[1].time, -W_diff, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, C[1].time, "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], B[1].time, -W_diff, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, C[1].time, "BC", "Y");
        L_BB[0][1][0] = (A[1].time <= C[1].time) ? update_sol(L_BB[0][1][0], -W_diff, B[1].time, "BB", "Y") : update_sol(L_BB[0][1][0], B[1].time, -W_diff, "BB", "X");
    }
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j].time, L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], max(B[j].time, L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], max(A[i].time, L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], max(B[j].time, L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], max(A[i].time, L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], max(C[k].time, L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], max(B[j].time, L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], max(C[k].time, L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], max(A[i].time, L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "X");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], max(A[i].time, L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], max(C[k].time, L_AC[1][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], max(C[k].time, L_BC[0][1][k - 1].time[1] + W_same), "BC", "Y");
    if (beta > 1)
    {
        if (last_XY == "AB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, L_AC[i][0][0].time[0] + W_diff), max(B[2].time, T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, L_AC[i][0][0].time[0] + W_diff), max(B[1].time, T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_diff), max(B[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, T_X + W_diff), max(B[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "AC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, L_AC[i][0][0].time[0] + W_diff), max(B[2].time, T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, L_AC[i][0][0].time[0] + W_diff), max(B[1].time, T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_diff), max(B[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, T_X + W_diff), max(B[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, L_AC[i][0][0].time[0] + W_diff), max(B[2].time, T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, L_AC[i][0][0].time[0] + W_diff), max(B[1].time, T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_same), max(B[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, T_X + W_same), max(B[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, L_AC[i][0][0].time[0] + W_diff), max(B[2].time, T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, L_AC[i][0][0].time[0] + W_diff), max(B[1].time, T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, T_X + W_same), max(B[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, T_X + W_same), max(B[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1].time, L_AC[i][0][0].time[0] + W_diff), B[2].time, "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2].time, L_AC[i][0][0].time[0] + W_diff), B[1].time, "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], B[1].time, max(B[2].time, L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], B[2].time, max(B[1].time, L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
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
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i].time, L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i].time, L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], max(B[j].time, L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], max(A[i].time, L_AC[i - 1][0][k - 1].time[0] + W_same), max(C[k].time, L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j].time, L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], max(C[k].time, L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], max(C[k].time, L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j].time, L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j].time, L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], max(B[j].time, L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
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
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i].time, L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i].time, L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(B[j].time, L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(B[j].time, L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i].time, L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i].time, L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(C[k].time, L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(C[k].time, L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j].time, L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j].time, L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(C[k].time, L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(C[k].time, L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j].time, max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1] + W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j].time, max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1] + W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(B[j].time, max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0] + W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(B[j].time, max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0] + W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
            }
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
    int i = alpha;
    int j = beta;
    int k = gamma;
    // string optTable;
    string table = "";
    string lanes = "";
    // Choose the optimal solution and the table to start backtracking
    tmpSolVec.push_back(L_AB[i][j][k]);
    tmpSolVec.push_back(L_AC[i][j][k]);
    tmpSolVec.push_back(L_BB[i][j][k]);
    tmpSolVec.push_back(L_BC[i][j][k]);
    table = get_opt_table(tmpSolVec);
    tmpSolVec.clear();

    // if (optTable == "AB")
    // {
    //     table = L_AB[i][j][k].table;
    //     lanes = L_AB[i][j][k].lane;
    //     if (lanes == "X")
    //     {
    //         stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
    //         --i;
    //     }
    //     else if (lanes == "Y")
    //     {
    //         stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
    //         --j;
    //     }
    //     else if (lanes == "XY")
    //     {
    //         stack_X.push(make_tuple('A', i, L_AB[i][j][k].time[0]));
    //         stack_Y.push(make_tuple('B', j, L_AB[i][j][k].time[1]));
    //         --i;
    //         --j;
    //     }
    // }
    // else if (optTable == "AC")
    // {
    //     table = L_AC[i][j][k].table;
    //     lanes = L_AC[i][j][k].lane;
    //     if (lanes == "X")
    //     {
    //         stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
    //         --i;
    //     }
    //     else if (lanes == "Y")
    //     {
    //         stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
    //         --k;
    //     }
    //     else if (lanes == "XY")
    //     {
    //         stack_X.push(make_tuple('A', i, L_AC[i][j][k].time[0]));
    //         stack_Y.push(make_tuple('C', k, L_AC[i][j][k].time[1]));
    //         --i;
    //         --k;
    //     }
    // }
    // else if (optTable == "BB")
    // {
    //     table = L_BB[i][j][k].table;
    //     lanes = L_BB[i][j][k].lane;
    //     if (lanes == "X")
    //     {
    //         stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
    //         --j;
    //     }
    //     else if (lanes == "Y")
    //     {
    //         stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
    //         --j;
    //     }
    //     else if (lanes == "XY")
    //     {
    //         stack_X.push(make_tuple('B', j - 1, L_BB[i][j][k].time[0]));
    //         stack_Y.push(make_tuple('B', j, L_BB[i][j][k].time[1]));
    //         j -= 2;
    //     }
    //     else if (lanes == "YX")
    //     {
    //         stack_Y.push(make_tuple('B', j - 1, L_BB[i][j][k].time[1]));
    //         stack_X.push(make_tuple('B', j, L_BB[i][j][k].time[0]));
    //         j -= 2;
    //     }
    // }
    // else if (optTable == "BC")
    // {
    //     table = L_BC[i][j][k].table;
    //     lanes = L_BC[i][j][k].lane;
    //     if (lanes == "X")
    //     {
    //         stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
    //         --j;
    //     }
    //     else if (lanes == "Y")
    //     {
    //         stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
    //         --k;
    //     }
    //     else if (lanes == "XY")
    //     {
    //         stack_X.push(make_tuple('B', j, L_BC[i][j][k].time[0]));
    //         stack_Y.push(make_tuple('C', k, L_BC[i][j][k].time[1]));
    //         --j;
    //         --k;
    //     }
    // }

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
    schedule_A.clear();
    schedule_B.clear();
    schedule_C.clear();
    // cout << "Lane X: " << endl;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        if (get<0>(stack_X.top()) == 'A')
        {
            schedule_A.push_back(vehicle(A[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
            wait_time += (get<2>(stack_X.top()) - A[get<1>(stack_X.top())].time);
        }
        else
        {
            Vehicle::setRouteID(B[get<1>(stack_X.top())].id, "route_1");
            schedule_B.push_back(vehicle(B[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
            wait_time += (get<2>(stack_X.top()) - B[get<1>(stack_X.top())].time);
        }
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    if (get<0>(stack_X.top()) == 'A')
    {
        schedule_A.push_back(vehicle(A[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
        wait_time += (get<2>(stack_X.top()) - A[get<1>(stack_X.top())].time);
    }
    else
    {
        Vehicle::setRouteID(B[get<1>(stack_X.top())].id, "route_1");
        schedule_B.push_back(vehicle(B[get<1>(stack_X.top())].id, get<2>(stack_X.top())));
        wait_time += (get<2>(stack_X.top()) - B[get<1>(stack_X.top())].time);
    }
    last_X = stack_X.top();

    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        if (get<0>(stack_Y.top()) == 'C')
        {
            schedule_C.push_back(vehicle(C[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
            wait_time += (get<2>(stack_Y.top()) - C[get<1>(stack_Y.top())].time);
        }
        else
        {
            Vehicle::setRouteID(B[get<1>(stack_Y.top())].id, "route_2");
            schedule_B.push_back(vehicle(B[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
            wait_time += (get<2>(stack_Y.top()) - B[get<1>(stack_Y.top())].time);
        }
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    if (get<0>(stack_Y.top()) == 'C')
    {
        schedule_C.push_back(vehicle(C[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
        wait_time += (get<2>(stack_Y.top()) - C[get<1>(stack_Y.top())].time);
    }
    else
    {
        Vehicle::setRouteID(B[get<1>(stack_Y.top())].id, "route_2");
        schedule_B.push_back(vehicle(B[get<1>(stack_Y.top())].id, get<2>(stack_Y.top())));
        wait_time += (get<2>(stack_Y.top()) - B[get<1>(stack_Y.top())].time);
    }
    last_Y = stack_Y.top();

    return make_tuple(last_X, last_Y, wait_time);
}

vector<vehicle> get_window_by_num(vector<vehicle> &traffic, int carNum)
{
    if (carNum >= traffic.size())
        carNum = traffic.size() - 1;
    vector<vehicle> subtraffic(traffic.begin(), traffic.begin() + carNum + 1);
    traffic.erase(traffic.begin() + 1, traffic.begin() + carNum + 1);
    return subtraffic;
}

void schedule_by_window_dp(vector<vehicle> A_all, vector<vehicle> B_all, vector<vehicle> C_all, int carNum, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    tuple<char, int, double> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, double> last_Y = make_tuple('0', 0, 0.0);
    double wait_time = 0;
    int vehicle_num = A_all.size() + B_all.size() + C_all.size() - 3;
    vector<vehicle> subSchedule_A, subSchedule_B, subSchedule_C;

    schedule_A.clear();
    schedule_B.clear();
    schedule_C.clear();
    while (A_all.size() > 1 || B_all.size() > 1 || C_all.size() > 1)
    {
        vector<vehicle> A = get_window_by_num(A_all, carNum);
        vector<vehicle> B = get_window_by_num(B_all, carNum);
        vector<vehicle> C = get_window_by_num(C_all, carNum);
        if (A.size() > 1 && B.size() > 1 && C.size() > 1)
        {
            tie(last_X, last_Y, wait_time) = window_dp_compute_entering_time(A, B, C, last_X, last_Y, subSchedule_A, subSchedule_B, subSchedule_C);
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
            schedule_B.insert(schedule_B.end(), subSchedule_B.begin(), subSchedule_B.end());
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (A.size() > 1 && B.size() > 1)
        {
            A.erase(A.begin());
            B.erase(B.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X, subSchedule_A);
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
            tie(last_Y, wait_time) = schedule_single_lane('B', B, last_Y, subSchedule_B);
            for (auto veh : B)
                Vehicle::setRouteID(veh.id, "route_2");
            schedule_B.insert(schedule_B.end(), subSchedule_B.begin(), subSchedule_B.end());
        }
        else if (A.size() > 1 and C.size() > 1)
        {
            A.erase(A.begin());
            C.erase(C.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X, subSchedule_A);
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y, subSchedule_C);
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (B.size() > 1 and C.size() > 1)
        {
            B.erase(B.begin());
            C.erase(C.begin());
            tie(last_X, wait_time) = schedule_single_lane('B', B, last_X, subSchedule_B);
            for (auto veh : B)
                Vehicle::setRouteID(veh.id, "route_1");
            schedule_B.insert(schedule_B.end(), subSchedule_B.begin(), subSchedule_B.end());
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y, subSchedule_C);
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (A.size() > 1)
        {
            A.erase(A.begin());
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X, subSchedule_A);
            schedule_A.insert(schedule_A.end(), subSchedule_A.begin(), subSchedule_A.end());
        }
        else if (C.size() > 1)
        {
            C.erase(C.begin());
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y, subSchedule_C);
            schedule_C.insert(schedule_C.end(), subSchedule_C.begin(), subSchedule_C.end());
        }
        else if (B.size() > 1)
        {
            if (get<2>(last_X) < get<2>(last_Y))
            {
                if (get<0>(last_X) == '0')
                {
                    Vehicle::setRouteID(B[1].id, "route_1");
                    schedule_B.push_back(vehicle(B[1].id, B[1].time));
                    last_X = make_tuple('B', 1, B[1].time);
                }
                else if (get<0>(last_X) == 'A')
                {
                    Vehicle::setRouteID(B[1].id, "route_1");
                    schedule_B.push_back(vehicle(B[1].id, max(B[1].time, get<2>(last_X) + W_diff)));
                    last_X = make_tuple('B', 1, max(B[1].time, get<2>(last_X) + W_diff));
                }
                else
                {
                    Vehicle::setRouteID(B[1].id, "route_1");
                    schedule_B.push_back(vehicle(B[1].id, max(B[1].time, get<2>(last_X) + W_same)));
                    last_X = make_tuple('B', 1, max(B[1].time, get<2>(last_X) + W_same));
                }
                if (B.size() > 2)
                {
                    if (get<0>(last_Y) == '0')
                    {
                        Vehicle::setRouteID(B[2].id, "route_2");
                        schedule_B.push_back(vehicle(B[2].id, B[2].time));
                        last_Y = make_tuple('B', 2, B[2].time);
                    }
                    else if (get<0>(last_Y) == 'C')
                    {
                        Vehicle::setRouteID(B[2].id, "route_2");
                        schedule_B.push_back(vehicle(B[2].id, max(B[2].time, get<2>(last_Y) + W_diff)));
                        last_Y = make_tuple('B', 2, max(B[2].time, get<2>(last_Y) + W_diff));
                    }
                    else
                    {
                        Vehicle::setRouteID(B[2].id, "route_2");
                        schedule_B.push_back(vehicle(B[2].id, max(B[2].time, get<2>(last_Y) + W_same)));
                        last_Y = make_tuple('B', 2, max(B[2].time, get<2>(last_Y) + W_same));
                    }
                    for (int i = 3; i < B.size(); ++i)
                    {
                        if (i % 2 == 1)
                        {
                            Vehicle::setRouteID(B[i].id, "route_1");
                            schedule_B.push_back(vehicle(B[i].id, max(B[i].time, get<2>(last_X) + W_same)));
                            last_X = make_tuple('B', i, max(B[i].time, get<2>(last_X) + W_same));
                        }
                        else
                        {
                            Vehicle::setRouteID(B[i].id, "route_2");
                            schedule_B.push_back(vehicle(B[i].id, max(B[i].time, get<2>(last_Y) + W_same)));
                            last_Y = make_tuple('B', i, max(B[i].time, get<2>(last_Y) + W_same));
                        }
                    }
                }
            }
            else
            {
                if (get<0>(last_Y) == '0')
                {
                    Vehicle::setRouteID(B[1].id, "route_2");
                    schedule_B.push_back(vehicle(B[1].id, B[1].time));
                    last_Y = make_tuple('B', 1, B[1].time);
                }
                else if (get<0>(last_Y) == 'C')
                {
                    Vehicle::setRouteID(B[1].id, "route_2");
                    schedule_B.push_back(vehicle(B[1].id, max(B[1].time, get<2>(last_Y) + W_diff)));
                    last_Y = make_tuple('B', 1, max(B[1].time, get<2>(last_Y) + W_diff));
                }
                else
                {
                    Vehicle::setRouteID(B[1].id, "route_2");
                    schedule_B.push_back(vehicle(B[1].id, max(B[1].time, get<2>(last_Y) + W_same)));
                    last_Y = make_tuple('B', 1, max(B[1].time, get<2>(last_Y) + W_same));
                }
                if (B.size() > 2)
                {
                    if (get<0>(last_X) == '0')
                    {
                        Vehicle::setRouteID(B[2].id, "route_1");
                        schedule_B.push_back(vehicle(B[2].id, B[2].time));
                        last_X = make_tuple('B', 2, B[2].time);
                    }
                    else if (get<0>(last_X) == 'A')
                    {
                        Vehicle::setRouteID(B[2].id, "route_1");
                        schedule_B.push_back(vehicle(B[2].id, max(B[2].time, get<2>(last_X) + W_diff)));
                        last_X = make_tuple('B', 2, max(B[2].time, get<2>(last_X) + W_diff));
                    }
                    else
                    {
                        Vehicle::setRouteID(B[2].id, "route_1");
                        schedule_B.push_back(vehicle(B[2].id, max(B[2].time, get<2>(last_X) + W_same)));
                        last_X = make_tuple('B', 2, max(B[2].time, get<2>(last_X) + W_same));
                    }
                    for (int i = 3; i < B.size(); ++i)
                    {
                        if (i % 2 == 1)
                        {
                            Vehicle::setRouteID(B[i].id, "route_2");
                            schedule_B.push_back(vehicle(B[i].id, max(B[i].time, get<2>(last_Y) + W_same)));
                            last_Y = make_tuple('B', i, max(B[i].time, get<2>(last_Y) + W_same));
                        }
                        else
                        {
                            Vehicle::setRouteID(B[i].id, "route_1");
                            schedule_B.push_back(vehicle(B[i].id, max(B[i].time, get<2>(last_X) + W_same)));
                            last_X = make_tuple('B', i, max(B[i].time, get<2>(last_X) + W_same));
                        }
                    }
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    // cout << "dp_" << carNum << " result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
}
