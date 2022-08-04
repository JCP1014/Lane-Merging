#include "pruned_dp.h"

// Pruned 3D DP-based algorithm
tuple<tuple<char, int, double>, tuple<char, int, double>, int, int, int, double> pruned_dp(vector<double> A, vector<double> B, vector<double> C, tuple<char, int, double> last_X, tuple<char, int, double> last_Y)
{
    int alpha = A.size() - 1;   // number of vehicles on Lane A
    int beta = B.size() - 1;    // number of vehicles on Lane B
    int gamma = C.size() - 1;   // number of vehicles on Lane C
    vector<vector<vector<Solution>>> L_AB, L_AC, L_BB, L_BC;    // DP tables
    string last_XY; // lanes that the last vehicles going to Lane X and Lane Y come from
    double T_X;    // scheduled entering time of the last vehicle going to Lane X
    double T_Y;    // scheduled entering time of the last vehicle going to Lane Y
    vector<Solution> tmpSolVec; // temporary vector for storing possible solutions
    double wait_time = 0;   // temporary variable for storing waiting time

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], get<2>(last_X), get<2>(last_Y), "", "");

    // Start from the result of the previous window
    last_XY.push_back(get<0>(last_X));
    last_XY.push_back(get<0>(last_Y));
    T_X = get<2>(last_X);
    T_Y = get<2>(last_Y);

    // Compute the base cases when there is(are) only one or two vehicle(s)
    if (last_XY == "AB")    // In the previous window, the last vehicle going to Lane X is from Lane A and the last vehicle going to Lane Y is from Lane B
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1], T_X + W_same), max(B[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1], T_X + W_same), max(C[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1], T_X + W_diff), max(C[1], T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1], T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1], T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1], T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1], T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1], T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1], T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1], T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "AC")   // In the previous window, the last vehicle going to Lane X is from Lane A and the last vehicle going to Lane Y is from Lane C
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1], T_X + W_same), max(B[1], T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1], T_X + W_same), max(C[1], T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1], T_X + W_diff), max(C[1], T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1], T_X + W_same), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1], T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1], T_X + W_same), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1], T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1], T_X + W_diff), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1], T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_diff), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1], T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BB")   // In the previous window, the last vehicle going to Lane X is from Lane B and the last vehicle going to Lane Y is from Lane B
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1], T_X + W_diff), max(B[1], T_Y + W_same), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1], T_X + W_diff), max(C[1], T_Y + W_diff), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1], T_X + W_same), max(C[1], T_Y + W_diff), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1], T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1], T_Y + W_same), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1], T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1], T_Y + W_diff), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1], T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1], T_Y + W_diff), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1], T_Y + W_same), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else if (last_XY == "BC")   // In the previous window, the last vehicle going to Lane X is from Lane B and the last vehicle going to Lane Y is from Lane C
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], max(A[1], T_X + W_diff), max(B[1], T_Y + W_diff), "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], max(A[1], T_X + W_diff), max(C[1], T_Y + W_same), "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], max(B[1], T_X + W_same), max(C[1], T_Y + W_same), "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], max(A[1], T_X + W_diff), T_Y, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], T_X, max(B[1], T_Y + W_diff), "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], max(A[1], T_X + W_diff), T_Y, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], T_X, max(C[1], T_Y + W_same), "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], max(B[1], T_X + W_same), T_Y, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], T_X, max(C[1], T_Y + W_same), "BC", "Y");
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_same), T_Y, "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], T_X, max(B[1], T_Y + W_diff), "BB", "Y");
        L_BB[0][1][0] = choose_best_sol(L_BB[0][1][0], tmpSolVec);
        tmpSolVec.clear();
    }
    else    // No previous window (This is the first window)
    {
        L_AB[1][1][0] = update_sol(L_AB[1][1][0], A[1], B[1], "AB", "XY");
        L_AC[1][0][1] = update_sol(L_AC[1][0][1], A[1], C[1], "AC", "XY");
        L_BC[0][1][1] = update_sol(L_BC[0][1][1], B[1], C[1], "BC", "XY");

        L_AB[1][0][0] = update_sol(L_AB[1][0][0], A[1], -W_diff, "AB", "X");
        L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, B[1], "AB", "Y");
        L_AC[1][0][0] = update_sol(L_AC[1][0][0], A[1], -W_diff, "AC", "X");
        L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, C[1], "AC", "Y");
        L_BC[0][1][0] = update_sol(L_BC[0][1][0], B[1], -W_diff, "BC", "X");
        L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, C[1], "BC", "Y");
        L_BB[0][1][0] = (A[1] <= C[1]) ? update_sol(L_BB[0][1][0], -W_diff, B[1], "BB", "Y") : update_sol(L_BB[0][1][0], B[1], -W_diff, "BB", "X");
    }

    // Compute the base cases when two lanes are empty
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], max(B[j], L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], max(A[i], L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], max(B[j], L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], max(A[i], L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], max(C[k], L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], max(B[j], L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], max(C[k], L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    // Compute the base cases when one lane is empty
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], max(A[i], L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "X");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], max(A[i], L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], max(C[k], L_AC[1][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], max(C[k], L_BC[0][1][k - 1].time[1] + W_same), "BC", "Y");
    if (beta > 1)
    {
        if (last_XY == "AB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], L_AC[i][0][0].time[0] + W_diff), max(B[2], T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], L_AC[i][0][0].time[0] + W_diff), max(B[1], T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_diff), max(B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], T_X + W_diff), max(B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "AC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], L_AC[i][0][0].time[0] + W_diff), max(B[2], T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], L_AC[i][0][0].time[0] + W_diff), max(B[1], T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_diff), max(B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], T_X + W_diff), max(B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BB")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], L_AC[i][0][0].time[0] + W_diff), max(B[2], T_Y + W_same), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], L_AC[i][0][0].time[0] + W_diff), max(B[1], T_Y + W_same), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_same), max(B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], T_X + W_same), max(B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else if (last_XY == "BC")
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], L_AC[i][0][0].time[0] + W_diff), max(B[2], T_Y + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], L_AC[i][0][0].time[0] + W_diff), max(B[1], T_Y + W_diff), "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], T_X + W_same), max(B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], T_X + W_same), max(B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
                L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
        else
        {
            for (int i = 1; i <= alpha; ++i)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[1], L_AC[i][0][0].time[0] + W_diff), B[2], "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[2], L_AC[i][0][0].time[0] + W_diff), B[1], "AC", "YX");
                L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
                tmpSolVec.clear();
            }
            for (int k = 1; k <= gamma; ++k)
            {
                tmpSolVec.resize(2);
                tmpSolVec[0] = update_sol(tmpSolVec[0], B[1], max(B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
                tmpSolVec[1] = update_sol(tmpSolVec[1], B[2], max(B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
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
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], max(B[j], L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], max(A[i], L_AC[i - 1][0][k - 1].time[0] + W_same), max(C[k], L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], max(C[k], L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], max(C[k], L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], max(B[j], L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    // Compute DP tables
    bool isCut = false; // whether we can cut at this step
    int N = max(max(alpha, beta), max(beta, gamma));    // maximum number of vehicles on a lane
    int cut_i = alpha, cut_j = beta, cut_k = gamma; // cutting point

    // Find where to cut
    for (int square = 1; square <= N; ++square)
    {
        for (int k = 1; k < square && k <= gamma; ++k)
        {
            // Find on j-axis
            for (int i = square, j = 1; j <= square && i <= alpha && j <= beta; ++j)
            {
                // L_AB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(B[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(B[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]), then we can cut at L_AB[i][j][k]
                if ((i == alpha || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(C[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(C[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]), then we can cut at L_AC[i][j][k]
                if ((i == alpha || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }
                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(C[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(C[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]), then we can cut at L_BC[i][j][k]
                if ((i == alpha || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }
                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1] + W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1] + W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(B[j], max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0] + W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(B[j], max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0] + W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]), then we can cut at L_BB[i][j][k]
                if ((i == alpha || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }

                // If we can cut at this step, set the cutting point and break
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

            // Find on i-axis
            for (int j = square, i = 1; i <= square && i <= alpha && j <= beta; ++i)
            {

                // L_AB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(B[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(B[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]), then we can cut at L_AB[i][j][k]
                if ((i == alpha || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(C[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(C[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]), then we can cut at L_AC[i][j][k]
                if ((i == alpha || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(C[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(C[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]), then we can cut at L_BC[i][j][k]
                if ((i == alpha || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1]+W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1]+W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(B[j], max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0]+W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(B[j], max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0]+W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
                // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]), then we can cut at L_BB[i][j][k]
                if ((i == alpha || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= A[i + 1]) &&
                    (j == beta || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= B[j + 1]) &&
                    (k == gamma || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= C[k + 1]))
                {
                    isCut = true;
                }

                // If we can cut at this step, set the cutting point and break
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
            // Find on the new surface (the (square)-th level on k-axis)
            int k = square;
            for (int i = 1; i <= square && i <= alpha; ++i)
            {
                for (int j = 1; j <= square && j <= beta; ++j)
                {

                    // L_AB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], max(B[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], max(B[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                    L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]), then we can cut at L_AB[i][j][k]
                    if ((i == alpha || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= A[i + 1]) &&
                        (j == beta || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= B[j + 1]) &&
                        (k == gamma || max(L_AB[i][j][k].time[0], L_AB[i][j][k].time[1]) <= C[k + 1]))
                    {
                        isCut = true;
                    }

                    // L_AC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(A[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(A[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], max(C[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], max(C[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                    L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]), then we can cut at L_AC[i][j][k]
                    if ((i == alpha || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= A[i + 1]) &&
                        (j == beta || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= B[j + 1]) &&
                        (k == gamma || max(L_AC[i][j][k].time[0], L_AC[i][j][k].time[1]) <= C[k + 1]))
                    {
                        isCut = true;
                    }

                    // L_BC
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], max(C[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], max(C[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                    L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]), then we can cut at L_BC[i][j][k]
                    if ((i == alpha || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= A[i + 1]) &&
                        (j == beta || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= B[j + 1]) &&
                        (k == gamma || max(L_BC[i][j][k].time[0], L_BC[i][j][k].time[1]) <= C[k + 1]))
                    {
                        isCut = true;
                    }

                    // L_BB
                    tmpSolVec.resize(4);
                    tmpSolVec[0] = update_sol(tmpSolVec[0], max(B[j], L_BB[i][j - 1][k].time[0] + W_same), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                    tmpSolVec[1] = update_sol(tmpSolVec[1], max(B[j], L_AB[i][j - 1][k].time[0] + W_diff), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                    tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], max(B[j], L_BB[i][j - 1][k].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][k]);
                    tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], max(B[j], L_BC[i][j - 1][k].time[1] + W_diff), "BC", "Y", &L_BC[i][j - 1][k]);
                    L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                    tmpSolVec.clear();
                    // If A_{i+1}, B_{j+1}, C_{k+1} >=  L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]), then we can cut at L_BB[i][j][k]
                    if ((i == alpha || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= A[i + 1]) &&
                        (j == beta || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= B[j + 1]) &&
                        (k == gamma || max(L_BB[i][j][k].time[0], L_BB[i][j][k].time[1]) <= C[k + 1]))
                    {
                        isCut = true;
                    }

                    // If we can cut at this step, set the cutting point and break
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

    stack<tuple<char, int, double>> stack_X;    // stack for storing the passing order of vehicles going to Lane X
    stack<tuple<char, int, double>> stack_Y;    // stack for storing the passing order of vehicles going to Lane Y
    int i = cut_i;
    int j = cut_j;
    int k = cut_k;
    // string optTable;
    string table = "";  // table that this solution is derived from
    string lanes = "";  // lanes that the last vehicle goes to (or the last two vehicles go to)

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

    // Backtrack the tables and push the passing order to the stacks
    while (i > 0 || j > 0 || k > 0)
    {
        if (i < 0 || j < 0 || k < 0)
            exit(1);
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

    // Pop the stack to get the passing order for Lane X and calculate waiting time (difference between scheduled entering time and earliest arrival time)
    while (stack_X.size() > 1)
    {
        if (get<0>(stack_X.top()) == 'A')
            wait_time += (get<2>(stack_X.top()) - A[get<1>(stack_X.top())]);
        else
            wait_time += (get<2>(stack_X.top()) - B[get<1>(stack_X.top())]);
        stack_X.pop();
    }
    if (get<0>(stack_X.top()) == 'A')
        wait_time += (get<2>(stack_X.top()) - A[get<1>(stack_X.top())]);
    else
        wait_time += (get<2>(stack_X.top()) - B[get<1>(stack_X.top())]);
    last_X = stack_X.top();

    // Pop the stack to get the passing order for Lane Y and calculate waiting time (difference between scheduled entering time and earliest arrival time)
    while (stack_Y.size() > 1)
    {
        if (get<0>(stack_Y.top()) == 'C')
            wait_time += (get<2>(stack_Y.top()) - C[get<1>(stack_Y.top())]);
        else
            wait_time += (get<2>(stack_Y.top()) - B[get<1>(stack_Y.top())]);
        stack_Y.pop();
    }
    if (get<0>(stack_Y.top()) == 'C')
        wait_time += (get<2>(stack_Y.top()) - C[get<1>(stack_Y.top())]);
    else
        wait_time += (get<2>(stack_Y.top()) - B[get<1>(stack_Y.top())]);
    last_Y = stack_Y.top();

    return make_tuple(last_X, last_Y, cut_i, cut_j, cut_k, wait_time);
}

tuple<double, double, double> schedule_by_pruned_dp(vector<double> A, vector<double> B, vector<double> C)
{
    auto t0 = chrono::high_resolution_clock::now(); // starting time of computation
    tuple<char, int, double> last_X = make_tuple('0', 0, 0.0);  // (lane, index, scheduled entering time) of the last vehicle going to Lane X
    tuple<char, int, double> last_Y = make_tuple('0', 0, 0.0);  // (lane, index, scheduled entering time) of the last vehicle going to Lane Y
    int cut_i = 0, cut_j = 0, cut_k = 0;    // cutting point
    double total_wait = 0;  // total waiting time (difference between scheduled entering time and earliest arrival time)
    double wait_time = 0;   // temporary variable for storing waiting time
    int vehicle_num = A.size() + B.size() + C.size() - 3;   // total number of vehicles

    // First cut
    tie(last_X, last_Y, cut_i, cut_j, cut_k, wait_time) = pruned_dp(A, B, C, last_X, last_Y);
    total_wait += wait_time;

    while (A.size() > 1 || B.size() > 1 || C.size() > 1)
    {
        // Remove the part that we have computed, then we can start next computation from the cutting point 
        A.erase(A.begin() + 1, A.begin() + cut_i + 1);
        B.erase(B.begin() + 1, B.begin() + cut_j + 1);
        C.erase(C.begin() + 1, C.begin() + cut_k + 1);
        
        if (A.size() > 1 && B.size() > 1 && C.size() > 1)   // if no lane is empty
        {
            tie(last_X, last_Y, cut_i, cut_j, cut_k, wait_time) = pruned_dp(A, B, C, last_X, last_Y);   //  compute from the cutting point and find next cut
            total_wait += wait_time;
        }
        else if (A.size() > 1 && B.size() > 1)  // if Lane C is empty
        {
            A.erase(A.begin()); // remove A_0 for using schedule_single_lane()
            B.erase(B.begin()); // remove B_0 for using schedule_single_lane()
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X);  // let all vehicles on Lane A go to Lane X
            total_wait += wait_time;
            tie(last_Y, wait_time) = schedule_single_lane('B', B, last_Y);  // let all vehicles on Lane B go to Lane Y
            total_wait += wait_time;
            A.clear();
            B.clear();
        }
        else if (A.size() > 1 and C.size() > 1) // if Lane B is empty
        {
            A.erase(A.begin()); // remove A_0 for using schedule_single_lane()
            C.erase(C.begin()); // remove C_0 for using schedule_single_lane()
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X);  // let all vehicles on Lane A go to Lane X
            total_wait += wait_time;
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y);  // let all vehicles on Lane C go to Lane Y
            total_wait += wait_time;
            A.clear();
            C.clear();
        }
        else if (B.size() > 1 and C.size() > 1) // if Lane A is empty
        {
            B.erase(B.begin()); // remove B_0 for using schedule_single_lane()
            C.erase(C.begin()); // remove C_0 for using schedule_single_lane()
            tie(last_X, wait_time) = schedule_single_lane('B', B, last_X);  // let all vehicles on Lane B go to Lane X
            total_wait += wait_time;
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y);  // let all vehicles on Lane C go to Lane Y
            total_wait += wait_time;
            B.clear();
            C.clear();
        }
        else if (A.size() > 1)  // if only Lane A is not empty
        {
            A.erase(A.begin()); // remove A_0 for using schedule_single_lane()
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X);  // let all vehicles on Lane A go to Lane X
            total_wait += wait_time;
            A.clear();
        }
        else if (C.size() > 1)  // if only Lane C is not empty
        {
            C.erase(C.begin()); // remove C_0 for using schedule_single_lane()
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y);  // let all vehicles on Lane C go to Lane Y
            total_wait += wait_time;
            C.clear();
        }
        else if (B.size() > 1)  // if only Lane B is not empty
        {
            // Zipper merge
            if (get<2>(last_X) < get<2>(last_Y))    // if the previous vehicle going to Lane X passes earlier than the previous vehicle going to Lane Y
            {
                // Let the currently first vehicle go to Lane X
                if (get<0>(last_X) == '0')  // if no previous vehicle
                    last_X = make_tuple('B', 1, B[1]);
                else if (get<0>(last_X) == 'A') // if the previous vehicle going to Lane X is from Lane B
                    last_X = make_tuple('B', 1, max(B[1], get<2>(last_X) + W_diff));
                else    // if the previous vehicle going to Lane X is also from Lane A
                    last_X = make_tuple('B', 1, max(B[1], get<2>(last_X) + W_same));
                total_wait += (get<2>(last_X) - B[1]);
                if (B.size() > 2)
                {
                    // Let the currently second vehicle go to Lane Y
                    if (get<0>(last_Y) == '0')
                        last_Y = make_tuple('B', 2, B[2]);
                    else if (get<0>(last_Y) == 'C')
                        last_Y = make_tuple('B', 2, max(B[2], get<2>(last_Y) + W_diff));
                    else
                        last_Y = make_tuple('B', 2, max(B[2], get<2>(last_Y) + W_same));
                    total_wait += (get<2>(last_Y) - B[2]);
                    for (int i = 3; i < B.size(); ++i)
                    {
                        if (i % 2 == 1) // let vehicles with odd index go to Lane X
                        {
                            last_X = make_tuple('B', i, max(B[i], get<2>(last_X) + W_same));
                            total_wait += (get<2>(last_X) - B[i]);
                        }
                        else    // let vehicles with even index go to Lane Y
                        {
                            last_Y = make_tuple('B', i, max(B[i], get<2>(last_Y) + W_same));
                            total_wait += (get<2>(last_Y) - B[i]);
                        }
                    }
                }
            }
            else
            {
                // Let the currently first vehicle go to Lane Y
                if (get<0>(last_Y) == '0')
                    last_Y = make_tuple('B', 1, B[1]);
                else if (get<0>(last_Y) == 'C')
                    last_Y = make_tuple('B', 1, max(B[1], get<2>(last_Y) + W_diff));
                else
                    last_Y = make_tuple('B', 1, max(B[1], get<2>(last_Y) + W_same));
                total_wait += (get<2>(last_Y) - B[1]);
                if (B.size() > 2)
                {
                    // Let the currently second vehicle go to Lane X
                    if (get<0>(last_X) == '0')
                        last_X = make_tuple('B', 2, B[2]);
                    else if (get<0>(last_X) == 'A')
                        last_X = make_tuple('B', 2, max(B[2], get<2>(last_X) + W_diff));
                    else
                        last_X = make_tuple('B', 2, max(B[2], get<2>(last_X) + W_same));
                    total_wait += (get<2>(last_X) - B[2]);
                    for (int i = 3; i < B.size(); ++i)
                    {
                        if (i % 2 == 1) // let vehicles with odd index go to Lane Y
                        {
                            last_Y = make_tuple('B', i, max(B[i], get<2>(last_Y) + W_same));
                            total_wait += (get<2>(last_Y) - B[i]);
                        }
                        else    // let vehicles with even index go to Lane X
                        {
                            last_X = make_tuple('B', i, max(B[i], get<2>(last_X) + W_same));
                            total_wait += (get<2>(last_X) - B[i]);
                        }
                    }
                }
            }
            B.clear();
        }
    }
    double T_last = max(get<2>(last_X), get<2>(last_Y));    // scheduled entering time of the last vehicle
    double T_delay = total_wait / vehicle_num;  // average difference betweeb each vehicle's scheduled entering time and its earliest arrival time
    auto t1 = chrono::high_resolution_clock::now(); // ending time of computation
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();  // computation time
    totalComputeTime *= 1e-9;

    return {T_last, T_delay, totalComputeTime};
}
