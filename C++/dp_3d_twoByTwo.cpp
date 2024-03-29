#include <dp_3d_twoByTwo.h>

tuple<tuple<char, int, double>, tuple<char, int, double>, double> window_oneSol_dp_v1(vector<double> a, vector<double> b, vector<double> c, double W_same, double W_diff, tuple<char, int, double> last_X, tuple<char, int, double> last_Y)
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
    stack<tuple<char, int, double>> stack_X, stack_Y;
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

tuple<double, double> schedule_by_window_dp_v1(vector<double> a_all, vector<double> b_all, vector<double> c_all, double W_same, double W_diff, int carNum)
{
    tuple<char, int, double> last_X = make_tuple('0', 0, 0.0);
    tuple<char, int, double> last_Y = make_tuple('0', 0, 0.0);
    double tmp;
    double T_last;
    auto t0 = chrono::high_resolution_clock::now();

    while (a_all.size() > 1 || b_all.size() > 1 || c_all.size() > 1)
    {
        vector<double> a = get_window_by_num(a_all, carNum);
        vector<double> b = get_window_by_num(b_all, carNum);
        vector<double> c = get_window_by_num(c_all, carNum);
        if (a.size() > 1 && b.size() > 1 && c.size() > 1)
            tie(last_X, last_Y, tmp) = window_oneSol_dp_v1(a, b, c, W_same, W_diff, last_X, last_Y);
        else if (a.size() > 1 && b.size() > 1)
        {
            a.erase(a.begin());
            b.erase(b.begin());
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('B', b, W_same, W_diff, last_Y);
        }
        else if (a.size() > 1 and c.size() > 1)
        {
            a.erase(a.begin());
            c.erase(c.begin());
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        }
        else if (b.size() > 1 and c.size() > 1)
        {
            b.erase(b.begin());
            c.erase(c.begin());
            last_X = schedule_single_lane('B', b, W_same, W_diff, last_X);
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
        }
        else if (a.size() > 1)
        {
            a.erase(a.begin());
            last_X = schedule_single_lane('A', a, W_same, W_diff, last_X);
        }
        else if (c.size() > 1)
        {
            c.erase(c.begin());
            last_Y = schedule_single_lane('C', c, W_same, W_diff, last_Y);
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
