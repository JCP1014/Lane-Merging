#include "group_dp.h"

double get_tail_time(vector<double> &traffic, pair<int, int> index, double head_time)
{
    double tail_time = max(traffic[index.first], head_time);
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i], tail_time + W_same);
    }
    return tail_time;
}

double get_wait_time(vector<double> &traffic, pair<int, int> index, double head_time)
{
    double tail_time = max(traffic[index.first], head_time);
    double wait_time = tail_time - traffic[index.first];
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i], tail_time + W_same);
        wait_time += (tail_time - traffic[i]);
    }
    return wait_time;
}

pair<double, double> grouped_dp(vector<double> a, vector<double> b, vector<double> c, vector<pair<int, int>> grouped_a, vector<pair<int, int>> grouped_b, vector<pair<int, int>> grouped_c)
{
    int alpha = grouped_a.size() - 1;
    int beta = grouped_b.size() - 1;
    int gamma = grouped_c.size() - 1;
    vector<vector<vector<Solution>>> L_AB;
    vector<vector<vector<Solution>>> L_AC;
    vector<vector<vector<Solution>>> L_BB;
    vector<vector<vector<Solution>>> L_BC;
    vector<Solution> tmpSolVec;
    double T_last;
    double total_wait = 0;
    int vehicle_num = a.size() + b.size() + c.size() - 3;

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], 0, 0, "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], 0, 0, "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], 0, 0, "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], 0, 0, "", "");

    L_AB[1][1][0] = update_sol(L_AB[1][1][0], get_tail_time(a, grouped_a[1], 0), get_tail_time(b, grouped_b[1], 0), "AB", "XY");
    L_AC[1][0][1] = update_sol(L_AC[1][0][1], get_tail_time(a, grouped_a[1], 0), get_tail_time(c, grouped_c[1], 0), "AC", "XY");
    L_BC[0][1][1] = update_sol(L_BC[0][1][1], get_tail_time(b, grouped_b[1], 0), get_tail_time(c, grouped_c[1], 0), "BC", "XY");

    L_AB[1][0][0] = update_sol(L_AB[1][0][0], get_tail_time(a, grouped_a[1], 0), -W_diff, "AB", "X");
    L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, get_tail_time(b, grouped_b[1], 0), "AB", "Y");
    L_AC[1][0][0] = update_sol(L_AC[1][0][0], get_tail_time(a, grouped_a[1], 0), -W_diff, "AC", "X");
    L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, get_tail_time(c, grouped_c[1], 0), "AC", "Y");
    L_BC[0][1][0] = update_sol(L_BC[0][1][0], get_tail_time(b, grouped_b[1], 0), -W_diff, "BC", "X");
    L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, get_tail_time(c, grouped_c[1], 0), "BC", "Y");
    L_BB[0][1][0] = (a[1] <= c[1]) ? update_sol(L_BB[0][1][0], -W_diff, get_tail_time(b, grouped_b[1], 0), "BB", "Y") : update_sol(L_BB[0][1][0], get_tail_time(b, grouped_b[1], 0), -W_diff, "BB", "X");

    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], get_tail_time(a, grouped_a[i], L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "X");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], get_tail_time(a, grouped_a[i], L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], get_tail_time(c, grouped_c[k], L_AC[1][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], get_tail_time(c, grouped_c[k], L_BC[0][1][k - 1].time[1] + W_same), "BC", "Y");
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[j], L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], get_tail_time(b, grouped_b[j], L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], get_tail_time(a, grouped_a[i], L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], get_tail_time(b, grouped_b[j], L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], get_tail_time(a, grouped_a[i], L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], get_tail_time(c, grouped_c[k], L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], get_tail_time(b, grouped_b[j], L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], get_tail_time(c, grouped_c[k], L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    if (beta > 1)
    {
        for (int i = 1; i <= alpha; ++i)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[1], L_AC[i][0][0].time[0] + W_diff), get_tail_time(b, grouped_b[2], 0), "AC", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(b, grouped_b[2], L_AC[i][0][0].time[0] + W_diff), get_tail_time(b, grouped_b[1], 0), "AC", "YX");
            L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[1], 0), get_tail_time(b, grouped_b[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(b, grouped_b[2], 0), get_tail_time(b, grouped_b[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
            L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(a, grouped_a[i], L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(a, grouped_a[i], L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], get_tail_time(b, grouped_b[j], L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], get_tail_time(a, grouped_a[i], L_AC[i - 1][0][k - 1].time[0] + W_same), get_tail_time(c, grouped_c[k], L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[j], L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], get_tail_time(c, grouped_c[k], L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], get_tail_time(c, grouped_c[k], L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[j], L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(b, grouped_b[j], L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], get_tail_time(b, grouped_b[j], L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
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
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(a, grouped_a[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(a, grouped_a[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], get_tail_time(b, grouped_b[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], get_tail_time(b, grouped_b[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(a, grouped_a[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(a, grouped_a[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], get_tail_time(c, grouped_c[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], get_tail_time(c, grouped_c[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(b, grouped_b[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], get_tail_time(c, grouped_c[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], get_tail_time(c, grouped_c[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(b, grouped_b[j], max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1] + W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(b, grouped_b[j], max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1] + W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], get_tail_time(b, grouped_b[j], max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0] + W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], get_tail_time(b, grouped_b[j], max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0] + W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
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
    double prev_tail = -W_diff;
    char prev_lane = '0';
    char curr_lane;
    while (stack_X.size() > 1)
    {
        // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
        curr_lane = get<0>(stack_X.top());
        if (curr_lane == 'A')
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(a, grouped_a[get<1>(stack_X.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(a, grouped_a[get<1>(stack_X.top())], prev_tail + W_diff);
        }
        else
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(b, grouped_b[get<1>(stack_X.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(b, grouped_b[get<1>(stack_X.top())], prev_tail + W_diff);
        }
        prev_lane = curr_lane;
        prev_tail = get<2>(stack_X.top());
        stack_X.pop();
    }
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
    curr_lane = get<0>(stack_X.top());
    if (curr_lane == 'A')
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(a, grouped_a[get<1>(stack_X.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(a, grouped_a[get<1>(stack_X.top())], prev_tail + W_diff);
    }
    else
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(b, grouped_b[get<1>(stack_X.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(b, grouped_b[get<1>(stack_X.top())], prev_tail + W_diff);
    }
    T_last = get<2>(stack_X.top());

    // cout << "Lane Y: " << endl;
    prev_tail = -W_diff;
    prev_lane = '0';
    while (stack_Y.size() > 1)
    {
        // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
        curr_lane = get<0>(stack_Y.top());
        if (curr_lane == 'C')
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(c, grouped_c[get<1>(stack_Y.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(c, grouped_c[get<1>(stack_Y.top())], prev_tail + W_diff);
        }
        else
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(b, grouped_b[get<1>(stack_Y.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(b, grouped_b[get<1>(stack_Y.top())], prev_tail + W_diff);
        }
        prev_lane = curr_lane;
        prev_tail = get<2>(stack_Y.top());
        stack_Y.pop();
    }
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    curr_lane = get<0>(stack_Y.top());
    if (curr_lane == 'C')
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(c, grouped_c[get<1>(stack_Y.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(c, grouped_c[get<1>(stack_Y.top())], prev_tail + W_diff);
    }
    else
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(b, grouped_b[get<1>(stack_Y.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(b, grouped_b[get<1>(stack_Y.top())], prev_tail + W_diff);
    }
    T_last = max(T_last, get<2>(stack_Y.top()));
    double T_delay = total_wait / vehicle_num;

    return {T_last, T_delay};
}

tuple<double, double, double> schedule_by_group_dp(vector<double> a_all, vector<double> b_all, vector<double> c_all)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<pair<int, int>> grouped_a = grouping(a_all);
    vector<pair<int, int>> grouped_b = grouping(b_all);
    vector<pair<int, int>> grouped_c = grouping(c_all);
    pair<double, double> res = grouped_dp(a_all, b_all, c_all, grouped_a, grouped_b, grouped_c);
    double T_last = res.first;
    double T_delay = res.second;
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    // cout << "dp_group result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
    // for (auto &g : grouped_a)
    //     cout << "[" << g.first << ", " << g.second << "]" << endl;
    // for (auto &g : grouped_b)
    //     cout << "[" << g.first << ", " << g.second << "]" << endl;
    // for (auto &g : grouped_c)
    //     cout << "[" << g.first << ", " << g.second << "]" << endl;
    return {T_last, T_delay, totalComputeTime};
}