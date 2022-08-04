#include "group_dp.h"

// Calculate the scheduled entering time of the last vehicles in a group
double get_tail_time(vector<double> &traffic, pair<int, int> index, double head_time)
{
    double tail_time = max(traffic[index.first], head_time);
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i], tail_time + W_same);
    }
    return tail_time;
}

// Calculate the waiting time of vehicles in a group
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

// Grouping 3D DP-based algorithm
pair<double, double> grouped_dp(vector<double> A, vector<double> B, vector<double> C, vector<pair<int, int>> grouped_A, vector<pair<int, int>> grouped_B, vector<pair<int, int>> grouped_C)
{
    int alpha = grouped_A.size() - 1;   // number of groups on Lane A
    int beta = grouped_B.size() - 1;    // number of groups on Lane B
    int gamma = grouped_C.size() - 1;   // number of groups on Lane C
    vector<vector<vector<Solution>>> L_AB, L_AC, L_BB, L_BC;    // DP tables
    vector<Solution> tmpSolVec; // temporary vector for storing possible solutions
    double T_last;  // scheduled entering time of the last vehicle
    double total_wait = 0;  // total waiting time (difference between scheduled entering time and earliest arrival time)
    int vehicle_num = A.size() + B.size() + C.size() - 3;   // total number of vehicles

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], 0, 0, "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], 0, 0, "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], 0, 0, "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], 0, 0, "", "");

    // Compute the base cases when there is only one vehicle
    L_AB[1][0][0] = update_sol(L_AB[1][0][0], get_tail_time(A, grouped_A[1], 0), -W_diff, "AB", "X");
    L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, get_tail_time(B, grouped_B[1], 0), "AB", "Y");
    L_AC[1][0][0] = update_sol(L_AC[1][0][0], get_tail_time(A, grouped_A[1], 0), -W_diff, "AC", "X");
    L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, get_tail_time(C, grouped_C[1], 0), "AC", "Y");
    L_BC[0][1][0] = update_sol(L_BC[0][1][0], get_tail_time(B, grouped_B[1], 0), -W_diff, "BC", "X");
    L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, get_tail_time(C, grouped_C[1], 0), "BC", "Y");
    L_BB[0][1][0] = (A[1] <= C[1]) ? update_sol(L_BB[0][1][0], -W_diff, get_tail_time(B, grouped_B[1], 0), "BB", "Y") : update_sol(L_BB[0][1][0], get_tail_time(B, grouped_B[1], 0), -W_diff, "BB", "X");

    // Compute the base cases when there are only two vehicles
    L_AB[1][1][0] = update_sol(L_AB[1][1][0], get_tail_time(A, grouped_A[1], 0), get_tail_time(B, grouped_B[1], 0), "AB", "XY");
    L_AC[1][0][1] = update_sol(L_AC[1][0][1], get_tail_time(A, grouped_A[1], 0), get_tail_time(C, grouped_C[1], 0), "AC", "XY");
    L_BC[0][1][1] = update_sol(L_BC[0][1][1], get_tail_time(B, grouped_B[1], 0), get_tail_time(C, grouped_C[1], 0), "BC", "XY");
    
    // Compute the base cases when two lanes are empty
    for (int j = 2; j <= beta; ++j)
    {
        tmpSolVec.resize(2);
        tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BB[0][j - 1][0].time[0] + W_same), L_BB[0][j - 1][0].time[1], "BB", "X");
        tmpSolVec[1] = update_sol(tmpSolVec[1], L_BB[0][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_BB[0][j - 1][0].time[1] + W_same), "BB", "Y");
        L_BB[0][j][0] = choose_best_sol(L_BB[0][j][0], tmpSolVec);
        tmpSolVec.clear();
    }
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][0][0] = update_sol(L_AB[i][0][0], get_tail_time(A, grouped_A[i], L_AB[i - 1][0][0].time[0] + W_same), L_AB[i - 1][0][0].time[1], "AB", "X");
    for (int j = 2; j <= beta; ++j)
        L_AB[0][j][0] = update_sol(L_AB[0][j][0], L_AB[0][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_AB[0][j - 1][0].time[1] + W_same), "AB", "Y");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][0] = update_sol(L_AC[i][0][0], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][0].time[0] + W_same), L_AC[i - 1][0][0].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[0][0][k] = update_sol(L_AC[0][0][k], L_AC[0][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[0][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int j = 2; j <= beta; ++j)
        L_BC[0][j][0] = update_sol(L_BC[0][j][0], get_tail_time(B, grouped_B[j], L_BC[0][j - 1][0].time[0] + W_same), L_BC[0][j - 1][0].time[1], "BC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][0][k] = update_sol(L_BC[0][0][k], L_BC[0][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][0][k - 1].time[1] + W_same), "BC", "Y");

    // Compute the base cases when one lane is empty
    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], get_tail_time(A, grouped_A[i], L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "X");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[1][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][1][k - 1].time[1] + W_same), "BC", "Y");
    if (beta > 1)
    {
        for (int i = 1; i <= alpha; ++i)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[1], L_AC[i][0][0].time[0] + W_diff), get_tail_time(B, grouped_B[2], 0), "AC", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[2], L_AC[i][0][0].time[0] + W_diff), get_tail_time(B, grouped_B[1], 0), "AC", "YX");
            L_BB[i][2][0] = choose_best_sol(L_BB[i][2][0], tmpSolVec);
            tmpSolVec.clear();
        }
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(2);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[1], 0), get_tail_time(B, grouped_B[2], L_AC[0][0][k].time[1] + W_diff), "AC", "XY");
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[2], 0), get_tail_time(B, grouped_B[1], L_AC[0][0][k].time[1] + W_diff), "AC", "YX");
            L_BB[0][2][k] = choose_best_sol(L_BB[0][2][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 2; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(A, grouped_A[i], L_AB[i - 1][j][0].time[0] + W_same), L_AB[i - 1][j][0].time[1], "AB", "X", &L_AB[i - 1][j][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(A, grouped_A[i], L_BB[i - 1][j][0].time[0] + W_diff), L_BB[i - 1][j][0].time[1], "BB", "X", &L_BB[i - 1][j][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][0].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][0]);
            L_AB[i][j][0] = choose_best_sol(L_AB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 2; i <= alpha; ++i)
    {
        for (int k = 2; k <= gamma; ++k)
            L_AC[i][0][k] = update_sol(L_AC[i][0][k], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][k - 1].time[0] + W_same), get_tail_time(C, grouped_C[k], L_AC[i - 1][0][k - 1].time[1] + W_same), "AC", "XY");
    }
    for (int j = 2; j <= beta; ++j)
    {
        for (int k = 1; k <= gamma; ++k)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BC[0][j - 1][k].time[0] + W_same), L_BC[0][j - 1][k].time[1], "BC", "X", &L_BC[0][j - 1][k]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], L_BC[0][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[0][j][k - 1]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[0][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BB[0][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[0][j][k - 1]);
            L_BC[0][j][k] = choose_best_sol(L_BC[0][j][k], tmpSolVec);
            tmpSolVec.clear();
        }
    }
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 3; j <= beta; ++j)
        {
            tmpSolVec.resize(3);
            tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BB[i][j - 1][0].time[0] + W_same), L_BB[i][j - 1][0].time[1], "BB", "X", &L_BB[i][j - 1][0]);
            tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][0].time[0] + W_diff), L_AB[i][j - 1][0].time[1], "AB", "X", &L_AB[i][j - 1][0]);
            tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][0].time[0], get_tail_time(B, grouped_B[j], L_BB[i][j - 1][0].time[1] + W_same), "BB", "Y", &L_BB[i][j - 1][0]);
            L_BB[i][j][0] = choose_best_sol(L_BB[i][j][0], tmpSolVec);
            tmpSolVec.clear();
        }
    }

    // Compute DP tables
    for (int i = 1; i <= alpha; ++i)
    {
        for (int j = 1; j <= beta; ++j)
        {
            for (int k = 1; k <= gamma; ++k)
            {
                // L_AB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(A, grouped_A[i], L_AB[i - 1][j][k].time[0] + W_same), L_AB[i - 1][j][k].time[1], "AB", "X", &L_AB[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(A, grouped_A[i], L_BB[i - 1][j][k].time[0] + W_diff), L_BB[i - 1][j][k].time[1], "BB", "X", &L_BB[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AB[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], L_AB[i][j - 1][k].time[1] + W_same), "AB", "Y", &L_AB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AC[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], L_AC[i][j - 1][k].time[1] + W_diff), "AC", "Y", &L_AC[i][j - 1][k]);
                L_AB[i][j][k] = choose_best_sol(L_AB[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_AC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(A, grouped_A[i], L_AC[i - 1][j][k].time[0] + W_same), L_AC[i - 1][j][k].time[1], "AC", "X", &L_AC[i - 1][j][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(A, grouped_A[i], L_BC[i - 1][j][k].time[0] + W_diff), L_BC[i - 1][j][k].time[1], "BC", "X", &L_BC[i - 1][j][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_AC[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[i][j][k - 1].time[1] + W_same), "AC", "Y", &L_AC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_AB[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AB[i][j][k - 1].time[1] + W_diff), "AB", "Y", &L_AB[i][j][k - 1]);
                L_AC[i][j][k] = choose_best_sol(L_AC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BC
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], L_BC[i][j - 1][k].time[0] + W_same), L_BC[i][j - 1][k].time[1], "BC", "X", &L_BC[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[j], L_AC[i][j - 1][k].time[0] + W_diff), L_AC[i][j - 1][k].time[1], "AC", "X", &L_AC[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BC[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[i][j][k - 1].time[1] + W_same), "BC", "Y", &L_BC[i][j][k - 1]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BB[i][j][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BB[i][j][k - 1].time[1] + W_diff), "BB", "Y", &L_BB[i][j][k - 1]);
                L_BC[i][j][k] = choose_best_sol(L_BC[i][j][k], tmpSolVec);
                tmpSolVec.clear();

                // L_BB
                tmpSolVec.resize(4);
                tmpSolVec[0] = update_sol(tmpSolVec[0], get_tail_time(B, grouped_B[j], max(L_BB[i][j - 1][k].time[0] + W_same, L_BB[i][j - 1][k].time[1] + W_same)), L_BB[i][j - 1][k].time[1], "BB", "X", &L_BB[i][j - 1][k]);
                tmpSolVec[1] = update_sol(tmpSolVec[1], get_tail_time(B, grouped_B[j], max(L_AB[i][j - 1][k].time[0] + W_diff, L_AB[i][j - 1][k].time[1] + W_same)), L_AB[i][j - 1][k].time[1], "AB", "X", &L_AB[i][j - 1][k]);
                tmpSolVec[2] = update_sol(tmpSolVec[2], L_BB[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], max(L_BB[i][j - 1][k].time[1] + W_same, L_BB[i][j - 1][k].time[0] + W_same)), "BB", "Y", &L_BB[i][j - 1][k]);
                tmpSolVec[3] = update_sol(tmpSolVec[3], L_BC[i][j - 1][k].time[0], get_tail_time(B, grouped_B[j], max(L_BC[i][j - 1][k].time[1] + W_diff, L_BC[i][j - 1][k].time[0] + W_same)), "BC", "Y", &L_BC[i][j - 1][k]);
                L_BB[i][j][k] = choose_best_sol(L_BB[i][j][k], tmpSolVec);
                tmpSolVec.clear();
            }
        }
    }

    // Push order to stack
    stack<tuple<char, int, double>> stack_X;    // stack for storing the passing order of vehicles going to Lane X
    stack<tuple<char, int, double>> stack_Y;    // stack for storing the passing order of vehicles going to Lane Y
    int i = alpha;
    int j = beta;
    int k = gamma;
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

    double prev_tail = -W_diff; // scheduled entering time of the last vehicle in the previous group
    char prev_lane = '0';   // incoming lane of the previous group
    char curr_lane; // incoming lane of this group
    // Pop the stack to get the passing order for Lane X and calculate waiting time (difference between scheduled entering time and earliest arrival time)
    while (stack_X.size() > 1)
    {
        curr_lane = get<0>(stack_X.top());
        if (curr_lane == 'A')
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_diff);
        }
        else
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(B, grouped_B[get<1>(stack_X.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(B, grouped_B[get<1>(stack_X.top())], prev_tail + W_diff);
        }
        prev_lane = curr_lane;
        prev_tail = get<2>(stack_X.top());
        stack_X.pop();
    }
    curr_lane = get<0>(stack_X.top());
    if (curr_lane == 'A')
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_diff);
    }
    else
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(B, grouped_B[get<1>(stack_X.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(B, grouped_B[get<1>(stack_X.top())], prev_tail + W_diff);
    }
    T_last = get<2>(stack_X.top());

    // Pop the stack to get the passing order for Lane Y and calculate waiting time (difference between scheduled entering time and earliest arrival time)
    prev_tail = -W_diff;
    prev_lane = '0';
    while (stack_Y.size() > 1)
    {
        curr_lane = get<0>(stack_Y.top());
        if (curr_lane == 'C')
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_diff);
        }
        else
        {
            if (prev_lane == curr_lane)
                total_wait += get_wait_time(B, grouped_B[get<1>(stack_Y.top())], prev_tail + W_same);
            else
                total_wait += get_wait_time(B, grouped_B[get<1>(stack_Y.top())], prev_tail + W_diff);
        }
        prev_lane = curr_lane;
        prev_tail = get<2>(stack_Y.top());
        stack_Y.pop();
    }
    curr_lane = get<0>(stack_Y.top());
    if (curr_lane == 'C')
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_diff);
    }
    else
    {
        if (prev_lane == curr_lane)
            total_wait += get_wait_time(B, grouped_B[get<1>(stack_Y.top())], prev_tail + W_same);
        else
            total_wait += get_wait_time(B, grouped_B[get<1>(stack_Y.top())], prev_tail + W_diff);
    }
    T_last = max(T_last, get<2>(stack_Y.top()));
    double T_delay = total_wait / vehicle_num;      // average difference betweeb each vehicle's scheduled entering time and its earliest arrival time

    return {T_last, T_delay};
}

tuple<double, double, double> schedule_by_group_dp(vector<double> A_all, vector<double> B_all, vector<double> C_all)
{
    auto t0 = chrono::high_resolution_clock::now(); // starting time of computation
    // Grouping
    vector<pair<int, int>> grouped_A = grouping(A_all); // get the vector of pairs of the starting index and the ending index of each group on Lane A
    vector<pair<int, int>> grouped_B = grouping(B_all); // get the vector of pairs of the starting index and the ending index of each group on Lane B
    vector<pair<int, int>> grouped_C = grouping(C_all); // get the vector of pairs of the starting index and the ending index of each group on Lane C
    // Solve the passing order of the groups with 3D DP-based algorithm
    pair<double, double> res = grouped_dp(A_all, B_all, C_all, grouped_A, grouped_B, grouped_C);
    double T_last = res.first;  // scheduled entering time of the last vehicle
    double T_delay = res.second;    // average difference betweeb each vehicle's scheduled entering time and its earliest arrival time
    auto t1 = chrono::high_resolution_clock::now(); // ending time of computation
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();  // computation time
    totalComputeTime *= 1e-9;

    return {T_last, T_delay, totalComputeTime};
}