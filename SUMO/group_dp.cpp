#include "group_dp.h"

double get_tail_time(vector<vehicle> &traffic, pair<int, int> index, double head_time)
{
    double tail_time = max(traffic[index.first].time, head_time);
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i].time, tail_time + W_same);
    }
    return tail_time;
}

void compute_member_time(vector<vehicle> &traffic, pair<int, int> index, double head_time, vector<vehicle> &schedule, double &total_wait)
{
    double tail_time = max(traffic[index.first].time, head_time);
    schedule.push_back(vehicle(traffic[index.first].id, tail_time));
    total_wait += (tail_time - traffic[index.first].time);
    for (int i = index.first + 1; i <= index.second; ++i)
    {
        tail_time = max(traffic[i].time, tail_time + W_same);
        schedule.push_back(vehicle(traffic[i].id, tail_time));
        total_wait += (tail_time - traffic[i].time);
    }
}

void group_dp_compute_entering_time(vector<vehicle> &A, vector<vehicle> &B, vector<vehicle> &C, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<pair<int, int>> grouped_A = grouping(A);
    vector<pair<int, int>> grouped_B = grouping(B);
    vector<pair<int, int>> grouped_C = grouping(C);
    int alpha = grouped_A.size() - 1;
    int beta = grouped_B.size() - 1;
    int gamma = grouped_C.size() - 1;
    vector<vector<vector<Solution>>> L_AB, L_AC, L_BB, L_BC;
    vector<Solution> tmpSolVec;
    double T_last;
    double total_wait = 0;
    int vehicle_num = A.size() + B.size() + C.size() - 3;

    L_AB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_AC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BB.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));
    L_BC.resize(alpha + 1, vector<vector<Solution>>(beta + 1, vector<Solution>(gamma + 1)));

    // Initialize
    L_AB[0][0][0] = update_sol(L_AB[0][0][0], 0, 0, "", "");
    L_AC[0][0][0] = update_sol(L_AC[0][0][0], 0, 0, "", "");
    L_BB[0][0][0] = update_sol(L_BB[0][0][0], 0, 0, "", "");
    L_BC[0][0][0] = update_sol(L_BC[0][0][0], 0, 0, "", "");

    L_AB[1][1][0] = update_sol(L_AB[1][1][0], get_tail_time(A, grouped_A[1], 0), get_tail_time(B, grouped_B[1], 0), "AB", "XY");
    L_AC[1][0][1] = update_sol(L_AC[1][0][1], get_tail_time(A, grouped_A[1], 0), get_tail_time(C, grouped_C[1], 0), "AC", "XY");
    L_BC[0][1][1] = update_sol(L_BC[0][1][1], get_tail_time(B, grouped_B[1], 0), get_tail_time(C, grouped_C[1], 0), "BC", "XY");

    L_AB[1][0][0] = update_sol(L_AB[1][0][0], get_tail_time(A, grouped_A[1], 0), -W_diff, "AB", "X");
    L_AB[0][1][0] = update_sol(L_AB[0][1][0], -W_diff, get_tail_time(B, grouped_B[1], 0), "AB", "Y");
    L_AC[1][0][0] = update_sol(L_AC[1][0][0], get_tail_time(A, grouped_A[1], 0), -W_diff, "AC", "X");
    L_AC[0][0][1] = update_sol(L_AC[0][0][1], -W_diff, get_tail_time(C, grouped_C[1], 0), "AC", "Y");
    L_BC[0][1][0] = update_sol(L_BC[0][1][0], get_tail_time(B, grouped_B[1], 0), -W_diff, "BC", "X");
    L_BC[0][0][1] = update_sol(L_BC[0][0][1], -W_diff, get_tail_time(C, grouped_C[1], 0), "BC", "Y");
    L_BB[0][1][0] = (A[1].time <= C[1].time) ? update_sol(L_BB[0][1][0], -W_diff, get_tail_time(B, grouped_B[1], 0), "BB", "Y") : update_sol(L_BB[0][1][0], get_tail_time(B, grouped_B[1], 0), -W_diff, "BB", "X");

    for (int i = 2; i <= alpha; ++i)
        L_AB[i][1][0] = update_sol(L_AB[i][1][0], get_tail_time(A, grouped_A[i], L_AB[i - 1][1][0].time[0] + W_same), L_AB[i - 1][1][0].time[1], "AB", "X");
    for (int i = 2; i <= alpha; ++i)
        L_AC[i][0][1] = update_sol(L_AC[i][0][1], get_tail_time(A, grouped_A[i], L_AC[i - 1][0][1].time[0] + W_same), L_AC[i - 1][0][1].time[1], "AC", "X");
    for (int k = 2; k <= gamma; ++k)
        L_AC[1][0][k] = update_sol(L_AC[1][0][k], L_AC[1][0][k - 1].time[0], get_tail_time(C, grouped_C[k], L_AC[1][0][k - 1].time[1] + W_same), "AC", "Y");
    for (int k = 2; k <= gamma; ++k)
        L_BC[0][1][k] = update_sol(L_BC[0][1][k], L_BC[0][1][k - 1].time[0], get_tail_time(C, grouped_C[k], L_BC[0][1][k - 1].time[1] + W_same), "BC", "Y");
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
    schedule_A.clear();
    schedule_B.clear();
    schedule_C.clear();
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
                compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_same, schedule_A, total_wait);
            else
                compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_diff, schedule_A, total_wait);
        }
        else
        {
            pair<int, int> index = grouped_B[get<1>(stack_X.top())];
            if (prev_lane == curr_lane)
                compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
            else
                compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
            for (int i = index.first; i <= index.second; ++i)
                Vehicle::setRouteID(B[i].id, "route_1");
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
            compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_same, schedule_A, total_wait);
        else
            compute_member_time(A, grouped_A[get<1>(stack_X.top())], prev_tail + W_diff, schedule_A, total_wait);
    }
    else
    {
        pair<int, int> index = grouped_B[get<1>(stack_X.top())];
        if (prev_lane == curr_lane)
            compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
        else
            compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
        for (int i = index.first; i <= index.second; ++i)
            Vehicle::setRouteID(B[i].id, "route_1");
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
                compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_same, schedule_C, total_wait);
            else
                compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_diff, schedule_C, total_wait);
        }
        else
        {
            pair<int, int> index = grouped_B[get<1>(stack_Y.top())];
            if (prev_lane == curr_lane)
                compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
            else
                compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
            for (int i = index.first; i <= index.second; ++i)
                Vehicle::setRouteID(B[i].id, "route_2");
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
            compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_same, schedule_C, total_wait);
        else
            compute_member_time(C, grouped_C[get<1>(stack_Y.top())], prev_tail + W_diff, schedule_C, total_wait);
    }
    else
    {
        pair<int, int> index = grouped_B[get<1>(stack_Y.top())];
        if (prev_lane == curr_lane)
            compute_member_time(B, index, prev_tail + W_same, schedule_B, total_wait);
        else
            compute_member_time(B, index, prev_tail + W_diff, schedule_B, total_wait);
        for (int i = index.first; i <= index.second; ++i)
            Vehicle::setRouteID(B[i].id, "route_2");
    }
    T_last = max(T_last, get<2>(stack_Y.top()));
    double T_delay = total_wait / vehicle_num;
}
