#include "dp_2d.h"

GreedySol update_greedySol(GreedySol s, float newTime, char newTable)
{
    s.time = newTime;
    s.table = newTable;
    return s;
}

pair<float, double> greedy_dp(vector<float> a_all, vector<float> b_all, vector<float> c_all, float W_same, float W_diff)
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
    // cout << get<0>(stack_X.top()) << " " << get<1>(stack_X.top()) << " " << get<2>(stack_X.top()) << endl;
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
    // cout << get<0>(stack_Y.top()) << " " << get<1>(stack_Y.top()) << " " << get<2>(stack_Y.top()) << endl;
    if (get<2>(stack_Y.top()) > T_last)
        T_last = get<2>(stack_Y.top());

    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    cout << "dp_2d result: " << T_last << " " << totalComputeTime << endl;
    return {T_last, totalComputeTime};
}