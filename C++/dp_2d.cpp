#include "dp_2d.h"

// Update a solution
GreedySol update_greedySol(GreedySol s, double newTime, char newTable)
{
    s.time = newTime;
    s.table = newTable;
    return s;
}

// 2D DP-based algorithm
tuple<double, double, double> dp_2d(vector<double> A_all, vector<double> B_all, vector<double> C_all)
{
    auto t0 = chrono::high_resolution_clock::now(); // starting time of computation
    int alpha = A_all.size() - 1;   // number of vehicles on Lane A
    int beta = B_all.size() - 1;    // number of vehicles on Lane B
    int gamma = C_all.size() - 1;   // number of vehicles on Lane C
    vector<vector<GreedySol>> LX_A, LX_B, LY_C, LY_B;   // DP tables
    int beta_sum = 1;   // number of vehicles (on Lane B) that we have decided its outgoing lane
    int beta_X = 1; // number of vehicles (on Lane B) going to Lane X
    int beta_Y = 1; // number of vehicles (on Lane B) going to Lane Y
    double min_X;   // temporary varible for storing minimum T_last when the vehicle goes to Lane X
    double min_Y;   // temporary varible for storing minimum T_last when the vehicle goes to Lane Y
    double T_last;  // scheduled entering time of the last vehicle
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    double total_wait = 0;  // total waiting time (difference between scheduled entering time and earliest arrival time)

    LX_A.resize(alpha + 1, vector<GreedySol>(beta + 1));
    LX_B.resize(alpha + 1, vector<GreedySol>(beta + 1));
    LY_C.resize(gamma + 1, vector<GreedySol>(beta + 1));
    LY_B.resize(gamma + 1, vector<GreedySol>(beta + 1));

    // Initialize
    LX_A[0][0] = update_greedySol(LX_A[0][0], 0, '0');
    LX_B[0][0] = update_greedySol(LX_B[0][0], 0, '0');
    LX_A[1][0] = update_greedySol(LX_A[1][0], A_all[1], '0');
    LY_C[0][0] = update_greedySol(LY_C[0][0], 0, '0');
    LY_B[0][0] = update_greedySol(LY_B[0][0], 0, '0');
    LY_C[1][0] = update_greedySol(LY_C[1][0], C_all[1], '0');
    for (int i = 2; i <= alpha; ++i)
    {
        LX_A[i][0] = update_greedySol(LX_A[i][0], max(A_all[i], LX_A[i - 1][0].time + W_same), 'A');
    }
    for (int i = 2; i <= gamma; ++i)
    {
        LY_C[i][0] = update_greedySol(LY_C[i][0], max(C_all[i], LY_C[i - 1][0].time + W_same), 'C');
    }

    // Decide the outgoing lane for each vehicle on Lane B 
    while (beta_sum <= beta)
    {
        if (beta_X == 1)    // the first vehicle going to Lane X
            LX_B[0][beta_X] = update_greedySol(LX_B[0][beta_X], B_all[beta_sum], '0');
        else
            LX_B[0][beta_X] = update_greedySol(LX_B[0][beta_X], max(B_all[beta_sum], LX_B[0][beta_X - 1].time + W_same), 'B');

        if (beta_Y == 1)    // the first vehicle going to Lane Y
            LY_B[0][beta_Y] = update_greedySol(LY_B[0][beta_Y], B_all[beta_sum], '0');
        else
            LY_B[0][beta_Y] = update_greedySol(LY_B[0][beta_Y], max(B_all[beta_sum], LY_B[0][beta_Y - 1].time + W_same), 'B');

        // If go to Lane X (merge with Lane A)
        for (int i = 1; i <= alpha; ++i)
        {
            // Compute LX_A
            if (max(A_all[i], LX_A[i - 1][beta_X].time + W_same) <= max(A_all[i], LX_B[i - 1][beta_X].time + W_diff))
            {
                LX_A[i][beta_X] = update_greedySol(LX_A[i][beta_X], max(A_all[i], LX_A[i - 1][beta_X].time + W_same), 'A');
            }
            else
            {
                LX_A[i][beta_X] = update_greedySol(LX_A[i][beta_X], max(A_all[i], LX_B[i - 1][beta_X].time + W_diff), 'B');
            }

            // Compute LX_B
            if (max(B_all[beta_sum], LX_A[i][beta_X - 1].time + W_diff) <= max(B_all[beta_sum], LX_B[i][beta_X - 1].time + W_same))
            {
                LX_B[i][beta_X] = update_greedySol(LX_B[i][beta_X], max(B_all[beta_sum], LX_A[i][beta_X - 1].time + W_diff), 'A');
            }
            else
            {
                LX_B[i][beta_X] = update_greedySol(LX_B[i][beta_X], max(B_all[beta_sum], LX_B[i][beta_X - 1].time + W_same), 'B');
            }
        }

        // If go to Lane Y (merge with Lane C)
        for (int i = 1; i <= gamma; ++i)
        {
            // Compute LY_A
            if (max(C_all[i], LY_C[i - 1][beta_Y].time + W_same) <= max(C_all[i], LY_B[i - 1][beta_Y].time + W_diff))
            {
                LY_C[i][beta_Y] = update_greedySol(LY_C[i][beta_Y], max(C_all[i], LY_C[i - 1][beta_Y].time + W_same), 'C');
            }
            else
            {
                LY_C[i][beta_Y] = update_greedySol(LY_C[i][beta_Y], max(C_all[i], LY_B[i - 1][beta_Y].time + W_diff), 'B');
            }

            // Compute LY_B
            if (max(B_all[beta_sum], LY_C[i][beta_Y - 1].time + W_diff) <= max(B_all[beta_sum], LY_B[i][beta_Y - 1].time + W_same))
            {
                LY_B[i][beta_Y] = update_greedySol(LY_B[i][beta_Y], max(B_all[beta_sum], LY_C[i][beta_Y - 1].time + W_diff), 'C');
            }
            else
            {
                LY_B[i][beta_Y] = update_greedySol(LY_B[i][beta_Y], max(B_all[beta_sum], LY_B[i][beta_Y - 1].time + W_same), 'B');
            }
        }

        min_X = min(LX_A[alpha][beta_X].time, LX_B[alpha][beta_X].time);    // if go to Lane X
        min_Y = min(LY_C[gamma][beta_Y].time, LY_B[gamma][beta_Y].time);    // if go to Lane Y
        if (min_X < min_Y)
            ++beta_X;   // go to Lane X
        else if (min_Y < min_X)
            ++beta_Y;   // go to Lane Y
        else    // randomly decide
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
    stack<tuple<char, int, double>> stack_X;    // stack for storing the passing order of vehicles going to Lane X
    stack<tuple<char, int, double>> stack_Y;    // stack for storing the passing order of vehicles going to Lane Y
    char prevTable; // which table the solution is derived from
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
    // Pop the stack to get the passing order for Lane X and calculate waiting time (difference between scheduled entering time and earliest arrival time)
    while (stack_X.size() > 1)
    {
        if (get<0>(stack_X.top()) == 'A')
            total_wait += (get<2>(stack_X.top()) - A_all[get<1>(stack_X.top())]);
        else
            total_wait += (get<2>(stack_X.top()) - B_all[get<1>(stack_X.top())]);
        stack_X.pop();
    }
    if (get<0>(stack_X.top()) == 'A')
        total_wait += (get<2>(stack_X.top()) - A_all[get<1>(stack_X.top())]);
    else
        total_wait += (get<2>(stack_X.top()) - B_all[get<1>(stack_X.top())]);
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
    // Pop the stack to get the passing order for Lane Y and calculate waiting time (difference between scheduled entering time and earliest arrival time)
    while (stack_Y.size() > 1)
    {
        if (get<0>(stack_Y.top()) == 'C')
            total_wait += (get<2>(stack_Y.top()) - C_all[get<1>(stack_Y.top())]);
        else
            total_wait += (get<2>(stack_Y.top()) - B_all[get<1>(stack_Y.top())]);
        stack_Y.pop();
    }
    if (get<0>(stack_Y.top()) == 'C')
        total_wait += (get<2>(stack_Y.top()) - C_all[get<1>(stack_Y.top())]);
    else
        total_wait += (get<2>(stack_Y.top()) - B_all[get<1>(stack_Y.top())]);
    T_last = max(T_last, get<2>(stack_Y.top()));
    
    double T_delay = total_wait / (alpha + beta + gamma);   // average difference betweeb each vehicle's scheduled entering time and its earliest arrival time
    auto t1 = chrono::high_resolution_clock::now(); // ending time of computation
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();  // computation time
    totalComputeTime *= 1e-9;

    return {T_last, T_delay, totalComputeTime};
}