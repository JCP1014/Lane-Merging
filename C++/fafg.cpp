#include "fafg.h"

// Schedule by First-Arrive-First-Go
tuple<double, double, double> first_arrive_first_go(vector<double> A_all, vector<double> B_all, vector<double> C_all)
{
    auto t0 = chrono::high_resolution_clock::now(); // starting time of computation
    vector<double> A(A_all.begin() + 1, A_all.end());   // ignore the vehicle whose index is 0
    vector<double> B(B_all.begin() + 1, B_all.end());
    vector<double> C(C_all.begin() + 1, C_all.end());
    double X_lastT = -W_diff;   // the scheduled entering time of the last vehicle going to Lane X
    double Y_lastT = -W_diff;   // the scheduled entering time of the last vehicle going to Lane Y
    char X_lastFrom = '0';  // which lane the last vehicle going to Lane X comes from
    char Y_lastFrom = '0';  // which lane the last vehicle going to Lane Y comes from
    tuple<char, int, double> last_X;    // (incoming lane, index, scheduled entering time) of the last vehicle going to Lane X
    tuple<char, int, double> last_Y;    // (incoming lane, index, scheduled entering time) of the last vehicle going to Lane Y
    double first = 0;   // the earliest arrival time of the vehicle that arrives first
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    double total_wait = 0;  // total waiting time (difference between scheduled entering time and earliest arrival time)
    double wait_time = 0;   // temporary variable for storing waiting time
    int vehicle_num = A_all.size() + B_all.size() + C_all.size() - 3;   // total number of vehicles

    while (A.size() > 0 || B.size() > 0 || C.size() > 0)    // while there is a vehicle
    {
        if (A.size() > 0 && B.size() > 0 && C.size() > 0)   // if no lane is empty
        {
            first = min(min(A[0], B[0]), C[0]);
            if (first == A[0])  // if the vehicle arriving first is from Lane A
            {
                if (X_lastFrom == 'A')  // if the previous vehicle going to Lane X is also from Lane A
                    X_lastT = max(A[0], X_lastT + W_same);
                else
                    X_lastT = max(A[0], X_lastT + W_diff);
                X_lastFrom = 'A';
                total_wait += (X_lastT - A[0]);
                A.erase(A.begin()); // remove from vector
            }
            if (first == C[0])  // if the vehicle arriving first is from Lane C
            {
                if (Y_lastFrom == 'C')  // if the previous vehicle going to Lane Y is also from Lane C
                    Y_lastT = max(C[0], Y_lastT + W_same);
                else
                    Y_lastT = max(C[0], Y_lastT + W_diff);
                Y_lastFrom = 'C';
                total_wait += (Y_lastT - C[0]);
                C.erase(C.begin());
            }
            if (first == B[0])  // if the vehicle arriving first is from Lane B
            {
                if (X_lastT < Y_lastT)  // if the previous vehicle going to Lane X passes earlier than the previous vehicle going to Lane Y
                {                       // then let the first vehicle go to Lane X
                    if (X_lastFrom == 'B')  // if the previous vehicle going to Lane X is also from Lane B
                    {
                        if (Y_lastFrom == 'B')  // if the previous vehicle going to Lane Y is also from Lane B
                            X_lastT = max(max(B[0], X_lastT + W_same), Y_lastT + W_same);   // this vehicle should not pass earlier than it
                        else
                            X_lastT = max(B[0], X_lastT + W_same);
                    }
                    else    // else the previous vehicle going to Lane X is from Lane A
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0], X_lastT + W_diff), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0], X_lastT + W_diff);
                    }
                    X_lastFrom = 'B';
                    total_wait += (X_lastT - B[0]);
                    B.erase(B.begin());
                }
                else if (Y_lastT < X_lastT) // if the previous vehicle going to Lane Y passes earlier than the previous vehicle going to Lane X
                {                           // then let the first vehicle go to Lane Y
                    if (Y_lastFrom == 'B')  // if the previous vehicle going to Lane Y is also from Lane B
                    {
                        if (X_lastFrom == 'B')  // if the previous vehicle going to Lane X is also from Lane B
                            Y_lastT = max(max(B[0], Y_lastT + W_same), X_lastT + W_same);    // this vehicle should not pass earlier than it
                        else
                            Y_lastT = max(B[0], Y_lastT + W_same);
                    }
                    else    // else the previous vehicle going to Lane X is from Lane C
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0], Y_lastT + W_diff), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0], Y_lastT + W_diff);
                    }
                    Y_lastFrom = 'B';
                    total_wait += (Y_lastT - B[0]);
                    B.erase(B.begin());
                }
                else    // else the previous vehicle going to Lane Y and the previous vehicle going to Lane X pass at the same time
                {
                    if (A[0] > C[0])    // if the first vehicle on Lane A will arrive earlier than the first vehicle on Lane C
                    {                   // then let the first vehicle go to Lane X
                        if (X_lastFrom == 'B')
                        {
                            if (Y_lastFrom == 'B')
                                X_lastT = max(max(B[0], X_lastT + W_same), Y_lastT + W_same);
                            else
                                X_lastT = max(B[0], X_lastT + W_same);
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                X_lastT = max(max(B[0], X_lastT + W_diff), Y_lastT + W_same);
                            else
                                X_lastT = max(B[0], X_lastT + W_diff);
                        }
                        X_lastFrom = 'B';
                        total_wait += (X_lastT - B[0]);
                        B.erase(B.begin());
                    }
                    else if (C[0] > A[0])   // if the first vehicle on Lane C will arrive earlier than the first vehicle on Lane A
                    {                       // then let the first vehicle go to Lane Y
                        if (Y_lastFrom == 'B')
                        {
                            if (X_lastFrom == 'B')
                                Y_lastT = max(max(B[0], Y_lastT + W_same), X_lastT + W_same);
                            else
                                Y_lastT = max(B[0], Y_lastT + W_same);
                        }
                        else
                        {
                            if (X_lastFrom == 'B')
                                Y_lastT = max(max(B[0], Y_lastT + W_diff), X_lastT + W_same);
                            else
                                Y_lastT = max(B[0], Y_lastT + W_diff);
                        }
                        Y_lastFrom = 'B';
                        total_wait += (Y_lastT - B[0]);
                        B.erase(B.begin());
                    }
                    else    // else randomly decide to let the first vehicle go to Lane X or Lane Y
                    {
                        if (distribution(generator) == 0)   // let it go to Lane X
                        {
                            if (X_lastFrom == 'B')
                            {
                                if (Y_lastFrom == 'B')
                                    X_lastT = max(max(B[0], X_lastT + W_same), Y_lastT + W_same);
                                else
                                    X_lastT = max(B[0], X_lastT + W_same);
                            }
                            else
                            {
                                if (Y_lastFrom == 'B')
                                    X_lastT = max(max(B[0], X_lastT + W_diff), Y_lastT + W_same);
                                else
                                    X_lastT = max(B[0], X_lastT + W_diff);
                            }
                            X_lastFrom = 'B';
                            total_wait += (X_lastT - B[0]);
                            B.erase(B.begin());
                        }
                        else    // let it go to Lane Y
                        {
                            if (Y_lastFrom == 'B')
                            {
                                if (X_lastFrom == 'B')
                                    Y_lastT = max(max(B[0], Y_lastT + W_same), X_lastT + W_same);
                                else
                                    Y_lastT = max(B[0], Y_lastT + W_same);
                            }
                            else
                            {
                                if (X_lastFrom == 'B')
                                    Y_lastT = max(max(B[0], Y_lastT + W_diff), X_lastT + W_same);
                                else
                                    Y_lastT = max(B[0], Y_lastT + W_diff);
                            }
                            Y_lastFrom = 'B';
                            total_wait += (Y_lastT - B[0]);
                            B.erase(B.begin());
                        }
                    }
                }
            }
        }
        else if (A.size() > 0 && B.size() > 0)  // if Lane C is empty
        {
            // Let all vehicles on Lane A go to Lane X
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X);
            total_wait += wait_time;

            // Let all vehicles on Lane B go to Lane Y
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('B', B, last_Y);
            total_wait += wait_time;

            // Clear vectors
            A.clear();
            B.clear();
        }
        else if (A.size() > 0 && C.size() > 0)  // if Lane B is empty
        {
            // Let all vehicles on Lane A go to Lane X
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X);
            total_wait += wait_time;

            // Let all vehicles on Lane C go to Lane Y
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y);
            total_wait += wait_time;

            A.clear();
            C.clear();
        }
        else if (B.size() > 0 && C.size() > 0)  // if Lane A is empty
        {
            // Let all vehicles on Lane B go to Lane X
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('B', B, last_X);
            total_wait += wait_time;

            // Let all vehicles on Lane C go to Lane Y
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y);
            total_wait += wait_time;

            B.clear();
            C.clear();
        }
        else if (A.size() > 0)   // if only Lane A is not empty
        {
            // Let all vehicles on Lane A go to Lane X
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X);
            total_wait += wait_time;
            A.clear();
        }
        else if (C.size() > 0)  // if only Lane C is not empty
        {
            // Let all vehicles on Lane C go to Lane Y
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y);
            total_wait += wait_time;
            C.clear();
        }
        else if (B.size() > 0)  // if only Lane B is not empty
        {
            while (B.size() > 0)
            {
                if (X_lastT < Y_lastT)  // if the previous vehicle going to Lane X passes earlier than the previous vehicle going to Lane Y
                {                       // then let the first vehicle go to Lane X
                    if (X_lastFrom == 'B')
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0], X_lastT + W_same), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0], X_lastT + W_same);
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0], X_lastT + W_diff), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0], X_lastT + W_diff);
                    }
                    X_lastFrom = 'B';
                    total_wait += (X_lastT - B[0]);
                    B.erase(B.begin());
                }
                else    // else let the first vehicle go to Lane Y
                {
                    if (Y_lastFrom == 'B')
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0], Y_lastT + W_same), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0], Y_lastT + W_same);
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0], Y_lastT + W_diff), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0], Y_lastT + W_diff);
                    }
                    Y_lastFrom = 'B';
                    total_wait += (Y_lastT - B[0]);
                    B.erase(B.begin());
                }
            }
        }
    }
    double T_last = max(max(get<2>(last_X), get<2>(last_Y)), max(X_lastT, Y_lastT));    // scheduled entering time of the last vehicle
    double T_delay = total_wait / vehicle_num;  // average difference betweeb each vehicle's scheduled entering time and its earliest arrival time
    auto t1 = chrono::high_resolution_clock::now(); // ending time of computation
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();  // computation time
    totalComputeTime *= 1e-9;

    return {T_last, T_delay, totalComputeTime};
}
