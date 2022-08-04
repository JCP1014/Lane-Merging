#include "fafg.h"

void fcfs_compute_entering_time(vector<vehicle> A_all, vector<vehicle> B_all, vector<vehicle> C_all, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<vehicle> A(A_all.begin() + 1, A_all.end());
    vector<vehicle> B(B_all.begin() + 1, B_all.end());
    vector<vehicle> C(C_all.begin() + 1, C_all.end());
    double X_lastT = -W_diff;
    double Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';
    tuple<char, int, double> last_X, last_Y;
    double first = 0;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    double wait_time = 0;
    int vehicle_num = A_all.size() + B_all.size() + C_all.size() - 3;

    schedule_A.clear();
    schedule_B.clear();
    schedule_C.clear();
    while (A.size() > 0 || B.size() > 0 || C.size() > 0)
    {
        if (A.size() > 0 && B.size() > 0 && C.size() > 0)
        {
            first = min(min(A[0].time, B[0].time), C[0].time);
            if (first == A[0].time)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(A[0].time, X_lastT + W_same);
                else
                    X_lastT = max(A[0].time, X_lastT + W_diff);
                X_lastFrom = 'A';
                schedule_A.push_back(vehicle(A[0].id, X_lastT));
                A.erase(A.begin());
            }
            if (first == C[0].time)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(C[0].time, Y_lastT + W_same);
                else
                    Y_lastT = max(C[0].time, Y_lastT + W_diff);
                Y_lastFrom = 'C';
                schedule_C.push_back(vehicle(C[0].id, Y_lastT));
                C.erase(C.begin());
            }
            if (first == B[0].time)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0].time, X_lastT + W_same), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0].time, X_lastT + W_same);
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0].time, X_lastT + W_diff), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0].time, X_lastT + W_diff);
                    }
                    X_lastFrom = 'B';
                    Vehicle::setRouteID(B[0].id, "route_1");
                    schedule_B.push_back(vehicle(B[0].id, X_lastT));
                    B.erase(B.begin());
                }
                else if (Y_lastT < X_lastT)
                {
                    if (Y_lastFrom == 'B')
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0].time, Y_lastT + W_same), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0].time, Y_lastT + W_same);
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0].time, Y_lastT + W_diff), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0].time, Y_lastT + W_diff);
                    }
                    Y_lastFrom = 'B';
                    Vehicle::setRouteID(B[0].id, "route_2");
                    schedule_B.push_back(vehicle(B[0].id, Y_lastT));
                    B.erase(B.begin());
                }
                else
                {
                    if (A[0].time > C[0].time)
                    {
                        if (X_lastFrom == 'B')
                        {
                            if (Y_lastFrom == 'B')
                                X_lastT = max(max(B[0].time, X_lastT + W_same), Y_lastT + W_same);
                            else
                                X_lastT = max(B[0].time, X_lastT + W_same);
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                X_lastT = max(max(B[0].time, X_lastT + W_diff), Y_lastT + W_same);
                            else
                                X_lastT = max(B[0].time, X_lastT + W_diff);
                        }
                        X_lastFrom = 'B';
                        Vehicle::setRouteID(B[0].id, "route_1");
                        schedule_B.push_back(vehicle(B[0].id, X_lastT));
                        B.erase(B.begin());
                    }
                    else if (C[0].time > A[0].time)
                    {
                        if (Y_lastFrom == 'B')
                        {
                            if (X_lastFrom == 'B')
                                Y_lastT = max(max(B[0].time, Y_lastT + W_same), X_lastT + W_same);
                            else
                                Y_lastT = max(B[0].time, Y_lastT + W_same);
                        }
                        else
                        {
                            if (X_lastFrom == 'B')
                                Y_lastT = max(max(B[0].time, Y_lastT + W_diff), X_lastT + W_same);
                            else
                                Y_lastT = max(B[0].time, Y_lastT + W_diff);
                        }
                        Y_lastFrom = 'B';
                        Vehicle::setRouteID(B[0].id, "route_2");
                        schedule_B.push_back(vehicle(B[0].id, Y_lastT));
                        B.erase(B.begin());
                    }
                    else
                    {
                        if (distribution(generator) == 0)
                        {
                            if (X_lastFrom == 'B')
                            {
                                if (Y_lastFrom == 'B')
                                    X_lastT = max(max(B[0].time, X_lastT + W_same), Y_lastT + W_same);
                                else
                                    X_lastT = max(B[0].time, X_lastT + W_same);
                            }
                            else
                            {
                                if (Y_lastFrom == 'B')
                                    X_lastT = max(max(B[0].time, X_lastT + W_diff), Y_lastT + W_same);
                                else
                                    X_lastT = max(B[0].time, X_lastT + W_diff);
                            }
                            X_lastFrom = 'B';
                            Vehicle::setRouteID(B[0].id, "route_1");
                            schedule_B.push_back(vehicle(B[0].id, X_lastT));
                            B.erase(B.begin());
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                            {
                                if (X_lastFrom == 'B')
                                    Y_lastT = max(max(B[0].time, Y_lastT + W_same), X_lastT + W_same);
                                else
                                    Y_lastT = max(B[0].time, Y_lastT + W_same);
                            }
                            else
                            {
                                if (X_lastFrom == 'B')
                                    Y_lastT = max(max(B[0].time, Y_lastT + W_diff), X_lastT + W_same);
                                else
                                    Y_lastT = max(B[0].time, Y_lastT + W_diff);
                            }
                            Y_lastFrom = 'B';
                            Vehicle::setRouteID(B[0].id, "route_2");
                            schedule_B.push_back(vehicle(B[0].id, Y_lastT));
                            B.erase(B.begin());
                        }
                    }
                }
            }
        }
        else if (A.size() > 0 && B.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X, schedule_A);
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('B', B, last_Y, schedule_B);
            for (auto veh : B)
                Vehicle::setRouteID(veh.id, "route_2");
            A.clear();
            B.clear();
        }
        else if (A.size() > 0 and C.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X, schedule_A);
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y, schedule_C);
            A.clear();
            C.clear();
        }
        else if (B.size() > 0 and C.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('B', B, last_X, schedule_B);
            for (auto veh : B)
                Vehicle::setRouteID(veh.id, "route_1");
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y, schedule_C);
            B.clear();
            C.clear();
        }
        else if (A.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', A, last_X, schedule_A);
            A.clear();
        }
        else if (C.size() > 0)
        {
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', C, last_Y, schedule_C);
            C.clear();
        }
        else if (B.size() > 0)
        {
            while (B.size() > 0)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0].time, X_lastT + W_same), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0].time, X_lastT + W_same);
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(B[0].time, X_lastT + W_diff), Y_lastT + W_same);
                        else
                            X_lastT = max(B[0].time, X_lastT + W_diff);
                    }
                    X_lastFrom = 'B';
                    Vehicle::setRouteID(B[0].id, "route_1");
                    schedule_B.push_back(vehicle(B[0].id, X_lastT));
                    B.erase(B.begin());
                }
                else
                {
                    if (Y_lastFrom == 'B')
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0].time, Y_lastT + W_same), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0].time, Y_lastT + W_same);
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(B[0].time, Y_lastT + W_diff), X_lastT + W_same);
                        else
                            Y_lastT = max(B[0].time, Y_lastT + W_diff);
                    }
                    Y_lastFrom = 'B';
                    Vehicle::setRouteID(B[0].id, "route_2");
                    schedule_B.push_back(vehicle(B[0].id, Y_lastT));
                    B.erase(B.begin());
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    // cout << "fcfs result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
}
