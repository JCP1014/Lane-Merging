#include "fcfs.h"

void fcfs_compute_entering_time(vector<vehicle> a_all, vector<vehicle> b_all, vector<vehicle> c_all, vector<vehicle> &schedule_A, vector<vehicle> &schedule_B, vector<vehicle> &schedule_C)
{
    vector<vehicle> a(a_all.begin() + 1, a_all.end());
    vector<vehicle> b(b_all.begin() + 1, b_all.end());
    vector<vehicle> c(c_all.begin() + 1, c_all.end());
    double X_lastT = -W_diff;
    double Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';
    tuple<char, int, double> last_X, last_Y;
    double first = 0;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    double total_wait = 0;
    double wait_time = 0;
    int vehicle_num = a_all.size() + b_all.size() + c_all.size() - 3;

    schedule_A.clear();
    schedule_B.clear();
    schedule_C.clear();
    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            first = min(min(a[0].time, b[0].time), c[0].time);
            if (first == a[0].time)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0].time, X_lastT + W_same);
                else
                    X_lastT = max(a[0].time, X_lastT + W_diff);
                X_lastFrom = 'A';
                schedule_A.push_back(vehicle(a[0].id, X_lastT));
                total_wait += (X_lastT - a[0].time);
                a.erase(a.begin());
            }
            if (first == c[0].time)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0].time, Y_lastT + W_same);
                else
                    Y_lastT = max(c[0].time, Y_lastT + W_diff);
                Y_lastFrom = 'C';
                schedule_C.push_back(vehicle(c[0].id, Y_lastT));
                total_wait += (Y_lastT - c[0].time);
                c.erase(c.begin());
            }
            if (first == b[0].time)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(b[0].time, X_lastT + W_same), Y_lastT + W_same);
                        else
                            X_lastT = max(b[0].time, X_lastT + W_same);
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(b[0].time, X_lastT + W_diff), Y_lastT + W_same);
                        else
                            X_lastT = max(b[0].time, X_lastT + W_diff);
                    }
                    X_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_1");
                    schedule_B.push_back(vehicle(b[0].id, X_lastT));
                    total_wait += (X_lastT - b[0].time);
                    b.erase(b.begin());
                }
                else if (Y_lastT < X_lastT)
                {
                    if (Y_lastFrom == 'B')
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(b[0].time, Y_lastT + W_same), X_lastT + W_same);
                        else
                            Y_lastT = max(b[0].time, Y_lastT + W_same);
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(b[0].time, Y_lastT + W_diff), X_lastT + W_same);
                        else
                            Y_lastT = max(b[0].time, Y_lastT + W_diff);
                    }
                    Y_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_2");
                    schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                    total_wait += (Y_lastT - b[0].time);
                    b.erase(b.begin());
                }
                else
                {
                    if (a[0].time > c[0].time)
                    {
                        if (X_lastFrom == 'B')
                        {
                            if (Y_lastFrom == 'B')
                                X_lastT = max(max(b[0].time, X_lastT + W_same), Y_lastT + W_same);
                            else
                                X_lastT = max(b[0].time, X_lastT + W_same);
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                X_lastT = max(max(b[0].time, X_lastT + W_diff), Y_lastT + W_same);
                            else
                                X_lastT = max(b[0].time, X_lastT + W_diff);
                        }
                        X_lastFrom = 'B';
                        Vehicle::setRouteID(b[0].id, "route_1");
                        schedule_B.push_back(vehicle(b[0].id, X_lastT));
                        total_wait += (X_lastT - b[0].time);
                        b.erase(b.begin());
                    }
                    else if (c[0].time > a[0].time)
                    {
                        if (Y_lastFrom == 'B')
                        {
                            if (X_lastFrom == 'B')
                                Y_lastT = max(max(b[0].time, Y_lastT + W_same), X_lastT + W_same);
                            else
                                Y_lastT = max(b[0].time, Y_lastT + W_same);
                        }
                        else
                        {
                            if (X_lastFrom == 'B')
                                Y_lastT = max(max(b[0].time, Y_lastT + W_diff), X_lastT + W_same);
                            else
                                Y_lastT = max(b[0].time, Y_lastT + W_diff);
                        }
                        Y_lastFrom = 'B';
                        Vehicle::setRouteID(b[0].id, "route_2");
                        schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                        total_wait += (Y_lastT - b[0].time);
                        b.erase(b.begin());
                    }
                    else
                    {
                        if (distribution(generator) == 0)
                        {
                            if (X_lastFrom == 'B')
                            {
                                if (Y_lastFrom == 'B')
                                    X_lastT = max(max(b[0].time, X_lastT + W_same), Y_lastT + W_same);
                                else
                                    X_lastT = max(b[0].time, X_lastT + W_same);
                            }
                            else
                            {
                                if (Y_lastFrom == 'B')
                                    X_lastT = max(max(b[0].time, X_lastT + W_diff), Y_lastT + W_same);
                                else
                                    X_lastT = max(b[0].time, X_lastT + W_diff);
                            }
                            X_lastFrom = 'B';
                            Vehicle::setRouteID(b[0].id, "route_1");
                            schedule_B.push_back(vehicle(b[0].id, X_lastT));
                            total_wait += (X_lastT - b[0].time);
                            b.erase(b.begin());
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                            {
                                if (X_lastFrom == 'B')
                                    Y_lastT = max(max(b[0].time, Y_lastT + W_same), X_lastT + W_same);
                                else
                                    Y_lastT = max(b[0].time, Y_lastT + W_same);
                            }
                            else
                            {
                                if (X_lastFrom == 'B')
                                    Y_lastT = max(max(b[0].time, Y_lastT + W_diff), X_lastT + W_same);
                                else
                                    Y_lastT = max(b[0].time, Y_lastT + W_diff);
                            }
                            Y_lastFrom = 'B';
                            Vehicle::setRouteID(b[0].id, "route_2");
                            schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                            total_wait += (Y_lastT - b[0].time);
                            b.erase(b.begin());
                        }
                    }
                }
            }
        }
        else if (a.size() > 0 && b.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X, schedule_A);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('B', b, last_Y, schedule_B);
            for (auto veh : b)
                Vehicle::setRouteID(veh.id, "route_2");
            total_wait += wait_time;
            a.clear();
            b.clear();
        }
        else if (a.size() > 0 and c.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X, schedule_A);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y, schedule_C);
            total_wait += wait_time;
            a.clear();
            c.clear();
        }
        else if (b.size() > 0 and c.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('B', b, last_X, schedule_B);
            for (auto veh : b)
                Vehicle::setRouteID(veh.id, "route_1");
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y, schedule_C);
            total_wait += wait_time;
            b.clear();
            c.clear();
        }
        else if (a.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X, schedule_A);
            total_wait += wait_time;
            a.clear();
        }
        else if (c.size() > 0)
        {
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y, schedule_C);
            total_wait += wait_time;
            c.clear();
        }
        else if (b.size() > 0)
        {
            while (b.size() > 0)
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(b[0].time, X_lastT + W_same), Y_lastT + W_same);
                        else
                            X_lastT = max(b[0].time, X_lastT + W_same);
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            X_lastT = max(max(b[0].time, X_lastT + W_diff), Y_lastT + W_same);
                        else
                            X_lastT = max(b[0].time, X_lastT + W_diff);
                    }
                    X_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_1");
                    schedule_B.push_back(vehicle(b[0].id, X_lastT));
                    total_wait += (X_lastT - b[0].time);
                    b.erase(b.begin());
                }
                else
                {
                    if (Y_lastFrom == 'B')
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(b[0].time, Y_lastT + W_same), X_lastT + W_same);
                        else
                            Y_lastT = max(b[0].time, Y_lastT + W_same);
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            Y_lastT = max(max(b[0].time, Y_lastT + W_diff), X_lastT + W_same);
                        else
                            Y_lastT = max(b[0].time, Y_lastT + W_diff);
                    }
                    Y_lastFrom = 'B';
                    Vehicle::setRouteID(b[0].id, "route_2");
                    schedule_B.push_back(vehicle(b[0].id, Y_lastT));
                    total_wait += (Y_lastT - b[0].time);
                    b.erase(b.begin());
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    // cout << "fcfs result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
}
