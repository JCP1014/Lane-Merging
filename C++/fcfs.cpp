#include "fcfs.h"

float W_same, W_diff;

tuple<tuple<char, int, float>, float> schedule_single_lane(char lane, vector<float> traffic, tuple<char, int, float> prev)
{
    char prevLane = get<0>(prev);
    float prevTime = get<2>(prev);
    tuple<char, int, float> last = make_tuple('0', 0, 0.0);
    float wait_time = 0;
    if (prevLane == '0')
    {
        last = make_tuple(lane, 1, traffic[0]);
    }
    else if (lane == prevLane)
    {
        last = make_tuple(lane, 1, max(traffic[0], prevTime + W_same));
    }
    else
    {
        last = make_tuple(lane, 1, max(traffic[0], prevTime + W_diff));
    }
    wait_time += (get<2>(last) - traffic[0]);
    for (int i = 1; i < traffic.size(); ++i)
    {
        prevTime = get<2>(last);
        last = make_tuple(lane, i, max(traffic[i], prevTime + W_same));
        wait_time += (get<2>(last) - traffic[i]);
    }
    return make_tuple(last, wait_time);
}

pair<float, double> first_come_first_serve_v1(float timeStep, vector<float> a_all, vector<float> b_all, vector<float> c_all)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<float> a(a_all.begin() + 1, a_all.end());
    vector<float> b(b_all.begin() + 1, b_all.end());
    vector<float> c(c_all.begin() + 1, c_all.end());
    float t = 0;         // time
    char B_prevTo = 'Y'; // Which lane the previous vehicle in lane B go to
    float X_lastT = -W_diff;
    float Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';

    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        t = t + timeStep;
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            if (a[0] == t && b[0] == t && c[0] == t)
            {
                if (a.size() > 1 && b.size() > 1 && c.size() > 1)
                {
                    if (b[1] >= a[1] && b[1] >= c[1])
                    {
                        if (X_lastFrom == 'A')
                            X_lastT = max(a[0], X_lastT + W_same);
                        else
                            X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';

                        if (Y_lastFrom == 'C')
                            Y_lastT = max(c[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';

                        if (X_lastT <= t && Y_lastT <= t)
                        {
                            if (a[0] >= c[0])
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                        }
                        else
                        {
                            if (X_lastT < Y_lastT)
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else if (Y_lastT < X_lastT)
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (a[0] >= c[0])
                                {
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                    b.erase(b.begin());
                                    X_lastFrom = 'B';
                                }
                                else
                                {
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                    b.erase(b.begin());
                                    Y_lastFrom = 'B';
                                }
                            }
                        }
                    }
                    else if (a[1] >= b[1] && a[1] >= c[1])
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';

                        if (Y_lastFrom == 'C')
                            Y_lastT = max(c[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';

                        X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';
                    }
                    else if (c[1] >= a[1] && c[1] >= b[1])
                    {
                        if (X_lastFrom == 'A')
                            X_lastT = max(a[0], X_lastT + W_same);
                        else
                            X_lastT = max(a[0], X_lastT + W_diff);
                        a.erase(a.begin());
                        X_lastFrom = 'A';

                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';

                        Y_lastT = max(c[0], Y_lastT + W_diff);
                        c.erase(c.begin());
                        Y_lastFrom = 'C';
                    }
                }
                else if (b.size() < 2)
                {
                    if (X_lastFrom == 'A')
                        X_lastT = max(a[0], X_lastT + W_same);
                    else
                        X_lastT = max(a[0], X_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';

                    if (Y_lastFrom == 'C')
                        Y_lastT = max(c[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';

                    if (X_lastT <= t && Y_lastT <= t)
                    {
                        if (a.size() < 1)
                        {
                            X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (c.size() < 1)
                        {
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (a[0] >= c[0])
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                        }
                    }
                    else
                    {
                        if (X_lastT < Y_lastT)
                        {
                            X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (Y_lastT < X_lastT)
                        {
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (a.size() < 1)
                            {
                                X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                            else if (c.size() < 1)
                            {
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (a[0] >= c[0])
                                {
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                    b.erase(b.begin());
                                    X_lastFrom = 'B';
                                }
                                else
                                {
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                    b.erase(b.begin());
                                    Y_lastFrom = 'B';
                                }
                            }
                        }
                    }
                }
                else if (a.size() < 2)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    b.erase(b.begin());
                    X_lastFrom = 'B';

                    if (Y_lastFrom == 'C')
                        Y_lastT = max(c[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';

                    X_lastT = max(a[0], Y_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';
                }
                else if (c.size() < 2)
                {
                    if (X_lastFrom == 'A')
                        X_lastT = max(a[0], X_lastT + W_same);
                    else
                        X_lastT = max(a[0], X_lastT + W_diff);
                    a.erase(a.begin());
                    X_lastFrom = 'A';

                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';

                    Y_lastT = max(c[0], Y_lastT + W_diff);
                    c.erase(c.begin());
                    Y_lastFrom = 'C';
                }
            }
            else if (a[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (a[0] == t && b[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
            }
            else if (b[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'B')
                    X_lastT = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (c[0] > a[0])
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                }
                else
                {
                    if (X_lastT <= t && Y_lastT <= t)
                    {
                        if (c[0] > a[0])
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                    }
                    else
                    {
                        if (X_lastT < Y_lastT)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            b.erase(b.begin());
                            X_lastFrom = 'B';
                        }
                        else if (Y_lastT < X_lastT)
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            b.erase(b.begin());
                            Y_lastFrom = 'B';
                        }
                        else
                        {
                            if (c[0] > a[0])
                            {
                                if (Y_lastFrom == 'B')
                                    Y_lastT = max(b[0], Y_lastT + W_same);
                                else
                                    Y_lastT = max(b[0], Y_lastT + W_diff);
                                b.erase(b.begin());
                                Y_lastFrom = 'B';
                            }
                            else
                            {
                                if (X_lastFrom == 'B')
                                    X_lastT = max(b[0], X_lastT + W_same);
                                else
                                    X_lastT = max(b[0], X_lastT + W_diff);
                                b.erase(b.begin());
                                X_lastFrom = 'B';
                            }
                        }
                    }
                }
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
        }
        else if (a.size() > 0 && c.size() > 0)
        {
            if (a[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
        }
        else if (a.size() > 0 && b.size() > 0)
        {
            if (a[0] == t && b[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';

                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
            }
            else if (a[0] == t)
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                a.erase(a.begin());
                X_lastFrom = 'A';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    b.erase(b.begin());
                    Y_lastFrom = 'B';
                }
                else
                {
                    if (X_lastT < Y_lastT)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                    else if (Y_lastT < X_lastT)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                }
            }
        }
        else if (b.size() > 0 && c.size() > 0)
        {
            if (b[0] == t && c[0] == t)
            {
                if (X_lastFrom == 'B')
                    X_lastFrom = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';

                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (c[0] == t)
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                c.erase(c.begin());
                Y_lastFrom = 'C';
            }
            else if (b[0] == t)
            {
                if (X_lastT <= t && Y_lastT <= t)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    b.erase(b.begin());
                    X_lastFrom = 'B';
                }
                else
                {
                    if (X_lastT < Y_lastT)
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                    else if (Y_lastT < X_lastT)
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        b.erase(b.begin());
                        Y_lastFrom = 'B';
                    }
                    else
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        b.erase(b.begin());
                        X_lastFrom = 'B';
                    }
                }
            }
        }
        else if (a.size() > 0)
        {
            if (X_lastFrom == 'A')
                X_lastT = max(a[0], X_lastT + W_same);
            else
                X_lastT = max(a[0], X_lastT + W_diff);
            a.erase(a.begin());
            X_lastFrom = 'A';
        }
        else if (c.size() > 0)
        {
            if (Y_lastFrom == 'C')
                Y_lastT = max(c[0], Y_lastT + W_same);
            else
                Y_lastT = max(c[0], Y_lastT + W_diff);
            c.erase(c.begin());
            Y_lastFrom = 'C';
        }
        else if (b.size() > 0)
        {
            if (B_prevTo == 'Y')
            {
                if (X_lastFrom == 'B')
                    X_lastT = max(b[0], X_lastT + W_same);
                else
                    X_lastT = max(b[0], X_lastT + W_diff);
                b.erase(b.begin());
                X_lastFrom = 'B';
                B_prevTo = 'X';
            }
            else
            {
                if (Y_lastFrom == 'B')
                    Y_lastT = max(b[0], Y_lastT + W_same);
                else
                    Y_lastT = max(b[0], Y_lastT + W_diff);
                b.erase(b.begin());
                Y_lastFrom = 'B';
                B_prevTo = 'Y';
            }
        }
    }
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    float T_last = max(X_lastT, Y_lastT);
    cout << "fcfs result: " << T_last << " " << totalComputeTime << endl;
    return {T_last, totalComputeTime};
}

tuple<float, float, double> first_come_first_serve_v2(vector<float> a_all, vector<float> b_all, vector<float> c_all)
{
    auto t0 = chrono::high_resolution_clock::now();
    vector<float> a(a_all.begin() + 1, a_all.end());
    vector<float> b(b_all.begin() + 1, b_all.end());
    vector<float> c(c_all.begin() + 1, c_all.end());
    float X_lastT = -W_diff;
    float Y_lastT = -W_diff;
    char X_lastFrom = '0';
    char Y_lastFrom = '0';
    tuple<char, int, float> last_X, last_Y;
    float first = 0;
    default_random_engine generator(time(NULL));
    uniform_int_distribution<int> distribution(0, 1);
    float total_wait = 0;
    float wait_time = 0;
    int vehicle_num = a_all.size() + b_all.size() + c_all.size() - 3;

    while (a.size() > 0 || b.size() > 0 || c.size() > 0)
    {
        if (a.size() > 0 && b.size() > 0 && c.size() > 0)
        {
            first = min(min(a[0], b[0]), c[0]);
            if (first == a[0])
            {
                if (X_lastFrom == 'A')
                    X_lastT = max(a[0], X_lastT + W_same);
                else
                    X_lastT = max(a[0], X_lastT + W_diff);
                X_lastFrom = 'A';
                total_wait += (X_lastT - a[0]);
                a.erase(a.begin());
            }
            if (first == c[0])
            {
                if (Y_lastFrom == 'C')
                    Y_lastT = max(c[0], Y_lastT + W_same);
                else
                    Y_lastT = max(c[0], Y_lastT + W_diff);
                Y_lastFrom = 'C';
                total_wait += (Y_lastT - c[0]);
                c.erase(c.begin());
            }
            if (first == b[0])
            {
                if (X_lastT < Y_lastT)
                {
                    if (X_lastFrom == 'B')
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    X_lastFrom = 'B';
                    total_wait += (X_lastT - b[0]);
                    b.erase(b.begin());
                }
                else if (Y_lastT < X_lastT)
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    total_wait += (Y_lastT - b[0]);
                    b.erase(b.begin());
                }
                else
                {
                    if (a[0] > c[0])
                    {
                        if (X_lastFrom == 'B')
                            X_lastT = max(b[0], X_lastT + W_same);
                        else
                            X_lastT = max(b[0], X_lastT + W_diff);
                        X_lastFrom = 'B';
                        total_wait += (X_lastT - b[0]);
                        b.erase(b.begin());
                    }
                    else if (c[0] > a[0])
                    {
                        if (Y_lastFrom == 'B')
                            Y_lastT = max(b[0], Y_lastT + W_same);
                        else
                            Y_lastT = max(b[0], Y_lastT + W_diff);
                        Y_lastFrom = 'B';
                        total_wait += (Y_lastT - b[0]);
                        b.erase(b.begin());
                    }
                    else
                    {
                        if (distribution(generator) == 0)
                        {
                            if (X_lastFrom == 'B')
                                X_lastT = max(b[0], X_lastT + W_same);
                            else
                                X_lastT = max(b[0], X_lastT + W_diff);
                            X_lastFrom = 'B';
                            total_wait += (X_lastT - b[0]);
                            b.erase(b.begin());
                        }
                        else
                        {
                            if (Y_lastFrom == 'B')
                                Y_lastT = max(b[0], Y_lastT + W_same);
                            else
                                Y_lastT = max(b[0], Y_lastT + W_diff);
                            Y_lastFrom = 'B';
                            total_wait += (Y_lastT - b[0]);
                            b.erase(b.begin());
                        }
                    }
                }
            }
        }
        else if (a.size() > 0 && b.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('B', b, last_Y);
            total_wait += wait_time;
            a.clear();
            b.clear();
        }
        else if (a.size() > 0 and c.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y);
            total_wait += wait_time;
            a.clear();
            c.clear();
        }
        else if (b.size() > 0 and c.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('B', b, last_X);
            total_wait += wait_time;
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y);
            total_wait += wait_time;
            b.clear();
            c.clear();
        }
        else if (a.size() > 0)
        {
            last_X = make_tuple(X_lastFrom, 0, X_lastT);
            tie(last_X, wait_time) = schedule_single_lane('A', a, last_X);
            total_wait += wait_time;
            a.clear();
        }
        else if (c.size() > 0)
        {
            last_Y = make_tuple(Y_lastFrom, 0, Y_lastT);
            tie(last_Y, wait_time) = schedule_single_lane('C', c, last_Y);
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
                        X_lastT = max(b[0], X_lastT + W_same);
                    else
                        X_lastT = max(b[0], X_lastT + W_diff);
                    X_lastFrom = 'B';
                    total_wait += (X_lastT - b[0]);
                    b.erase(b.begin());
                }
                else
                {
                    if (Y_lastFrom == 'B')
                        Y_lastT = max(b[0], Y_lastT + W_same);
                    else
                        Y_lastT = max(b[0], Y_lastT + W_diff);
                    Y_lastFrom = 'B';
                    total_wait += (Y_lastT - b[0]);
                    b.erase(b.begin());
                }
            }
        }
        // cout << "last_X: " << get<0>(last_X) << " " << get<1>(last_X) << " " << get<2>(last_X) << endl;
        // cout << "last_Y: " << get<0>(last_Y) << " " << get<1>(last_Y) << " " << get<2>(last_Y) << endl;
    }
    float T_last = max(max(get<2>(last_X), get<2>(last_Y)), max(X_lastT, Y_lastT));
    float T_delay = total_wait / vehicle_num;
    auto t1 = chrono::high_resolution_clock::now();
    double totalComputeTime = chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count();
    totalComputeTime *= 1e-9;
    // cout << "fcfs result: " << T_last << " " << T_delay << " " << totalComputeTime << endl;
    return {T_last, T_delay, totalComputeTime};
}
